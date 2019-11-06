#include "rene/IKProvider.h"
#include "rene/StateCostProvider.h"
#include "rene/Viterbi.hpp"

#include <nav_msgs/Path.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <geometry_msgs/TwistStamped.h>

#include <std_srvs/Trigger.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

#include <tf/transform_listener.h>

#include <pluginlib/class_loader.h>

#include <Eigen/Geometry>
#include <tf/transform_broadcaster.h>
#include <rene/JointHomeCostProvider.h>

// TODO: Replace with moveit-loaded values

std::vector<double> JOINT_WEIGHTS;
std::vector<double> JOINT_HOME;

template <typename T>
void debugClassLoader(pluginlib::ClassLoader<T>& loader, const std::string& subclassName)
{
	loader.refreshDeclaredClasses();
	std::cerr << "Declared Plugin XML files:" << std::endl;
	for (const auto& c : loader.getPluginXmlPaths()) { std::cerr << "\t" << c << std::endl; }
	std::cerr << "Declared Classes:" << std::endl;
	for (const auto& c : loader.getDeclaredClasses()) { std::cerr << "\t" << c << std::endl; }
	std::cerr << "Registered Libraries:" << std::endl;
	for (const auto& c : loader.getRegisteredLibraries()) { std::cerr << "\t"  << c << std::endl; }
	std::cerr << "Available? " << loader.isClassAvailable(subclassName) << std::endl;
	std::cerr << "Loaded? " << loader.isClassLoaded(subclassName) << std::endl;
	std::cerr << "Class package: '" << loader.getClassPackage(subclassName) << "'" << std::endl;
	std::cerr << "Class manifest path: '" << loader.getPluginManifestPath(subclassName) << "'" << std::endl;
	std::cerr << "Class library path: '" << loader.getClassLibraryPath(subclassName) << "'" << std::endl;
	std::cerr << "Class type: '" << loader.getClassType(subclassName) << "'" << std::endl;
	std::cerr << "Class description: '" << loader.getClassDescription(subclassName) << "'" << std::endl;
	std::cerr << "Class name: '" << loader.getName(subclassName) << "'" << std::endl;
	try
	{
		loader.loadLibraryForClass(subclassName);
		std::cerr << "Loaded? " << loader.isClassLoaded(subclassName) << std::endl;
	}
	catch (pluginlib::LibraryLoadException& ex)
	{
		std::cerr << ex.what() << std::endl;
		std::cerr << "Loaded (Exception)? " << loader.isClassLoaded(subclassName) << std::endl;
	}
}


double transitionCost(const std::vector<double>& q1, const double t1, const std::vector<double>& q2, const double t2)
{
	assert(JOINT_WEIGHTS.size() == q1.size());
	assert(JOINT_WEIGHTS.size() == q2.size());
	assert(t2 > t1);
	double cost = 0;
	for (size_t j = 0; j < q1.size(); ++j)
	{
		cost += fabs(q1[j] - q2[j]) * JOINT_WEIGHTS[j];
	}
	return cost/(t2-t1);
}

//#define DEBUG_TRANSFORMS

ros::Publisher jointVizPub, jointPub;
control_msgs::FollowJointTrajectoryGoalPtr traj_ptr;
std::shared_ptr<rene::IKProvider> ikProvider;
std::shared_ptr<rene::StateCostProvider> stateCostProvider;
std::shared_ptr<tf::TransformListener> tl;
std::string tool_frame_id;

#ifdef DEBUG_TRANSFORMS
std::shared_ptr<tf::TransformBroadcaster> tb;
#endif

void pathCallback(const nav_msgs::PathPtr& path_ptr)
{
	assert(ikProvider);

	const std::vector<geometry_msgs::PoseStamped>& poses = path_ptr->poses;
	const std::string path_frame = poses.front().header.frame_id;

	if (poses.size() < 2)
	{
		ROS_ERROR("Not enough poses to produce a trajectory");
		return;
	}

	if (!tl->waitForTransform(ikProvider->getSolverBaseFrame(), path_frame, ros::Time::now(), ros::Duration(10.0)))
	{
		ROS_ERROR_STREAM("Failed to look up transform '" << path_frame << "'->'" << ikProvider->getSolverBaseFrame() << "'.");
		return;
	}
	tf::StampedTransform solverTpath;
	tl->lookupTransform(ikProvider->getSolverBaseFrame(), path_frame, ros::Time(0), solverTpath);

	tf::StampedTransform toolTsolverOut;
	if (tool_frame_id.empty())
	{
		toolTsolverOut.setIdentity();
	}
	else
	{
		if (!tl->waitForTransform(tool_frame_id, ikProvider->getSolverFinalFrame(), ros::Time::now(), ros::Duration(10.0)))
		{
			ROS_ERROR_STREAM("Failed to look up transform '" << tool_frame_id << "'->'" << ikProvider->getSolverFinalFrame() << "'.");
			return;
		}
		tl->lookupTransform(tool_frame_id, ikProvider->getSolverFinalFrame(), ros::Time(0), toolTsolverOut);
	}

	const double t0 = poses.front().header.stamp.toSec();
	const double tF = poses.back().header.stamp.toSec();

	// Time step, solution #, joint #
	std::vector<std::vector<std::vector<double>>> trellis;
	std::vector<double> times;
//	std::vector<trajectory_msgs::JointTrajectory> jointTrajectories(8);
	for (int i = 0; i < static_cast<int>(poses.size()); ++i)
	{
		// t : [t0, tF]
		double t = poses[i].header.stamp.toSec();
		times.push_back(t);

		assert(t >= 0); assert(t >= t0); assert(t <= tF);

		rene::Pose goalPose = rene::Pose(solverTpath) * rene::Pose(poses[i].pose) * rene::Pose(toolTsolverOut);

#ifdef DEBUG_TRANSFORMS

		tb->sendTransform(tf::StampedTransform(rene::Pose(solverTpath).toTF(), ros::Time::now(), ikProvider->getSolverBaseFrame(), path_frame + "_target"));
		tb->sendTransform(tf::StampedTransform(rene::Pose(poses[i].pose).toTF(), ros::Time::now(), path_frame + "_target", "tool_target"));
		tb->sendTransform(tf::StampedTransform(rene::Pose(toolTsolverOut).toTF(), ros::Time::now(), "tool_target", "solver_output_target"));
//		tb->sendTransform(tf::StampedTransform(rene::Pose(toolTsolverOut).inv().toTF(), ros::Time::now(), ikProvider->getSolverFinalFrame(), "tool_frame_gathered"));
		tb->sendTransform(tf::StampedTransform(goalPose.toTF(), ros::Time::now(), ikProvider->getSolverBaseFrame(), ikProvider->getSolverFinalFrame() + "_target"));
		ros::Duration(0.1).sleep();
#endif

		rene::IKProvider::SolutionContainer frameSolutions = ikProvider->getSolutions(goalPose);

		if (frameSolutions.empty())
		{
			ROS_ERROR_STREAM("Step " << i << "/" << poses.size()-1 << " is unreachable.\n" << poses[i]);
			return;
		}

		std::cerr << "Step " << i << " has " << frameSolutions.size() << " solutions." << std::endl;

		trellis.push_back(frameSolutions);
		std::cerr << std::endl;
	}

	// Compute a reasonable cost threshold
	double dist = M_PI/4.0;
	std::vector<double> fromJoints(JOINT_HOME.size(), 0.0), toJoints(JOINT_HOME.size(), dist);
	double maxCost = transitionCost(fromJoints, times[0], toJoints, times[1]);
	std::cerr << "Maximum transition cost is " << maxCost << std::endl;

	const auto stateCostFn = [&](const std::vector<double>& q){ return stateCostProvider->operator()(q); };
	std::vector<int> cheapestPath = rene::viterbi(trellis, times, stateCostFn, &transitionCost, maxCost);
	if (cheapestPath.size() != poses.size())
	{
		ROS_ERROR_STREAM("Unable to compute Cartesian trajectory");
		return;
	}

	moveit_msgs::DisplayTrajectory displayTraj;
	moveit_msgs::RobotTrajectory robotTraj;
	trajectory_msgs::JointTrajectory bestTraj;

	for (int i = 0; i < static_cast<int>(poses.size()); ++i)
	{
		int slnIdx = cheapestPath[i];
		double t = times[i];

		trajectory_msgs::JointTrajectoryPoint jtpt;
		jtpt.positions = trellis[i][slnIdx];
		jtpt.time_from_start = ros::Duration(t - t0);
		bestTraj.points.push_back(jtpt);
	}
	bestTraj.joint_names = ikProvider->getSolvedJointNames();
	robotTraj.joint_trajectory = bestTraj;
	displayTraj.trajectory.push_back(robotTraj);

	// Publish all trajectories
//	for (int j = 0; j < 8; ++j)
//	{
//		for (int i = 1; i <= 6; ++i) { jointTrajectories[j].joint_names.push_back("joint_"+std::to_string(i)); }
//
//		robotTraj.joint_trajectory = jointTrajectories[j];
//		displayTraj.trajectory.push_back(robotTraj);
//	}

	jointVizPub.publish(displayTraj);

	traj_ptr = boost::make_shared<control_msgs::FollowJointTrajectoryGoal>();
	traj_ptr->trajectory = bestTraj;
}

bool triggerCallback(std_srvs::Trigger::Request &, std_srvs::Trigger::Response &res) {

	if (traj_ptr)
	{
		control_msgs::FollowJointTrajectoryActionGoal action_goal;
		action_goal.goal = *traj_ptr;
		jointPub.publish(action_goal);
		res.success = true;
	}
	else
	{
		res.success = false;
		res.message = "No Joint Trajectory ready";
	}

	return res.success;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "simple_path_trajectory");
	ros::NodeHandle node_handle, private_handle("~");

	std::string ikClassName;
	private_handle.getParam("ik_provider", ikClassName);

	pluginlib::ClassLoader<rene::IKProvider> ikLoader("rene", "rene::IKProvider");
	try
	{
		ikProvider = std::shared_ptr<rene::IKProvider>(ikLoader.createUnmanagedInstance(ikClassName));
	}
	catch (pluginlib::PluginlibException& ex)
	{
		ROS_ERROR_STREAM("The kinematics plugin '" << ikClassName << "' failed to load. Error: " << ex.what());
		debugClassLoader(ikLoader, ikClassName);
		return -1;
	}
	if (!ikProvider)
	{
		ROS_ERROR_STREAM("The kinematics plugin '" << ikClassName << "' failed to load.");
		return -1;
	}

	std::string stateCostClassName;
	private_handle.param<std::string>("state_cost_provider", stateCostClassName, "rene/JointHomeCostProvider");

	pluginlib::ClassLoader<rene::StateCostProvider> stateCostLoader("rene", "rene::StateCostProvider");
	try
	{
		stateCostProvider = std::shared_ptr<rene::StateCostProvider>(stateCostLoader.createUnmanagedInstance(stateCostClassName));
	}
	catch (pluginlib::PluginlibException& ex)
	{
		ROS_ERROR_STREAM("The cost plugin '" << stateCostClassName << "' failed to load. Error: " << ex.what());
		debugClassLoader(stateCostLoader, stateCostClassName);
		return -1;
	}
	if (!stateCostProvider)
	{
		ROS_ERROR_STREAM("The cost plugin '" << stateCostClassName << "' failed to load.");
		return -1;
	}

	stateCostProvider->configure(ikProvider.get());

	JOINT_WEIGHTS = std::vector<double>(static_cast<unsigned>(ikProvider->getActiveJointDimension()), 1.0);
	for (const auto& limits : ikProvider->getJointLimits())
	{
		JOINT_HOME.push_back((limits.max_position + limits.min_position)/2.0);
	}

	tl = std::make_shared<tf::TransformListener>();
#ifdef DEBUG_TRANSFORMS
	tb = std::make_shared<tf::TransformBroadcaster>();
#endif

	private_handle.param<std::string>("tool_frame_id", tool_frame_id, "");


	ros::Subscriber sub = node_handle.subscribe("tool_path", 1, &pathCallback);
	jointVizPub = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
	jointPub = node_handle.advertise<control_msgs::FollowJointTrajectoryActionGoal>("joint_trajectory", 1, false);
	ros::ServiceServer service = node_handle.advertiseService("execute_trajectory", triggerCallback);

	ros::spin();

	return 0;
}
