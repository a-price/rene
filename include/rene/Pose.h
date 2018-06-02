//
// Created by arprice on 6/1/18.
//

#ifndef RENE_POSE_H
#define RENE_POSE_H

#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Geometry>

namespace rene
{

struct Pose
{
	// Point
	Eigen::Vector3d r;
	// Quaternion
	Eigen::Quaterniond q;

	Pose(const Eigen::Vector3d& _r = Eigen::Vector3d::Zero(),
	     const Eigen::Quaterniond& _q = Eigen::Quaterniond::Identity())
		:r(_r), q(_q)
	{ }

	Pose(const Eigen::Affine3d& _R)
		:r(_R.translation()), q(_R.linear())
	{ }

	Pose(const geometry_msgs::Pose& pose)
		:r(pose.position.x, pose.position.y, pose.position.z),
		 q(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z)
	{ }

	Pose(const geometry_msgs::PoseStamped& pose)
		:Pose(pose.pose)
	{ }

	Pose(const tf::Transform& pose)
		:r(pose.getOrigin().x(), pose.getOrigin().y(), pose.getOrigin().z()),
		 q(pose.getRotation().w(), pose.getRotation().x(), pose.getRotation().y(), pose.getRotation().z())
	{ }

	bool operator==(const Pose& other) const
	{
		return this->r==other.r
		       && (this->q.coeffs()==other.q.coeffs() || this->q.coeffs()==-other.q.coeffs());
	}

	bool operator!=(const Pose& other) const
	{
		return !(*this==other);
	}

	Eigen::Vector3d operator*(const Eigen::Vector3d& point) const
	{
		return (q*point)+r;
	}

	Pose operator*(const Pose& rhs) const
	{
		return Pose((q*rhs.r)+r, q*rhs.q);
	}

	Pose inv() const
	{
		return {-r, q.inverse()};
	}
};

} // namespace rene

#endif //RENE_POSE_H
