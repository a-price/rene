//
// Created by arprice on 6/1/18.
//

#ifndef RENE_IKPROVIDER_H
#define RENE_IKPROVIDER_H

#include "Pose.h"

#include <string>
#include <vector>

namespace rene
{

struct SingleJointLimits
{
	double min_position;
	double max_position;

	SingleJointLimits(const double min, const double max)
		:min_position(min), max_position(max)
	{ }
};

class IKProvider
{
public:
	typedef std::vector<double> Solution;
	typedef std::vector<Solution> SolutionContainer;
	typedef std::vector<SingleJointLimits> Limits;

	// If the robot state is needed, pass const robot_state::RobotState* state to the concrete base class constructor

	virtual int getActiveJointDimension() = 0;

	virtual std::string getSolverBaseFrame() = 0;

	virtual std::string getSolverFinalFrame() = 0;

	virtual std::vector<std::string> getSolvedJointNames() = 0;

	virtual Limits getJointLimits() = 0;

	virtual SolutionContainer getSolutions(const Pose& pose) = 0;

	static void
	enumeratePeriodicSolutions(const Solution& initial_sln, const Limits& limits, SolutionContainer& container,
	                           const int num_joints);
};

} // namespace rene

#endif //RENE_IKPROVIDER_H
