//
// Created by arprice on 6/4/18.
//

#ifndef RENE_IKFAST_MACRO_H
#define RENE_IKFAST_MACRO_H


#define IKFAST_PLUGIN_MACRO_INTERIOR(freeJointDataPtr) \
const rene::IKProvider::Limits limits = getJointLimits(); \
\
ikfast::IkSolutionList<IkReal> solutions; \
Eigen::Matrix3d R(pose.q); \
ComputeIk(pose.r.data(), R.data(), freeJointDataPtr, solutions); \
for (size_t j = 0; j<solutions.GetNumSolutions(); ++j) \
{ \
	const ikfast::IkSolutionBase<IkReal>& sol = solutions.GetSolution(j); \
	std::vector<double> solnJoints, freeJoints; \
	sol.GetSolution(solnJoints, freeJoints); \
\
	bool isValidSolution = true; \
	for (size_t joint_idx = 0; joint_idx<solnJoints.size(); ++joint_idx) \
	{ \
		assert(limits[joint_idx].min_position <= limits[joint_idx].max_position); \
		if (!std::isfinite(solnJoints[joint_idx])) \
		{ \
			isValidSolution = false; \
			break; \
		}\
\
		while (solnJoints[joint_idx]<limits[joint_idx].min_position) \
		{ \
			solnJoints[joint_idx] += 2.0*M_PI; \
		} \
		while (solnJoints[joint_idx]>limits[joint_idx].max_position) \
		{ \
			solnJoints[joint_idx] -= 2.0*M_PI; \
		} \
		if (limits[joint_idx].min_position>solnJoints[joint_idx] \
		    || limits[joint_idx].max_position<solnJoints[joint_idx]) \
		{ \
			isValidSolution = false; \
		} \
	} \
	if (!isValidSolution) { continue; } \
\
	std::vector<std::vector<double>> allJointSolutions; \
	rene::IKProvider::enumeratePeriodicSolutions(solnJoints, limits, allJointSolutions, solnJoints.size()); \
\
	for (size_t solution_idx = 0; solution_idx<allJointSolutions.size(); ++solution_idx) \
	{ \
		Solution sln; \
		for (double jointVal : allJointSolutions[solution_idx]) \
		{ \
			sln.push_back(jointVal); \
		} \
		frameSolutions.push_back(sln); \
	} \
}


#define IKFAST_PLUGIN_MACRO(ClassName, freeJointDataPtr) \
rene::IKProvider::SolutionContainer ClassName::getSolutions(const rene::Pose& pose) \
{ \
	rene::IKProvider::SolutionContainer frameSolutions; \
	IKFAST_PLUGIN_MACRO_INTERIOR(freeJointDataPtr) \
	return frameSolutions; \
}

#define IKFAST_PLUGIN_MACRO_DEBUG(ClassName, freeJointDataPtr) \
rene::IKProvider::SolutionContainer ClassName::getSolutions(const rene::Pose& pose) \
{ \
	const rene::IKProvider::Limits limits = getJointLimits(); \
\
	ikfast::IkSolutionList<IkReal> solutions; \
	Eigen::Matrix3d R(pose.q); \
	ComputeIk(pose.r.data(), R.data(), freeJointDataPtr, solutions); \
	rene::IKProvider::SolutionContainer frameSolutions; \
	for (size_t j = 0; j<solutions.GetNumSolutions(); ++j) \
	{ \
		const ikfast::IkSolutionBase<IkReal>& sol = solutions.GetSolution(j); \
		std::vector<double> solnJoints, freeJoints; \
		sol.GetSolution(solnJoints, freeJoints); \
\
		bool isValidSolution = true; \
		for (size_t joint_idx = 0; joint_idx<solnJoints.size(); ++joint_idx) \
		{ \
			if (limits[joint_idx].min_position>solnJoints[joint_idx] \
			    || limits[joint_idx].max_position<solnJoints[joint_idx]) \
			{ \
				std::cerr << "IKFast Solution " << j << " does not obey joint limit " << joint_idx << std::endl; \
				std::cerr << "( " << limits[joint_idx].min_position << " : " << solnJoints[joint_idx] << " : " \
				          << limits[joint_idx].max_position << " )" << std::endl; \
			} \
			assert(limits[joint_idx].min_position <= limits[joint_idx].max_position); \
\
			while (solnJoints[joint_idx]<limits[joint_idx].min_position) \
			{ \
				solnJoints[joint_idx] += 2.0*M_PI; \
			} \
			while (solnJoints[joint_idx]>limits[joint_idx].max_position) \
			{ \
				solnJoints[joint_idx] -= 2.0*M_PI; \
			} \
			if (limits[joint_idx].min_position>solnJoints[joint_idx] \
			    || limits[joint_idx].max_position<solnJoints[joint_idx]) \
			{ \
				std::cerr << "IKFast Solution " << j << " still does not obey joint limit " << joint_idx \
				          << ". Failing!!!" << std::endl; \
				std::cerr << "( " << limits[joint_idx].min_position << " : " << solnJoints[joint_idx] << " : " \
				          << limits[joint_idx].max_position << " )" << std::endl; \
				isValidSolution = false; \
			} \
		} \
		if (!isValidSolution) { continue; } \
\
		std::vector<std::vector<double>> allJointSolutions; \
		rene::IKProvider::enumeratePeriodicSolutions(solnJoints, limits, allJointSolutions, solnJoints.size()); \
      std::cerr << "Expanding solution " << j << " into " << allJointSolutions.size() << " solutions." << std::endl; \
\
		for (size_t solution_idx = 0; solution_idx<allJointSolutions.size(); ++solution_idx) \
		{ \
			Solution sln; \
			for (double jointVal : allJointSolutions[solution_idx]) \
			{ \
				sln.push_back(jointVal); \
			} \
			frameSolutions.push_back(sln); \
		} \
	} \
	return frameSolutions; \
}

#endif //RENE_IKFAST_MACRO_H
