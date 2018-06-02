//
// Created by arprice on 6/1/18.
//

#include "rene/IKProvider.h"

namespace rene
{

void enumerateSolutionsHelper(const IKProvider::Solution& initial_sln, const IKProvider::Limits& limits,
                              IKProvider::Solution& partial_sln, IKProvider::SolutionContainer& container,
                              const int num_joints, const int j)
{
	if (j==num_joints)
	{
		// A new solution is constructed and ready to be inserted
		container.push_back(partial_sln);
	}
	else
	{
		double q = initial_sln[j];

		// Add the current joint to the partial solution
		if (limits[j].min_position<=q && q<=limits[j].max_position)
		{
			partial_sln[j] = q;
			enumerateSolutionsHelper(initial_sln, limits, partial_sln, container, num_joints, j+1);
		}

		// Search up the configuration space
		q = initial_sln[j]+2.0*M_PI;
		while (q<=limits[j].max_position)
		{
			partial_sln[j] = q;
			enumerateSolutionsHelper(initial_sln, limits, partial_sln, container, num_joints, j+1);
			q += 2.0*M_PI;
		}

		// Search down the configuration space
		q = initial_sln[j]-2.0*M_PI;
		while (q>=limits[j].min_position)
		{
			partial_sln[j] = q;
			enumerateSolutionsHelper(initial_sln, limits, partial_sln, container, num_joints, j+1);
			q -= 2.0*M_PI;
		}
	}
}

void
IKProvider::enumeratePeriodicSolutions(const Solution& initial_sln, const Limits& limits, SolutionContainer& container,
                                       const int num_joints)
{
	assert(limits.size()==num_joints);
	Solution partial(num_joints, 0.0);
	enumerateSolutionsHelper(initial_sln, limits, partial, container, num_joints, 0);
}

} // namespace rene