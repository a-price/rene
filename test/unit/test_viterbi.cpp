/**
 * \file test_viterbi.cpp
 * \brief
 *
 * \author Andrew Price
 * \date 2014-12-16
 *
 * \copyright
 *
 * Copyright (c) 2014, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * This file is provided under the following "BSD-style" License:
 * Redistribution and use in source and binary forms, with or
 * without modification, are permitted provided that the following
 * conditions are met:
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above
 *   copyright notice, this list of conditions and the following
 *   disclaimer in the documentation and/or other materials provided
 *   with the distribution.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 * USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <math.h>
#include <algorithm>

#include "rene/Viterbi.hpp"
#include <gtest/gtest.h>

using rene::viterbi;

const std::vector< std::vector<double> > trellisA =
{
    {1,2},
    {2,1},
    {1,2}
};

const std::vector< std::vector<double> > trellisB =
{
    {1,2,4,2,5,3},
    {2,1,11,2,4,2},
    {1,2,2,2,2,3},
    {2,3,4,1,5,4},
    {1,2,3,4,5}
};

double stateCost(const double& s)
{
	return fabs(s);
}

double transitionCost(const double& s1, const double& /*t1*/, const double& s2, const double& /*t2*/)
{
	return fabs(s1)+fabs(s2);
}

void check_equal(const std::vector<int>& sln, const std::vector<int>& path)
{
	ASSERT_EQ(sln.size(), path.size());
	for (size_t i = 0; i < sln.size(); ++i)
	{
		ASSERT_EQ(sln[i], path[i]);
	}
}

TEST(ViterbiTest, testTrellisA)
{
	std::vector<int> path = viterbi<double>(trellisA, &stateCost, &transitionCost);

	const std::vector<int> sln = {0,1,0};

	check_equal(sln, path);
}

TEST(ViterbiTest, testTrellisB)
{
	std::vector<int> path = viterbi<double>(trellisB, &stateCost, &transitionCost);
	const std::vector<int> sln = {0,1,0,3,0};

	check_equal(sln, path);
}

TEST(ViterbiTest, testInfeasible)
{
	std::vector<int> path = viterbi<double>(trellisB, &stateCost, &transitionCost, 1e-3);
	const std::vector<int> sln = {};

	check_equal(sln, path);
}


int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}
