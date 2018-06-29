/**
 * \file JointHomeCostProvider.cpp
 * \brief
 *
 * \author Andrew Price
 * \date 2018-06-02
 *
 * \copyright
 *
 * Copyright (c) 2018, Andrew Price
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

#include "rene/JointHomeCostProvider.h"
#include <cassert>
#include <cmath>

namespace rene
{

void JointHomeCostProvider::configure(IKProvider* ik)
{
	qWeights = std::vector<double>(static_cast<unsigned>(ik->getActiveJointDimension()), 1.0);
	for (const auto& limits : ik->getJointLimits())
	{
		qHome.push_back((limits.max_position + limits.min_position)/2.0);
	}
}

void JointHomeCostProvider::setHome(const std::vector<double>& _qHome, const std::vector<double>& _qWeights)
{
	qHome = _qHome;
	qWeights = _qWeights;

	if (qWeights.empty())
	{
		qWeights = std::vector<double>(qHome.size(), 1.0);
	}
}

double JointHomeCostProvider::operator()(const std::vector<double>& q) const
{
	assert(qHome.size() == q.size());
	assert(qWeights.size() == q.size());

	double cost = 0;
	for (std::size_t j = 0; j < q.size(); ++j)
	{
		cost += std::fabs(q[j]-qHome[j])*qWeights[j]/100.0;
	}
	return cost;
}


} // namespace rene
