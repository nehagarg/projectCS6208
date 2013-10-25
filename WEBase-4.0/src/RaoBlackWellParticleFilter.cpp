/*
 * RaoBlackWellParticleFilter.cpp
 *
 *  Created on: Oct 24, 2013
 *      Author: neha
 */

#include "RaoBlackWellParticleFilter.h"

RaoBlackWellParticleFilter::RaoBlackWellParticleFilter(Agent & agent):
	mAgent ( agent )
{
	initRobotAndBallEstimates();
}

RaoBlackWellParticleFilter::RaoBlackWellParticleFilter(Agent & agent, int N, int L):
	mAgent ( agent )
{
	this->numRobotLocationSamples = N;
	this->numRobotLocationSamples = L;
	initRobotAndBallEstimates();
}

RaoBlackWellParticleFilter::~RaoBlackWellParticleFilter() {
}

std::list<RobotLocationEstimate> RaoBlackWellParticleFilter::getNewRobotLocationEstimate() {
	//TODO : implement
	return NULL;
}

std::list<BallEstimate> RaoBlackWellParticleFilter::getNewBallEstimate() {
	//TODO: implement
	return NULL;
}

void RaoBlackWellParticleFilter::initRobotAndBallEstimates() {
	//TODO implement
}






