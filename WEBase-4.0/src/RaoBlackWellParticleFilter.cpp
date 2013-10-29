/*
 * RaoBlackWellParticleFilter.cpp
 *
 *  Created on: Oct 24, 2013
 *      Author: neha
 */

#include "RaoBlackWellParticleFilter.h"

RaoBlackWellParticleFilter::RaoBlackWellParticleFilter()
{
	this->numRobotLocationSamples = 50;
	this->numRobotLocationSamples = 20;
	//initRobotAndBallEstimates();
}

RaoBlackWellParticleFilter::RaoBlackWellParticleFilter(int N, int L)
{
	this->numRobotLocationSamples = N;
	this->numRobotLocationSamples = L;
	//initRobotAndBallEstimates();
}

RaoBlackWellParticleFilter::~RaoBlackWellParticleFilter() {
}

std::list<RobotLocationEstimate> RaoBlackWellParticleFilter::getNewRobotLocationEstimate(Agent &agent) {
	if (RobotEstimateSet.empty()) {
		initRobotAndBallEstimates(agent);
	}
	return this->RobotEstimateSet;
}

std::list<BallEstimate> RaoBlackWellParticleFilter::getNewBallEstimate(Agent &agent) {
	//TODO: implement
	return this->BallEstimateSet;
}

void RaoBlackWellParticleFilter::initRobotAndBallEstimates(Agent &agent) {
	//std::cout << "Initializing blackwell filter\n";
	Vector selfPos = agent.GetSelfPosWithQueuedActions();

	Vector selfPos1 = agent.GetSelf().GetPos();
	double posConf = agent.GetSelf().GetPosConf();
	double posEps = agent.GetSelf().GetPosEps();
	int posDelay = agent.GetSelf().GetPosDelay();
	std::cout << "selfpos " << selfPos.X() << "   and : " << selfPos.Y()  <<  std::endl;
	std::cout << "selfpos1 " << selfPos1.X() << ", " << selfPos1.Y()  << ", " << posConf << ", "<< posEps << ", " << posDelay <<  std::endl;


	/*Vector selfVel = agent.GetSelfVelWithQueuedActions();
	std::cout << "selfVel " << selfVel.X() << "   and : " << selfVel.Y()  <<  std::endl;
	AngleDeg a = agent.GetSelfBodyDirWithQueuedActions();
	std::cout << "selfDirection " << a <<  std::endl;
	*/
	Vector ballPos = agent.GetBallPosWithQueuedActions();
	std::cout << "ballPos " << ballPos.X() << "   and : " << ballPos.Y()  <<  std::endl;
	Vector ballPos1 = agent.World().Ball().GetPos();
	double ballConf = agent.World().Ball().GetPosConf();
	double ballEps = agent.World().Ball().GetPosEps();
	int ballDelay = agent.World().Ball().GetPosDelay();
	std::cout << "ballpos1 " << ballPos1.X() << ", " << ballPos1.Y()  << ", " << ballConf << ", "<< ballEps << ", " << ballDelay <<  std::endl;

	/*Vector ballVel = agent.GetBallVelWithQueuedActions();
	std::cout << "ballVel " << ballVel.X() << "   and : " << ballVel.Y()  <<  std::endl;
	*/

}






