/************************************************************************************
 * WrightEagle (Soccer Simulation League 2D)                                        *
 * BASE SOURCE CODE RELEASE 2013                                                    *
 * Copyright (c) 1998-2013 WrightEagle 2D Soccer Simulation Team,                   *
 *                         Multi-Agent Systems Lab.,                                *
 *                         School of Computer Science and Technology,               *
 *                         University of Science and Technology of China            *
 * All rights reserved.                                                             *
 *                                                                                  *
 * Redistribution and use in source and binary forms, with or without               *
 * modification, are permitted provided that the following conditions are met:      *
 *     * Redistributions of source code must retain the above copyright             *
 *       notice, this list of conditions and the following disclaimer.              *
 *     * Redistributions in binary form must reproduce the above copyright          *
 *       notice, this list of conditions and the following disclaimer in the        *
 *       documentation and/or other materials provided with the distribution.       *
 *     * Neither the name of the WrightEagle 2D Soccer Simulation Team nor the      *
 *       names of its contributors may be used to endorse or promote products       *
 *       derived from this software without specific prior written permission.      *
 *                                                                                  *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND  *
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED    *
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE           *
 * DISCLAIMED. IN NO EVENT SHALL WrightEagle 2D Soccer Simulation Team BE LIABLE    *
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL       *
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR       *
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER       *
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,    *
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF *
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.                *
 ************************************************************************************/

#include "Player.h"
#include "DecisionTree.h"
#include "DynamicDebug.h"
#include "Formation.h"
#include "CommandSender.h"
#include "Parser.h"
#include "Thread.h"
#include "UDPSocket.h"
#include "WorldModel.h"
#include "Agent.h"
#include "VisualSystem.h"
#include "Logger.h"
#include "CommunicateSystem.h"
#include "TimeTest.h"
#include "Dasher.h"
#include "RaoBlackWellParticleFilter.h"

Player::Player() :
		mpDecisionTree(new DecisionTree), mpRaoBlackWellParticleFilter(
				new RaoBlackWellParticleFilter) {
	numRuns = 0;
	numEpisodes = 0;
	lookedAtBall = false;
	lookedAtMarker = false;
	positionsSet = false;
	//ballPosVector(0,0);// = new Vector(0,0);
	//selfPosVector(0,0);// = new Vector(0,0);
}

Player::~Player() {
	delete mpDecisionTree;
}

void Player::SendOptionToServer() {
	while (!mpParser->IsClangOk()) {
		mpAgent->CheckCommands(mpObserver);
		mpAgent->Clang(7, 8);
		mpObserver->SetCommandSend();
		WaitFor(200);
	}

	while (!mpParser->IsSyncOk()) {
		mpAgent->CheckCommands(mpObserver);
		mpAgent->SynchSee();
		mpObserver->SetCommandSend();
		WaitFor(200);
	}

	mpAgent->CheckCommands(mpObserver);
	mpAgent->EarOff(false);
	mpObserver->SetCommandSend();
	WaitFor(200);
}

void Player::Run() {
	//TIMETEST("Run");

	static Time last_time = Time(-100, 0);

	mpObserver->Lock();

	/** 下面几个更新顺序不能变 */
	Formation::instance.SetTeammateFormations();
	CommunicateSystem::instance().Update(); //在这里解析hear信息，必须首先更新
	mpAgent->CheckCommands(mpObserver);
	mpWorldModel->Update(mpObserver);

	mpObserver->UnLock();

	const Time & time = mpAgent->GetWorldState().CurrentTime();

	if (last_time.T() >= 0) {
		if (time != Time(last_time.T() + 1, 0)
				&& time != Time(last_time.T(), last_time.S() + 1)) {
			if (time == last_time) {
				mpAgent->World().SetCurrentTime(
						Time(last_time.T(), last_time.S() + 1)); //否则决策数据更新会出问题
			}
		}
	}

	last_time = time;

	Formation::instance.UpdateOpponentRole(); //TODO: 暂时放在这里，教练未发来对手阵型信息时自己先计算
	//mpRaoBlackWellParticleFilter->getNewRobotLocationEstimate(*mpAgent);
	//VisualSystem::instance().ResetVisualRequest();

	if (mpAgent->World().GetPlayMode() == PM_Play_On) {
		positionsSet = false;
		if(lookedAtBall && lookedAtMarker){
		if (!mpAgent->isEpisodeEnded()) {

			if (numRuns % 5 == 0) {
				actionNeckValue = VisualSystem::instance().MyDecision(*mpObserver);
				//CommunicateSystem::instance().SendTeammateStatus(*mpAgent->World(), mpAgent->GetSelf().GetUnum(), 0);
			}
			if (numRuns % 5 == 1){ // Change 0 to 1 for sarsop data gathering
				ss.str("");
				ss << actionNeckValue;
				ss << " " << mpAgent->GetSelf().GetPosDelay();
				ss << " " << mpAgent->World().Ball().GetPosDelay();
				ss << " " << mpAgent->GetSelf().GetPos().X();
				ss << " " << mpAgent->GetSelf().GetPos().Y();
				ss << " " << mpAgent->GetSelf().GetPosConf();
				ss << " " << mpAgent->GetSelf().GetBodyDir();
				ss << " " << mpAgent->GetSelf().GetBodyDirConf();
				ss << " " << mpAgent->World().Ball().GetPos().X();
				ss << " " << mpAgent->World().Ball().GetPos().Y();
				ss << " " << mpAgent->World().Ball().GetPosConf();
				ss << " " << mpAgent->GetSelf().GetPosEps();
				ss << " " << mpAgent->World().Ball().GetPosEps();
				std::cout << "Sending string " << ss.str() << std::endl;
				mpAgent->Say(ss.str());
			}

			//VisualSystem::instance().Decision();
			//CommunicateSystem::instance().Decision();

			mpAgent->SetHistoryActiveBehaviors();
			numRuns++;

			Logger::instance().LogSight();

		}

		bool actionExecuted = mpDecisionTree->Decision(*mpAgent);
		if(mpAgent->isEpisodeEnded() && actionExecuted)
			{
				ss.str("");
				ss << "9";
				ss << " 0" ; // neck turn angle
				ss << " " << mpAgent->GetSelf().GetPosDelay();
				ss << " " << mpAgent->World().Ball().GetPosDelay();
				ss << " " << mpAgent->GetSelf().GetPos().X();
				ss << " " << mpAgent->GetSelf().GetPos().Y();
				ss << " " << mpAgent->GetSelf().GetPosConf();
				ss << " " << mpAgent->GetSelf().GetBodyDir();
				ss << " " << mpAgent->GetSelf().GetBodyDirConf();
				ss << " " << mpAgent->World().Ball().GetPos().X();
				ss << " " << mpAgent->World().Ball().GetPos().Y();
				ss << " " << mpAgent->World().Ball().GetPosConf();
				ss << " " << mpAgent->GetSelf().GetPosEps();
				ss << " " << mpAgent->World().Ball().GetPosEps();
				//std::cout << "Sending episode end message to trainer" << ss.str() << " \n";
				mpAgent->Say(ss.str());

			}
		}
		if (lookedAtMarker && !lookedAtBall) {
			AngleDeg target = (mpAgent->World().Ball().GetPos()
					- mpAgent->GetSelf().GetPos()).Dir();
			mpAgent->TurnNeck(
					target - mpAgent->GetSelf().GetNeckDir()
							- mpAgent->GetSelf().GetBodyDir());
			lookedAtBall = true;
		}
		if (!lookedAtMarker) {
			AngleDeg target = (mpObserver->Marker(Goal_L).GlobalPosition()
					- mpAgent->GetSelf().GetPos()).Dir();
			mpAgent->TurnNeck(
					target - mpAgent->GetSelf().GetNeckDir()
							- mpAgent->GetSelf().GetBodyDir());
			lookedAtMarker = true;
		}



	}
	else
	{
		mpAgent->setEpisodeEnded(false);
		//int  ballballPosVectorY;
		//int selfPosVectorX, selfPosVectorY;
		int ballPositionXArray[2] = {35, 25};
		int ballPositionYArray[5] = {20, 10, 0, -10, -20};
		int selfPositionXArray[1] = {-10};
		int selfPositionYArray[5] = {25, 15, 0 , -10, -20};
		if(!positionsSet)
		{

			//ballPosVectorX = 35; ballPosVectorY = 20;// = new Vector(35, 20); // Set ball position
			//selfPosVectorX = -10; selfPosVectorY = 10;
			positionsSet = true;
			numEpisodes++;
		}
		lookedAtBall = false;
		lookedAtMarker = false;
		Vector *b = new Vector(ballPositionXArray[((numEpisodes -1)/5)%2], ballPositionYArray[(numEpisodes -1)%5] );
		Vector *p = new Vector(selfPositionXArray[((numEpisodes -1)/50)%1], selfPositionYArray[((numEpisodes -1)/10)%5]);
		//Vector *b = new Vector(25, 0);
		//Vector *p = new Vector(-10, -20);
		mpAgent->World().Ball().UpdatePos(*b, 0, 1.0);
		mpAgent->Self().UpdatePos(*p, 0, 1.0);

		//lookedAtBall = true;
		//lookedAtMarker = true;
		//if(lookedAtBall && lookedAtMarker)
		//{
			ss.str("");
			ss << mpAgent->GetSelf().GetUnum();
			ss << " 0" ; // neck turn angle
			ss << " " << mpAgent->GetSelf().GetPosDelay();
			ss << " " << mpAgent->World().Ball().GetPosDelay();
			ss << " " << mpAgent->GetSelf().GetPos().X();
			ss << " " << mpAgent->GetSelf().GetPos().Y();
			ss << " " << mpAgent->GetSelf().GetPosConf();
			ss << " " << mpAgent->GetSelf().GetBodyDir();
			ss << " " << mpAgent->GetSelf().GetBodyDirConf();
			ss << " " << mpAgent->World().Ball().GetPos().X();
			ss << " " << mpAgent->World().Ball().GetPos().Y();
			ss << " " << mpAgent->World().Ball().GetPosConf();
			ss << " " << mpAgent->GetSelf().GetPosEps();
			ss << " " << mpAgent->World().Ball().GetPosEps();

			mpAgent->Say(ss.str());
			ss.clear();
			numRuns = 0;


			if(numEpisodes == 51)
			{
				exit(0);
			}
		//}
		//mpRaoBlackWellParticleFilter->getNewRobotLocationEstimate(*mpAgent);
	}
}
