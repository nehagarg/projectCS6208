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

#include "VisualSystem.h"
#include "Strategy.h"
#include "Formation.h"
#include "TimeTest.h"
#include "PositionInfo.h"
#include "InterceptInfo.h"
#include "InterceptModel.h"
#include "Agent.h"
#include "BehaviorShoot.h"
#include "Logger.h"
#include "Trainer.h"

#include <list>
#include <string>
#include <sstream>
#include <iostream>
#include <fstream>
#include <vector>
#include <math.h>

using namespace std;

VisualSystem::VisualSystem()
{
	mCanForceChangeViewWidth = false;
	mIsSearching = false;
	mIsCritical = false;
	mForbidden = false;
	mForceToSeeObject.bzero();
}

VisualSystem::~VisualSystem() {
}

VisualSystem & VisualSystem::instance()
{
	static VisualSystem info_system;
	return info_system;
}

void VisualSystem::Initial(Agent * agent)
{
	mpAgent         = agent;
	mpWorldState    = & (agent->World());
	mpInfoState     = & (agent->Info());
	mpBallState     = & (agent->World().Ball());
	mpSelfState     = & (agent->Self());

	mVisualRequest.GetOfBall().mpObject = mpBallState;
	mVisualRequest.GetOfBall().mUnum = 0;

	for (Unum i = 1; i <= TEAMSIZE; ++i) {
		mVisualRequest.GetOfTeammate(i).mpObject = & (mpWorldState->GetTeammate(i));
		mVisualRequest.GetOfOpponent(i).mpObject = & (mpWorldState->GetOpponent(i));
		mVisualRequest.GetOfTeammate(i).mUnum = i;
		mVisualRequest.GetOfOpponent(i).mUnum = -i;
	}
}

void VisualSystem::ResetVisualRequest()
{
	mCanTurn = false;
	mSenseBallCycle = 0;

	if (mpAgent->IsNewSight()) { //有新视觉才重置，否则维持上周期的结果
		mCanForceChangeViewWidth = false;
		mIsSearching = false;
		mIsCritical = false;
		mForbidden = false;

		for (Unum i = -TEAMSIZE; i <= TEAMSIZE; ++i){
			mVisualRequest[i].Clear();
		}

		mHighPriorityPlayerSet.clear();
		mForceToSeeObject.bzero();
	}

	if (mpSelfState->GetPosConf() < FLOAT_EPS) {
		mIsCritical = true;
		mCanForceChangeViewWidth = false;
	}
	if (mpBallState->GetPosConf() < FLOAT_EPS) {
		RaiseForgotObject(0);
	}

	ViewModeDecision();
}

double VisualSystem::normaldist(double mean,double std,double x)
{
	double c=1/std/sqrt(2*3.1415926);
	double e=exp(-(x-mean)*(x-mean)/2/std/std);
	return c*e;
}
int VisualSystem::getDistance(double dis) {
	//<=20: 1; 20<x<=30: 2; 30<x<=40: 3; 40<x<=50: 4; 50<x<=60: 5; >60: 6

	if (dis <= 10)
		return 1;
	else if (dis <= 20)
		return 2;
	else if (dis <= 30)
		return 3;
	else if (dis <= 40)
		return 4;
	else if (dis <= 50)
		return 5;
	else
		return 6;
}

int VisualSystem::getDirectionBall(double dir) {
	dir = abs(dir - 1);
	//discretize into 6 regions
	return int(floor(dir / 30.0));
}

int VisualSystem::getDirectionGoal(double dir) {
	dir = abs(dir - 1);
	return int(floor(dir / 30.0));
}

struct alpha_vector{
		int action;
		vector<double> values;
};

int VisualSystem::OptimalLSPIAction(Observer &observer)
{
	//Read Weights
	ifstream wFile ("./train/Weights.txt", ios::in);
	double weights[6][9][6];
	string weightsLine;
	while (getline(wFile, weightsLine)){
		istringstream iss(weightsLine);
		int i = 0;
		int distance = -1;
		int action = -1;
		while(iss){
			string weightLinePart;
			iss >> weightLinePart;
			if(i==0)
			{
				distance = atoi(weightLinePart.c_str());
			}
			if(i==1)
			{
				distance = atoi(weightLinePart.c_str());
			}
			if(i>1)
			{
				weights[distance][action][i-2] = atof(weightLinePart.c_str());
			}
			i++;
		}
	}
	wFile.close();

	//get player's estimate of Db, Angle2ball, Angle2goal TODO
	double Db,A2b,A2g;

	Db = mpAgent->World().Ball().GetPos().Dist(mpAgent->Self().GetPos());
	A2b = abs((mpAgent->World().Ball().GetPos() - mpAgent->Self().GetPos()).Dir() - mpAgent->Self().GetBodyDir());
	A2g = abs((observer.Marker(MarkerType(1)).GlobalPosition() - mpAgent->Self().GetPos()).Dir() - mpAgent->Self().GetBodyDir());

	int discretizedDistance = getDistance(Db) -1;

	double QValue[9];
	for(int i = 0; i< 9; i++)
	{
		double target;
		if (i == 2) //look at ball
		{
			target = (mpAgent->World().Ball().GetPos() - mpAgent->GetSelf().GetPos()).Dir();

		}
		else  //look at marker
		{
				target = (observer.Marker((MarkerType)i).GlobalPosition()- mpAgent->GetSelf().GetPos()).Dir();
		}
		double estimatedTurnNeckAngle = abs(target - mpAgent->GetSelf().GetNeckDir() - mPreBodyDir);
		QValue[i] = (A2b*weights[discretizedDistance][i][0]) + (A2g*weights[discretizedDistance][i][1]) +
				(estimatedTurnNeckAngle * weights[discretizedDistance][i][2]) +
				(mpAgent->Self().GetPosConf() * weights[discretizedDistance][i][3]) +
				(mpAgent->World().Ball().GetPosConf() * weights[discretizedDistance][i][4]) + weights[discretizedDistance][i][5];

	}

	double max = QValue[0];
	int maxIndex = 0;
	for(int i=1; i < 9;i++)
	{
		if (QValue[i] > max)
		{
			max = QValue[i];
			maxIndex = i;
		}
	}
	return maxIndex;
}
int VisualSystem::OptimalAction(Observer &observer)
{
	//int action=std::rand() % 9;
	ifstream statelst ("state_list.txt", ios::in);
	vector<int> state_list;
	if (statelst.is_open()) // get the state list
	  {
		while(statelst.good()) // To get you all the lines.
		 {
			string STRING;
			getline(statelst,STRING); // Saves the line in STRING.
			int S = atoi(STRING.c_str());
			state_list.push_back(S);
		 }
	    statelst.close();
	  }
	//get conf of Db, Angle2ball, Angle2goal TODO
	double conf_Db,conf_A2b,conf_A2g;
	conf_Db = mpAgent->World().Ball().GetPosConf();
	conf_A2b = conf_Db;
	conf_A2g = mpAgent->Self().GetPosConf();
	//get player's estimate of Db, Angle2ball, Angle2goal TODO
	double Db,A2b,A2g;

	Db = mpAgent->World().Ball().GetPos().Dist(mpAgent->Self().GetPos());
	A2b = abs((mpAgent->World().Ball().GetPos() - mpAgent->Self().GetPos()).Dir() - mpAgent->Self().GetBodyDir());
	A2g = abs((observer.Marker(MarkerType(1)).GlobalPosition() - mpAgent->Self().GetPos()).Dir() - mpAgent->Self().GetBodyDir());

	int Db_mean_state,A2b_mean_state,A2g_mean_state;
	Db_mean_state=getDistance(Db);
	A2b_mean_state=getDirectionBall(A2b);
	A2g_mean_state=getDirectionGoal(A2g);
	//form three independent normal distribution with player's estimates as mean
	//and mapping conf=0 as norm (std=1) and conf=1 as norm (std=0.0001) (linear mapping)
	double std_Db,std_A2b,std_A2g;
	std_Db=(0.0001-1)*conf_Db+1;
	std_A2b=(0.0001-1)*conf_A2b+1;
	std_A2g=(0.0001-1)*conf_A2g+1;

	int vsize=state_list.size();
	vector<double> belief;
	//get the belief of every state in the state list
	for (int i=0;i<vsize;i++)
	{
		int state_Db, state_A2b, state_A2g;
		int state=state_list[i];
		//decompose each state into 3 component
		state_A2g=state%6;
		state_A2b=((state-state_A2g)/6)%6;
		state_Db=(state-state_A2g-state_A2b*6)/36;
		//get belief of each elementary state
		double belief_Db,belief_A2b,belief_A2g,state_belief;
		belief_Db=normaldist(Db_mean_state,std_Db,state_Db);
		belief_A2b=normaldist(A2b_mean_state,std_A2b,state_A2b);
		belief_A2g=normaldist(A2g_mean_state,std_A2g,state_A2g);
		//total belief of state_list[i]
		state_belief=belief_Db*belief_A2b*belief_A2g;//assume independence
		belief.push_back(state_belief);
	}

	//read alpha vector file
	ifstream file("alphaVector.txt");
		int dim;
		if (file.is_open())
		{
			string dimstr;
			getline(file,dimstr);
			dim=atoi(dimstr.c_str());
			//cout<<dim<<endl;
		}
		string line;
		list<alpha_vector> alpha_vector_list;
	    alpha_vector avt;
		while (getline(file,line))
		{
			avt.action=line[0]-'0';
			int len=line.length();
			string value;
			for (int i=2;i<len;i++)
			{
				if ((isspace(line[i]))| (i==(len-1)))
				{
					avt.values.push_back(atof(value.c_str()));
					value="";
				}
				else
				{
					value+=line[i];
				}
			}
			alpha_vector_list.push_back(avt);
			avt=(struct alpha_vector){0};
		}
		//dot product of belief and alpha vector to get the optimal action
		alpha_vector avv,optimal_avv;
		int size=alpha_vector_list.size();
		double max=-1;
		for (int i=0;i<size;i++)
			{
				avv=alpha_vector_list.front();
				alpha_vector_list.pop_front();
				double sum=0;
				int avv_size=avv.values.size();
				if (avv_size!=vsize)
				{
					cout<<"Dimension of belief and alpha vector does not match!"<<endl;
					return -1;
				}
				else
				{
					for (int ii=0;ii<avv_size;i++)
						sum+=avv.values[ii]*belief[ii];
					if (sum>=max)
					{
						max=sum;
						optimal_avv=avv;
					}
				}
			}

	return optimal_avv.action;
}


std::string VisualSystem::MyDecision(Observer &observer)
{
	//AngleDeg selfPosAngle = mpAgent->GetSelf().GetPos().Dir();
	AngleDeg selfAngle = mpAgent->GetSelf().GetNeckDir();
	std::cout << "Self angle value : " << selfAngle << std::endl;
	AngleDeg target;


	int i;
	//i = std::rand() % 9 ; //2
	//i = OptimalAction(observer);
	i=OptimalLSPIAction(observer);


	if (i == 2) //look at ball
	{
		target = (mpAgent->World().Ball().GetPos() - mpAgent->GetSelf().GetPos()).Dir();

	}
	else  //look at marker
	{
		target = (observer.Marker((MarkerType)i).GlobalPosition()- mpAgent->GetSelf().GetPos()).Dir();
	}
	std::cout << "Turning neck to " << target << "from " << selfAngle << "to look at" << i <<  std::endl;
	/*if ((target - selfAngle)  > 180)
	{
		mpAgent->TurnNeck(target  - selfAngle - 360);
	}
	else if ((target - selfAngle)  < - 180)
	{
		mpAgent->TurnNeck(target  - selfAngle + 360);
	}
	else
	{*/

		AngleDeg finalValue = target  - selfAngle - mPreBodyDir;
		std::cout << "Turning neck by : " << finalValue << std::endl;
		std::ostringstream ss;
		ss << i;
		ss << " " << finalValue; // neck turn angle
		/*
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
		mpAgent->Say(ss.str());*/
		mpAgent->TurnNeck(finalValue);
		return ss.str();
	//}
	//selfAngle = mpAgent->GetSelf().GetNeckGlobalDir();

}

void VisualSystem::Decision(){
	if (mpAgent->GetActionEffector().IsTurnNeck()) return; //其他地方已经产生了转脖子动作
	if (mForbidden) return;

	if (!DealWithSetPlayMode()) {
		DoDecision();
	}
}

void VisualSystem::ViewModeDecision()
{
	ChangeViewWidth(mpSelfState->GetViewWidth());

	if (mpAgent->IsNewSight() == false){
		ChangeViewWidth(mpSelfState->GetViewWidth());
		return;
	}

	if (mpWorldState->GetPlayMode() != PM_Before_Kick_Off){
		double balldis  = mpInfoState->GetPositionInfo().GetBallDistToTeammate(mpSelfState->GetUnum());
		if (balldis > 60.0){
			ChangeViewWidth(VW_Wide);
		}
		else if (balldis > 40.0){
			ChangeViewWidth(VW_Normal);
		}
		else {
			ChangeViewWidth(VW_Narrow);
		}
	}
	else{
		ChangeViewWidth(VW_Narrow);
	}
}

void VisualSystem::DealVisualRequest()
{
	DealWithSpecialObjects();
	SetVisualRing();
	GetBestVisualAction();
}

void VisualSystem::EvaluateVisualRequest()
{
	{
		VisualRequest *vr = &mVisualRequest.GetOfBall();
		vr->mValid = mpBallState->GetPosConf() > FLOAT_EPS;

		if (vr->mValid) {
			vr->mPrePos = mPreBallPos - mPreSelfPos;
			vr->mCycleDelay = mpBallState->GetPosDelay();

			vr->UpdateEvaluation();

			if (vr->mConf < FLOAT_EPS){
				mForceToSeeObject.GetOfBall() = true;
			}
		}
	}

	PlayMode play_mode = mpWorldState->GetPlayMode();

	if (play_mode == PM_Our_Penalty_Ready || play_mode == PM_Our_Penalty_Taken) {
		const int goalie = mpWorldState->GetOpponentGoalieUnum();
		if (!goalie) return;

		VisualRequest *vr = &mVisualRequest.GetOfOpponent(goalie);

		if (mpWorldState->GetOpponent(goalie).IsAlive()) {
			vr->mPrePos = mpWorldState->Opponent(goalie).GetPredictedPos(1) - mPreSelfPos;
			vr->mValid = true;
			vr->mCycleDelay = vr->mpObject->GetPosDelay();

			vr->UpdateEvaluation();

			if (vr->mConf < FLOAT_EPS) {
				mHighPriorityPlayerSet.insert(vr);
			}
		}
		else {
			vr->mValid = false;
		}
	}
	else {
		for (int i = -TEAMSIZE; i <= TEAMSIZE; ++i) {
			if (!i || i == mpSelfState->GetUnum()) continue;

			VisualRequest *vr = &mVisualRequest[i];
			vr->mValid = mpWorldState->GetPlayer(i).IsAlive();

			if (vr->mValid) {
				vr->mPrePos = mpWorldState->GetPlayer(i).GetPredictedPos() - mPreSelfPos;
				vr->mCycleDelay = vr->mpObject->GetPosDelay();

				vr->UpdateEvaluation();

				if (vr->mConf < FLOAT_EPS){
					mHighPriorityPlayerSet.insert(vr);
				}
			}
		}
	}
}

void VisualSystem::DoInfoGather()
{
	const Strategy & strategy = mpAgent->GetStrategy();

	if (!mCanTurn){
		if (mpAgent->GetActionEffector().IsMutex() == false && !mIsCritical){
			SetCanTurn(true);
		}
	}
	else {
		if (mpAgent->GetActionEffector().IsMutex() || mIsCritical){
			SetCanTurn(false);
		}
	}

	if (mpSelfState->IsIdling()) {
		SetCanTurn(false);
	}

	UpdatePredictInfo();

	if(mpWorldState->GetPlayMode() == PM_Our_Penalty_Ready || mpWorldState->GetPlayMode() == PM_Our_Penalty_Taken /*|| mpWorldState->GetPlayMode() == PM_Our_Penalty_Setup*/ ){
		RaisePlayer(-mpWorldState->GetOpponentGoalieUnum(), 1.0);
		RaiseBall();
		return;
	}

	if (mpWorldState->GetPlayMode() > PM_Opp_Mode && mpInfoState->GetPositionInfo().GetClosestOpponentDistToBall() < 3.0 && mpBallState->GetPos().Dist(mpSelfState->GetPos()) < 20.0 && mpBallState->GetPosDelay() > 1){
		SetForceSeeBall(); //关注对手发球
	}

	if(mpSelfState->GetPos().X() - mpInfoState->GetPositionInfo().GetTeammateOffsideLine() > -5.0 && mpInfoState->GetPositionInfo().GetTeammateOffsideLineOpp() != Unum_Unknown){//offside look
		RaisePlayer(-mpInfoState->GetPositionInfo().GetTeammateOffsideLineOpp(), 2.0);
	}

	if (mpSelfState->IsGoalie()){
		if (strategy.IsMyControl() || mpWorldState->GetPlayMode() == PM_Our_Goal_Kick){
			DoInfoGatherForDefense();
		}
		else {
			DoInfoGatherForGoalie();
		}
	}
	else {
		if (strategy.IsBallFree()
				&& strategy.GetController() != 0
				&& mpAgent->GetFormation().GetTeammateRoleType(mpSelfState->GetUnum()).mLineType != LT_Defender
				&& (strategy.GetBallFreeCycleLeft() > 3 || (strategy.IsMyControl() && strategy.GetMyInterCycle() > 3)))
		{
			DoInfoGatherForBallFree();
		}
		else {
			switch(strategy.GetSituation()){
			case ST_Defense:
				DoInfoGatherForDefense();
				break;
			case ST_Forward_Attack:
				DoInfoGatherForFastForward();
				break;
			case ST_Penalty_Attack:
				DoInfoGatherForPenaltyAttack();
				break;
			}
		}
	}

	if (mpWorldState->GetPlayMode() != PM_Play_On) {
		DoInfoGatherForBallFree();
	}
}

void VisualSystem::DoInfoGatherForBallFree()
{
	const Strategy & strategy = mpAgent->GetStrategy();

	double ball_free_cyc_left; //ball free cycle left,这里是经过调整的有概率意义的freecyc,不同于StrategyInfo里的那个值
	const double rate = 0.6; //可以认为是调整值符合真实值的成功率
	double my_int_cycle, tm_int_cycle, opp_int_cycle;

	my_int_cycle = strategy.GetMyInterCycle();
	tm_int_cycle = strategy.GetMinTmInterCycle() * rate + strategy.GetSureTmInterCycle() * (1 - rate);
	opp_int_cycle = strategy.GetMinOppInterCycle() * rate + strategy.GetSureOppInterCycle() * (1 - rate);

	ball_free_cyc_left = Min(tm_int_cycle, opp_int_cycle);
	ball_free_cyc_left = Min(my_int_cycle, ball_free_cyc_left);

	if(opp_int_cycle < ball_free_cyc_left + 2 && ball_free_cyc_left > 3){
		ball_free_cyc_left -= 1;
	}

	std::vector<OrderedIT>::const_iterator it;
	const std::vector<OrderedIT> & OIT = mpInfoState->GetInterceptInfo().GetOIT();

	if (strategy.IsMyControl() && mpAgent->GetFormation().GetTeammateRoleType(mpSelfState->GetUnum()).mLineType != LT_Defender && ball_free_cyc_left < 6 && ball_free_cyc_left > 2){
		RaiseBall(1.0);
	}
	else{
		RaiseBall();
	}

	if(!strategy.IsMyControl()){//自己不用拿球
		int eva = 3;
		for (it = OIT.begin(); it != OIT.end(); ++it){
			if(it->mpInterceptInfo->mMinCycle > 50) break;
			RaisePlayer(it->mUnum, eva++);
		}
	}
	else {//自己需要拿球
		RaiseBall(2.0);
		int eva = 3;
		for (it = OIT.begin(); it != OIT.end(); ++it){
			if(it->mpInterceptInfo->mMinCycle > 50) break;
			if (it->mUnum == mpSelfState->GetUnum()) continue;
			RaisePlayer(it->mUnum, ++eva);
			if (mpWorldState->GetPlayer(it->mUnum).GetPosDelay() > it->mpInterceptInfo->mMinCycle) {
				mHighPriorityPlayerSet.insert(GetVisualRequest(it->mUnum));
			}
		}
	}
}

void VisualSystem::DoInfoGatherForFastForward()
{
	const Strategy & strategy = mpAgent->GetStrategy();

	RaiseBall();
	switch (mpAgent->GetFormation().GetTeammateRoleType(mpSelfState->GetUnum()).mLineType){
	case LT_Defender:
		for (Unum i = 1; i <= TEAMSIZE; ++i){
			if (i != mpSelfState->GetUnum() && i != mpWorldState->GetTeammateGoalieUnum()){
				switch (mpAgent->GetFormation().GetTeammateRoleType(i).mLineType){
				case LT_Defender:
					RaisePlayer(i, 8.0);
					break;
				case LT_Midfielder:
					RaisePlayer(i, 5.0);
					break;
				case LT_Forward:
					RaisePlayer(i, 5.0);
					break;
				default:
					PRINT_ERROR("line type error");
					break;
				}
			}
		}
		break;
	case LT_Midfielder:
		for (Unum i = 1; i <= TEAMSIZE; ++i){
			if (i != mpSelfState->GetUnum() && i != mpWorldState->GetTeammateGoalieUnum()){
				switch (mpAgent->GetFormation().GetTeammateRoleType(i).mLineType){
				case LT_Defender:
					RaisePlayer(i, 100.0);
					break;
				case LT_Midfielder:
					RaisePlayer(i, 5.0);
					break;
				case LT_Forward:
					RaisePlayer(i, 2.0);
					break;
				default:
					PRINT_ERROR("line type error");
					break;
				}
			}
			if (mpWorldState->GetOpponent(i).GetPosConf() > FLOAT_EPS){
				if(mpWorldState->GetOpponent(i).GetPos().X() > mPreBallPos.X() - 8.0)
					RaisePlayer(-i, 4.0);
				else
					RaisePlayer(-i, 100.0);
			}
		}
		if (mpWorldState->GetOpponentGoalieUnum() && mpWorldState->GetOpponent(mpWorldState->GetOpponentGoalieUnum()).GetPosConf() > FLOAT_EPS){
			RaisePlayer(-mpWorldState->GetOpponentGoalieUnum(), 20.0);
		}
		break;
	case LT_Forward:
		for (Unum i = 1; i <= TEAMSIZE; ++i){
			if (i != mpSelfState->GetUnum() && i != mpWorldState->GetTeammateGoalieUnum()){
				switch (mpAgent->GetFormation().GetTeammateRoleType(i).mLineType){
				case LT_Defender:
					RaisePlayer(i, 100.0);
					break;
				case LT_Midfielder:
					RaisePlayer(i, 5.0);
					break;
				case LT_Forward:
					RaisePlayer(i, 3.0);
					break;
				default:
					PRINT_ERROR("line type error");
					break;
				}
			}
			if (mpWorldState->GetOpponent(i).GetPosConf() > FLOAT_EPS){
				double opp_x = mpWorldState->GetOpponent(i).GetPos().X();
				if (!strategy.IsMyControl() && fabs(opp_x - mpInfoState->GetPositionInfo().GetTeammateOffsideLine()) < 1.0){
					RaisePlayer(-i, 2.6);
				}
				else if (!strategy.IsMyControl() && fabs(opp_x - mpInfoState->GetPositionInfo().GetTeammateOffsideLine()) < 3.6){
					RaisePlayer(-i, 3);
				}
				else if (opp_x > mPreBallPos.X() - 8.0){
					if (opp_x > mPreBallPos.X() + 36.0){
						RaisePlayer(-i, 6);
					}
					else{
						RaisePlayer(-i, 4);
					}
				}
				else{
					RaisePlayer(-i, 100);
				}
			}
		}
		if (mpWorldState->GetOpponentGoalieUnum() && mpWorldState->GetOpponent(mpWorldState->GetOpponentGoalieUnum()).GetPosConf() > FLOAT_EPS){
			RaisePlayer(-mpWorldState->GetOpponentGoalieUnum(), 15.0);
		}
		break;
	default:
		PRINT_ERROR("line type error");
		break;
	}
}

void VisualSystem::DoInfoGatherForPenaltyAttack()
{
	const Strategy & strategy = mpAgent->GetStrategy();

	RaiseBall();
	switch (mpAgent->GetFormation().GetTeammateRoleType(mpSelfState->GetUnum()).mLineType){
	case LT_Defender:
		for (Unum i = 1; i <= TEAMSIZE; ++i){
			if (i != mpSelfState->GetUnum() && i != mpWorldState->GetTeammateGoalieUnum()){
				RaisePlayer(i);
			}
			RaisePlayer(-i);
		}
		break;
	case LT_Midfielder:
		for (Unum i = 1; i <= TEAMSIZE; ++i){
			if (i != mpSelfState->GetUnum() && i != mpWorldState->GetTeammateGoalieUnum()){
				switch (mpAgent->GetFormation().GetTeammateRoleType(i).mLineType){
				case LT_Defender:
					RaisePlayer(i, 100.0);
					break;
				case LT_Midfielder:
					RaisePlayer(i, 5.0);
					break;
				case LT_Forward:
					RaisePlayer(i, 5.0);
					break;
				default:
					PRINT_ERROR("line type error");
					break;
				}
			}
			if (mpWorldState->GetOpponent(i).GetPosConf() > FLOAT_EPS){
				if(mpWorldState->GetOpponent(i).GetPos().X() > mPreBallPos.X() - 8.0)
					RaisePlayer(-i, 3.0);
				else
					RaisePlayer(-i, 100.0);
			}
		}
		if (mpWorldState->GetOpponentGoalieUnum() && mpWorldState->GetOpponent(mpWorldState->GetOpponentGoalieUnum()).GetPosConf() > FLOAT_EPS){
			RaisePlayer(-mpWorldState->GetOpponentGoalieUnum(), 20.0);
		}
		break;
	case LT_Forward:
		for (Unum i = 1; i <= TEAMSIZE; ++i){
			if (i != mpSelfState->GetUnum() && i != mpWorldState->GetTeammateGoalieUnum()){
				switch (mpAgent->GetFormation().GetTeammateRoleType(i).mLineType){
				case LT_Defender:
					RaisePlayer(i, 100.0);
					break;
				case LT_Midfielder:
					RaisePlayer(i, 5.0);
					break;
				case LT_Forward:
					RaisePlayer(i, 3.0);
					break;
				default:
					PRINT_ERROR("line type error");
					break;
				}
			}
			if (mpWorldState->GetOpponent(i).GetPosConf() > FLOAT_EPS){
				if(mpWorldState->GetOpponent(i).GetPos().X() > mPreBallPos.X() - 8.0)
					RaisePlayer(-i, 3.0);
				else
					RaisePlayer(-i, 100.0);
			}
		}
		break;
	default:
		PRINT_ERROR("line type error");
		break;
	}

	//特殊视觉请求
	if(ServerParam::instance().oppPenaltyArea().IsWithin(mPreSelfPos)
			&& (strategy.IsMyControl()
					|| (strategy.IsTmControl() && mpInfoState->GetPositionInfo().GetBallDistToTeammate(mpSelfState->GetUnum()) < 20.0))){
		if (mpWorldState->GetOpponentGoalieUnum() && mpWorldState->GetOpponent(mpWorldState->GetOpponentGoalieUnum()).GetPosConf() > FLOAT_EPS){
			if (strategy.IsMyControl()
					&& (mpSelfState->GetPos().X() > 38.0 || (mpInfoState->GetPositionInfo().GetBallDistToOpponent(mpWorldState->GetOpponentGoalieUnum()) < 8.0 && mpSelfState->GetPos().X() > 36.0)))
			{
				if (mpWorldState->GetPlayMode() == PM_Our_Back_Pass_Kick || mpWorldState->GetPlayMode() == PM_Our_Indirect_Free_Kick){
					RaisePlayer(-mpWorldState->GetOpponentGoalieUnum(), 1.2);
				}
				else {
					RaisePlayer(-mpWorldState->GetOpponentGoalieUnum(), 1.0);
				}
			}
			else {
				RaisePlayer(-mpWorldState->GetOpponentGoalieUnum(), 2.0);
			}
		}
	}
}

void VisualSystem::DoInfoGatherForDefense()
{
	if(mVisualRequest.GetOfBall().mConf < FLOAT_EPS && !mCanTurn && !mIsCritical){
		mForceToSeeObject.GetOfBall() = true;
	}

	RaiseBall();
	switch (mpAgent->GetFormation().GetTeammateRoleType(mpSelfState->GetUnum()).mLineType){
	case LT_Goalie:
	case LT_Defender:
		for (Unum i = 1; i <= TEAMSIZE; ++i){
			if (i != mpSelfState->GetUnum() && i != mpWorldState->GetTeammateGoalieUnum()){
				switch (mpAgent->GetFormation().GetTeammateRoleType(i).mLineType){
				case LT_Defender:
					RaisePlayer(i, 5.0);
					break;
				case LT_Midfielder:
					RaisePlayer(i, 10.0);
					break;
				case LT_Forward:
					RaisePlayer(i, 100.0);
					break;
				default:
					PRINT_ERROR("line type error");
					break;
				}
			}
			RaisePlayer(-i);
		}
		break;
	case LT_Midfielder:
		for (Unum i = 1; i <= TEAMSIZE; ++i){
			if (i != mpSelfState->GetUnum() && i != mpWorldState->GetTeammateGoalieUnum()){
				RaisePlayer(i);
			}
			RaisePlayer(-i, 12.0);
		}
		if (mpWorldState->GetOpponentGoalieUnum() && mpWorldState->GetOpponent(mpWorldState->GetOpponentGoalieUnum()).GetPosConf() > FLOAT_EPS){
			RaisePlayer(-mpWorldState->GetOpponentGoalieUnum(), 20.0);
		}
		break;
	case LT_Forward:
		for (Unum i = 1; i <= TEAMSIZE; ++i){
			if (i != mpSelfState->GetUnum() && i != mpWorldState->GetTeammateGoalieUnum()){
				switch (mpAgent->GetFormation().GetTeammateRoleType(i).mLineType){
				case LT_Defender:
					RaisePlayer(i, 12.0);
					break;
				case LT_Midfielder:
					RaisePlayer(i, 8.0);
					break;
				case LT_Forward:
					RaisePlayer(i, 8.0);
					break;
				default:
					PRINT_ERROR("line type error");
					break;
				}
			}
			RaisePlayer(-i, 12.0);
		}
		break;
	default:
		PRINT_ERROR("line type error");
		break;
	}
}

void VisualSystem::DoInfoGatherForGoalie()
{
	const Strategy & strategy = mpAgent->GetStrategy();

	RaiseBall();
	for (Unum i = 1; i <= TEAMSIZE; i++){
		if (i != mpSelfState->GetUnum()){
			RaisePlayer(i, 50);
		}
		RaisePlayer(-i, 50);
	}

	//特殊视觉请求
	if(mpInfoState->GetPositionInfo().GetBallDistToTeammate(mpSelfState->GetUnum()) < 26.0){
		SetForceSeeBall();
		if (strategy.IsOppControl()){
			RaisePlayer(strategy.GetController(), 2.0);
		}
	}
}

void VisualSystem::DoVisualExecute()
{
	mBestVisualAction.mDir = GetNormalizeAngleDeg(mBestVisualAction.mDir);

	AngleDeg finalnec = GetNormalizeAngleDeg(mBestVisualAction.mDir - mPreBodyDir); //最后要看的方向与当前（或者说执行过本周期动作后，如turn）身体正对方向的夹角，即最后的neckrelangle。

	if (fabs(finalnec) > ServerParam::instance().maxNeckAngle()) {
		if (mCanTurn) {
			if (finalnec < 0.0) {
				mpAgent->Turn(finalnec - ServerParam::instance().minNeckAngle());
				mpAgent->TurnNeck(ServerParam::instance().minNeckMoment());
			}
			else {
				mpAgent->Turn(finalnec - ServerParam::instance().maxNeckAngle());
				mpAgent->TurnNeck(ServerParam::instance().maxNeckMoment());
			}
		}
		else {
			if (finalnec < 0.0) {
				mpAgent->TurnNeck(ServerParam::instance().minNeckMoment());
			}
			else {
				mpAgent->TurnNeck(ServerParam::instance().maxNeckMoment());
			}
		}
	}
	else {
		finalnec -= mpSelfState->GetNeckDir();
		mpAgent->TurnNeck(finalnec);
	}

	if (mViewWidth != mpSelfState->GetViewWidth()) {
		mpAgent->ChangeView(mViewWidth);
	}
}

/**
* 提出球的视觉请求
* @param eva 视觉请求的强度，理解成每eva周期看一次球
*/
void VisualSystem::RaiseBall(double eva)
{
	const Strategy & strategy = mpAgent->GetStrategy();

	if (mpWorldState->GetPlayMode() < PM_Our_Mode && mpWorldState->GetPlayMode() > PM_Play_On){
		eva = Min(2.0, eva);
	}

	if (eva < FLOAT_EPS ){
		if(strategy.IsBallFree() && mpWorldState->GetPlayMode() == PM_Play_On){
			if(mpBallState->GetVelDelay() <= mpWorldState->CurrentTime() - strategy.GetLastBallFreeTime()){//球free后曾看到过球速
				//在球即将不再free时要看球,其他可以不看,但上限是3个周期,其间可以收集些别的信息
				eva = Max(Min(strategy.IsMyControl()? double(strategy.GetMyInterCycle()): strategy.GetBallFreeCycleLeft(), 3.0), 1.0);
			}
			else{ //球free后还未看到过球速
				eva = 1.0;
				SetForceSeeBall();
			}
		}
		else if(strategy.IsTmControl()){
			double ball_dis = (mPreBallPos - mPreSelfPos).Mod();
			eva = ball_dis / 20.0 + 1.0;
			eva = Max(eva, 3.0);
		}
		else{
			double ball_dis = (mPreBallPos - mPreSelfPos).Mod();
			eva = ball_dis / 20.0;
			eva = Max(eva, 2.0);
		}
	}

	mVisualRequest.GetOfBall().mFreq = Min(mVisualRequest.GetOfBall().mFreq, eva);
}

/**
* 提出球员的视觉请求
* @param num 球员号码，+ 队友，- 对手
* @param eva 视觉请求的强度，理解成每eva周期看一次这个人
*/
void VisualSystem::RaisePlayer(ObjectIndex unum, double eva)
{
	if (unum == 0 || !mpWorldState->GetPlayer(unum).IsAlive() || unum == mpSelfState->GetUnum()) {
		return;
	}

	if (eva < FLOAT_EPS ){
		double dis = mpInfoState->GetPositionInfo().GetPlayerDistToPlayer(mpSelfState->GetUnum(), unum);
		if(dis < 3.0){
			eva = 6.0;
		}
		else if(dis < 6.0){
			eva = 5.0;
		}
		else if(dis < 20.0){
			eva = 10.0;
		}
		else if(dis < 40.0){
			eva = 26.0;
		}
		else{
			eva = 50.0;
		}
	}

	VisualRequest *vr = GetVisualRequest(unum);
	vr->mFreq = Min(vr->mFreq, eva);

	if (mpWorldState->GetPlayer(unum).GetPosConf() < FLOAT_EPS) {
		if(eva <= 2){
			RaiseForgotObject(unum);
		}
	}

	PlayMode pm = mpWorldState->GetPlayMode();
	if ( ((/*pm == PM_Our_Penalty_Setup ||*/ pm == PM_Our_Penalty_Ready || pm == PM_Our_Penalty_Taken) && mpWorldState->GetPlayer(unum).IsGoalie())
			|| (-unum == mpInfoState->GetPositionInfo().GetTeammateOffsideLineOpp() && mpAgent->GetFormation().GetMyRole().mLineType == LT_Forward && eva <= mpWorldState->GetPlayer(unum).GetPosDelay())) {
		RaiseForgotObject(unum);
	}
}

void VisualSystem::RaiseForgotObject(ObjectIndex unum)
{
	mIsSearching = true;

	if (unum == 0) {
		mForceToSeeObject.GetOfBall() = true;
	}
	else {
		mHighPriorityPlayerSet.insert(GetVisualRequest(unum));
	}
}

inline void VisualSystem::UpdatePredictInfo()
{
	mPreBodyDir = mpAgent->GetSelfBodyDirWithQueuedActions();
	mPreSelfPos = mpAgent->GetSelfPosWithQueuedActions();

	const Unum kickale_player = mpAgent->GetInfoState().GetPositionInfo().GetPlayerWithBall();
	if (kickale_player != 0 && kickale_player != mpSelfState->GetUnum()) {
		mPreBallPos = mpBallState->GetPos();
	}
	else {
		mPreBallPos = mpAgent->GetBallPosWithQueuedActions();
	}
}

VisualSystem::VisualAction VisualSystem::VisualRing::GetBestVisualAction(const AngleDeg left_most, const AngleDeg right_most, const AngleDeg interval_length)
{
	//区间定义成： [left, right]
	AngleDeg left = left_most;
	AngleDeg right = left;

	double sum = 0.0;

	for (; right < left + interval_length; ++right) {
		sum += Score(right);
	}
	sum += Score(right);

	double max = sum;
	AngleDeg best = (left + right) * 0.5;

	for (; right < right_most; ++right, ++left) {
		AngleDeg in = (Score(right + 1) - Score(left));

		sum += in;

		if (in < FLOAT_EPS) continue;

		if (sum > max) {
			max = sum;

			AngleDeg alpha = left + 1;
			while (Score(alpha) < FLOAT_EPS && alpha < right_most) alpha ++;
			AngleDeg beta = right + 1;
			while (Score(beta) < FLOAT_EPS && beta > alpha) beta --;
			best = (alpha + beta) * 0.5;
		}
	}

	return VisualAction(best, max);
}

bool VisualSystem::DealWithSetPlayMode()
{
	static bool check_both_side = false;
	static bool check_one_side = false;

	PlayMode play_mode = mpWorldState->GetPlayMode();
	PlayMode last_play_mode = mpWorldState->GetLastPlayMode();

	if (play_mode == PM_Our_Penalty_Ready || play_mode == PM_Our_Penalty_Setup || play_mode == PM_Our_Penalty_Taken) return false;

	if (play_mode != PM_Play_On && !(mForceToSeeObject.GetOfBall()) && !mpSelfState->IsGoalie()
			&& play_mode != PM_Before_Kick_Off
			&& last_play_mode != PM_Before_Kick_Off
			&& last_play_mode != PM_Our_Kick_Off
			&& last_play_mode != PM_Opp_Kick_Off
	){
		//特殊模式下要强制看看全场， 防止遗漏
		if (mpWorldState->CurrentTime() < mpWorldState->GetPlayModeTime() + 3){ //前三个周期等视觉信息到来
			check_both_side = false;
			check_one_side = false;
		}
		else {
			if (!mpAgent->IsNewSight()) return true;

			if (!check_both_side) {
				if (!check_one_side){
					int diff = mpWorldState->CurrentTime() - mpWorldState->GetPlayModeTime();
					int flag = diff % 6;
					int base = mpSelfState->GetUnum() % 6; //不要大家都一起改变视角，这样可能一起错过对方发球的瞬间

					if (flag == base) {
						mPreBodyDir = mpAgent->GetSelfBodyDirWithQueuedActions();
						mCanTurn = false;
						mBestVisualAction.mDir = GetNormalizeAngleDeg(mPreBodyDir + 90.0);
						ChangeViewWidth(VW_Wide); //new info will come in 3 cycle
						mCanForceChangeViewWidth = false;
						DoVisualExecute();
						check_one_side = true;

						return true;
					}
				}
				else {
					mBestVisualAction.mDir = GetNormalizeAngleDeg(mpSelfState->GetNeckGlobalDir() + 180.0);
					ChangeViewWidth(VW_Wide); //new info will come in 3 cycle
					mCanForceChangeViewWidth = false;
					DoVisualExecute();
					check_one_side = false;
					check_both_side = true;

					return true;
				}
			}
		}
	}

	return false;
}

void VisualSystem::DealWithSpecialObjects()
{
	static const double buffer = 0.25;
	static const int object_count = TEAMSIZE * 2 + 1;

	static const double high_priority_multi = object_count * 0.5;
	static const double force_to_see_player_multi = object_count * high_priority_multi;
	static const double force_to_see_ball_multi = object_count * force_to_see_player_multi;

	if (!mHighPriorityPlayerSet.empty()) {
		for (std::set<VisualRequest*>::iterator it = mHighPriorityPlayerSet.begin(); it != mHighPriorityPlayerSet.end(); ++it) {
			if (!(*it)->mValid) continue;
			(*it)->mScore = (*it)->Multi() * (mForceToSeeObject[(*it)->mUnum]? force_to_see_player_multi: high_priority_multi);

			if ((*it)->mpObject->GetPosConf() > 0.9 && (*it)->PreDistance() < ServerParam::instance().visibleDistance() - buffer) {
				SetCritical(true);
			}
		}
	}

	if (mForceToSeeObject[0] && mVisualRequest[0].mValid) {
		if (mpBallState->GetPosConf() > 0.9 && mVisualRequest[0].PreDistance() < ServerParam::instance().visibleDistance() - buffer) {
			if (mpBallState->GetPosDelay() == 0) {
				mVisualRequest[0].mScore = FLOAT_EPS; //just sense it
			}

			SetCritical(true);
		}
		else {
			mVisualRequest[0].mScore = mVisualRequest[0].Multi() * force_to_see_ball_multi;
		}
	}
}

void VisualSystem::SetVisualRing()
{
	mVisualRing.Clear();

	for (int i = -TEAMSIZE; i <= TEAMSIZE; ++i) {
		const VisualRequest & visual_request = mVisualRequest[i];

		if (!visual_request.mValid) continue;

		for (double dir = -5.0; dir <= 5.0; dir += 1.0) {
			mVisualRing.Score(visual_request.mPrePos.Dir() - mPreBodyDir + dir) += visual_request.mScore / 11.0;
		}
	}
}

void VisualSystem::GetBestVisualAction()
{
	if (mIsCritical) {
		if (NewSightComeCycle(VW_Wide) == 1) {
			ChangeViewWidth(VW_Wide);
		}
		else if (NewSightComeCycle(VW_Normal) == 1) {
			ChangeViewWidth(VW_Normal);
		}
		else {
			ChangeViewWidth(VW_Narrow);
		}

		mBestVisualAction = GetBestVisualActionWithViewWidth(mViewWidth, true);
	}
	else {
		if (mViewWidth != VW_Narrow && !mIsSearching) {
			mBestVisualAction = GetBestVisualActionWithViewWidth(mViewWidth, true);
		}
		else {
			mCanForceChangeViewWidth = true;


			Array<VisualAction, 3> best_visual_action;

			//球在3.0米以内，尽量保证每周期都能感知到球，除非对球的下周期状态预测很有把握
			const bool force =
					mpWorldState->GetPlayMode() != PM_Our_Penalty_Taken
					&& mpWorldState->GetBall().GetPosDelay() == 0 && mpWorldState->GetHistory(1)->GetBall().GetPosDelay() == 0
					&& mpAgent->GetInfoState().GetPositionInfo().GetClosestOpponentDistToBall() > 3.0;

			if (force && PlayerParam::instance().SaveTextLog()) {
				Logger::instance().GetTextLogger("sure_ball") << mpWorldState->CurrentTime() << std::endl;
			}

			if (NewSightComeCycle(VW_Wide) == 1) {
				best_visual_action[2] = GetBestVisualActionWithViewWidth(VW_Wide, force);
			}
			else if (NewSightComeCycle(VW_Normal) == 1) {
				best_visual_action[1] = GetBestVisualActionWithViewWidth(VW_Normal, force);
				best_visual_action[2] = GetBestVisualActionWithViewWidth(VW_Wide, force);
			}
			else {
				best_visual_action[0] = GetBestVisualActionWithViewWidth(VW_Narrow, force);
				best_visual_action[1] = GetBestVisualActionWithViewWidth(VW_Normal, force);
				best_visual_action[2] = GetBestVisualActionWithViewWidth(VW_Wide, force);
			}

			const double buffer = FLOAT_EPS;
			if (best_visual_action[0].mScore > best_visual_action[1].mScore - buffer) {
				if (best_visual_action[0].mScore > best_visual_action[2].mScore - buffer) {
					//narrow
					ChangeViewWidth(VW_Narrow);
					mBestVisualAction = best_visual_action[0];
				}
				else {
					//wide
					ChangeViewWidth(VW_Wide);
					mBestVisualAction = best_visual_action[2];
				}
			}
			else {
				if (best_visual_action[1].mScore > best_visual_action[2].mScore - buffer) {
					//normal
					ChangeViewWidth(VW_Normal);
					mBestVisualAction = best_visual_action[1];
				}
				else {
					//wide
					ChangeViewWidth(VW_Wide);
					mBestVisualAction = best_visual_action[2];
				}
			}
		}
	}

	mBestVisualAction.mDir += mPreBodyDir;
	//Assert(mBestVisualAction.mScore > 0.0);
}

VisualSystem::VisualAction VisualSystem::GetBestVisualActionWithViewWidth(ViewWidth view_width, bool force)
{
	if (force || GetSenseBallCycle() >= NewSightComeCycle(view_width)) {
		AngleDeg max_turn_ang = mCanTurn? mpSelfState->GetMaxTurnAngle(): 0.0;
		AngleDeg half_view_angle = sight::ViewAngle(view_width) * 0.5;
		AngleDeg neck_left_most = ServerParam::instance().minNeckAngle() - max_turn_ang; //脖子可以到达的极限角度（相对于当前身体正前方而言）
		AngleDeg neck_right_most = ServerParam::instance().maxNeckAngle() + max_turn_ang;
		AngleDeg left_most = neck_left_most - half_view_angle;
		AngleDeg right_most = neck_right_most + half_view_angle;

		VisualAction best_visual_action = mVisualRing.GetBestVisualAction(left_most, right_most, half_view_angle * 2.0);

		best_visual_action.mScore /= NewSightWaitCycle(view_width);

		Assert(!IsInvalid(best_visual_action.mScore));

		return best_visual_action;
	}
	else {
		return VisualAction();
	}
}

bool VisualSystem::ForceSearchBall()
{
	if (!mpSelfState->IsIdling() && !mVisualRequest[0].mValid) { //force to scan
		mpAgent->GetActionEffector().ResetForScan();

		if (NewSightComeCycle(VW_Wide) == 1) {
			mpAgent->ChangeView(VW_Wide);
		}
		else if (NewSightComeCycle(VW_Normal) == 1){
			mpAgent->ChangeView(VW_Normal);
		}
		else {
			mpAgent->ChangeView(VW_Narrow);
		}

		mpAgent->Turn(sight::ViewAngle(VW_Narrow) - 5.0);

		return true;
	}
	return false;
}

void VisualSystem::DoDecision()
{
	if (!mpAgent->IsNewSight()){
		if (mCanForceChangeViewWidth) {
			mViewWidth = VW_Narrow; //重置为窄视角
		}
		else {
			mViewWidth = mpSelfState->GetViewWidth();
		}
	}

	DoInfoGather(); //提请求
	EvaluateVisualRequest(); //评价

	if (!ForceSearchBall()) {
		DealVisualRequest(); //处理请求SetForceSeeBall
		DoVisualExecute(); //视觉执行
	}
}

int VisualSystem::GetSenseBallCycle()
{
	if (mSenseBallCycle == 0) { //用拦截模型来算什么时候可以感知到球
		if (mVisualRequest[0].mValid) {
			if (mVisualRequest[0].PreDistance() < ServerParam::instance().visibleDistance()) {
				mSenseBallCycle = 1;
			}
			else {
				PlayerInterceptInfo int_info;
				VirtualSelf virtual_self(*mpSelfState);

				virtual_self.UpdateIsGoalie(false); //否则会用可扑范围代替感知范围计算截球

				int_info.mRes = IR_None;
				int_info.mpPlayer = & virtual_self;

				InterceptInfo::CalcTightInterception(*mpBallState, & int_info);

				mSenseBallCycle = int_info.mMinCycle;
				mSenseBallCycle = Max(mSenseBallCycle, 1);
			}
		}
		else {
			mSenseBallCycle = 1000;
		}
	}

	return mSenseBallCycle;
}

void VisualSystem::SetForceSeeBall()
{
	mForceToSeeObject.GetOfBall() = true;

	Logger::instance().GetTextLogger("force_to_see") << mpWorldState->CurrentTime() << ": ball" << std::endl;
}

void VisualSystem::SetForceSeePlayer(Unum i)
{
	if (i != 0) {
		mForceToSeeObject[i] = true;
		mHighPriorityPlayerSet.insert(GetVisualRequest(i));

		Logger::instance().GetTextLogger("force_to_see") << mpWorldState->CurrentTime() << ": player " << i << std::endl;
	}
}
