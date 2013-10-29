/*
 * LSPI.cpp
 *
 *  Created on: Oct 25, 2013
 *      Author: juekun
 */

#include "LSPI.h"
#include <stdlib.h>
#include <math.h>

#define PI 3.14159265
//3d grid dimension
#define A 4
#define B 4
#define C 4
//2d grid dimension
#define D 4
#define E 4
//1d grid dimension
#define F 4

LSPI::LSPI(RaoBlackWellParticleFilter& R): RBPF ( R )
{
	this->RLE=RBPF.getNewRobotLocationEstimate();
	this->BLE=RBPF.getNewBallEstimate();
	this->dim=1+A*B*C+D*E+F+1;

}

float LSPI::getAbsOriBall2Robot() //abs(theta\b)
{
	//assume both location of ball and robot are global
	Vector Ballmeanpos=getBallMeanLocation(BLE);
	Vector Robotmeanpos=getRobotMeanLocation(RLE);
	float RobotOri=getRobotMeanOrientation(RLE);

	if (Ballmeanpos.X()==Robotmeanpos.X())
		return AngleDeg(abs(90-RobotOri));

	float angle=atan2(Ballmeanpos.Y()-Robotmeanpos.Y(),Ballmeanpos.X()-Robotmeanpos.X()) * 180/PI;
	return (abs(angle-RobotOri));

}

float LSPI::getSenseCost(AngleDeg current_neck_pos, AngleDeg target_neck_pos)
{
	return float(abs(current_neck_pos-target_neck_pos));
}

list<float> LSPI::BallEntropy()
{

}

list<float> LSPI::RobotEntropy()
{

}

list<float> LSPI::DirRob2GoalEntropy()
{
	list<RobotLocationEstimate> Dir(RLE.begin(),RLE.end());
	list<float> Dir2;

	while (!Dir.empty())
	{
		Dir2.push_back(Dir.front().RobotOri2Goal);
		Dir.pop_front();
	}
	//Divide 1d grid
	Vector range=getRange(Dir2);
	float* arr = getInterval(range,F);
	//TO DO
}

int LSPI::getDim()
{
	return this->dim;
}

float getEntropy(list<float> ls)
{
	//weight should not be zero
	float sum=0;
	while(!ls.empty())
	{
		sum+=(-ls.front()*log(ls.front()));
		ls.pop_front();
	}
	return sum;
}

Vector getRange(list<float> ls)
{
	float max=-1000000000;
	float min=10000000000;
	while(!ls.empty())
	{
		if (ls.front()>max)
			max=ls.front();
		if (ls.front()<min)
			min=ls.front();
		ls.pop_front();
	}
	return Vector(min,max);

}

float mean(list<float> ls)
{
	float sum=0;
	while (!ls.empty())
	{
		sum+=ls.front();
		ls.pop_front();
	}
	return (sum/ls.size());
}

float* listToArray(list<float> ls)
{
	float *arr=new float[ls.size()];
	copy(ls.begin(),ls.end(),arr);
	return arr;
}

Vector getRobotMeanLocation(list<RobotLocationEstimate> ls)
{
	list<float> xx;
	list<float> yy;
	while (!ls.empty())
	{
		xx.push_back(ls.front().roboPos.X());
		yy.push_back(ls.front().roboPos.Y());
		ls.pop_front();
	}
	Vector mean=(mean(xx),mean(yy));
	return mean;
}

Vector getBallMeanLocation(list<BallEstimate> ls)
{
	list<float> xx;
	list<float> yy;
	while (!ls.empty())
	{
		xx.push_back(ls.front().ballPos.X());
		yy.push_back(ls.front().ballPos.Y());
		ls.pop_front();
	}
	Vector mean=(mean(xx),mean(yy));
	return mean;
}

float getRobotMeanOrientation(list<RobotLocationEstimate> ls)
{
	list<float> ori;
	while(!ls.empty())
	{
		ori.push_back(ls.front().roboDir);
		ls.pop_front();
	}
	return mean(ori);
}

float* getInterval(Vector range, int size)
{
	float *arr=new float[size];
	float step=(range.Y()-range.X())/size;
	for (int i=0;i<size;i++)
	{
		if (i==size-1)
			arr[i]=range.Y();
		else
			arr[i]=range.X()+i*step;
	}

	return arr;

}
