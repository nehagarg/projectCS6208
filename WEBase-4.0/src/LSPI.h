/*
 * LSPI.h
 *
 *  Created on: Oct 25, 2013
 *      Author: juekun
 */

#ifndef LSPI_H_
#define LSPI_H_

#include "RaoBlackWellParticleFilter.h"
using namespace std;

class LSPI {
public:
	LSPI(RaoBlackWellParticleFilter& );
	float getAbsOriBall2Robot();
	float getSenseCost(AngleDeg current_neck_pos, AngleDeg target_neck_pos);
	list<float> BallEntropy();
	list<float> RobotEntropy();
	list<float> DirRob2GoalEntropy();
	int getDim();


private:
	RaoBlackWellParticleFilter&  RBPF;
	float getEntropy(list<float>);
	Vector getRange(list<float>);
	list<RobotLocationEstimate> RLE;
	list<BallEstimate> BLE;
	float mean(list<float>);
	float* listToArray(list<float> ls);
	Vector getRobotMeanLocation(list<RobotLocationEstimate> ls);
	Vector getBallMeanLocation(list<BallEstimate> ls);
	float getRobotMeanOrientation(list<RobotLocationEstimate> ls);
	float* getInterval(Vector range, int size);
	int dim;
};

#endif /* LSPI_H_ */
