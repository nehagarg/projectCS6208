/*
 * LSPIWeight.h
 *
 *  Created on: Oct 26, 2013
 *      Author: juekun
 */

#ifndef LSPIWEIGHT_H_
#define LSPIWEIGHT_H_
#include "LSPI.h"
using namespace std;

class LSPIWeight {
public:
	LSPIWeight(LSPI& L);
	float* EntryConcatenation(float abs_OriBall2Robot, list<float> BallposEntropy, list<float> RobotposEntropy, list<float> RobotOri2GoalEntropy, float sensecost);
	float** getA(float* Fi_current, float* Fi_next);
	float* getb(float* Fi_current);
	float* WeightUpdate(float** A, float* b);

private:
	LSPI lspi;
	float abs_OriBall2Robot;
	list<float> BallposEntropy;
	list<float> RobotposEntropy;
	list<float> RobotOri2GoalEntropy;
	float sensecost;
	int dim;
	float gamma=0.95;
	float reward=1;
	float** A;
	float* b;


};

#endif /* LSPIWEIGHT_H_ */
