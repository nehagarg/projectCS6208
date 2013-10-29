#ifndef RAOBLACKWELLPARTICLEFILTER_H_
#define RAOBLACKWELLPARTICLEFILTER_H_

#include "Agent.h"

struct RobotLocationEstimate {
	Vector roboPos;
	AngleDeg roboDir;
	double weight;
	AngleDeg RobotOri2Goal;
};

enum BallModel {
	BM_None,
	BM_Grabbed,
	BM_Kicked,
	BM_Bounced,
	BM_Deflected
};

struct BallEstimate {
	Vector ballPos;
	Vector ballVel;
	BallModel m;
	RobotLocationEstimate rho;
	double weight;

};

class RaoBlackWellParticleFilter {
public:
	RaoBlackWellParticleFilter();
	RaoBlackWellParticleFilter( int N, int L);
	virtual ~RaoBlackWellParticleFilter();

	int getNumBallLocationSamples() const {
		return numBallLocationSamples;
	}

	void setNumBallLocationSamples(int numBallLocationSamples) {
		this->numBallLocationSamples = numBallLocationSamples;
	}

	int getNumRobotLocationSamples() const {
		return numRobotLocationSamples;
	}

	void setNumRobotLocationSamples(int numRobotLocationSamples) {
		this->numRobotLocationSamples = numRobotLocationSamples;
	}

	std::list<RobotLocationEstimate> getNewRobotLocationEstimate(Agent &agent);
	std::list<RobotLocationEstimate> getNewRobotLocationEstimate();
	std::list<BallEstimate> getNewBallEstimate(Agent &agent);
	std::list<BallEstimate> getNewBallEstimate();
private:
	int numRobotLocationSamples;
	int numBallLocationSamples;
	std::list<RobotLocationEstimate> RobotEstimateSet;
	std::list<BallEstimate> BallEstimateSet;
	void initRobotAndBallEstimates(Agent &agent);
};



#endif /* RAOBLACKWELLPARTICLEFILTER_H_ */




