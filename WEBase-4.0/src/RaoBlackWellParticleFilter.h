#ifndef RAOBLACKWELLPARTICLEFILTER_H_
#define RAOBLACKWELLPARTICLEFILTER_H_

#include "Agent.h"

struct RobotLocationEstimate {
	Vector roboPos;
	AngleDeg roboDir;
	float weight;
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
	float weight;

};

class RaoBlackWellParticleFilter {
public:
	RaoBlackWellParticleFilter(Agent &agent);
	RaoBlackWellParticleFilter(Agent &agent, int N, int L);
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

	std::list<RobotLocationEstimate> getNewRobotLocationEstimate();
	std::list<BallEstimate> getNewBallEstimate();

private:
	Agent & mAgent;
	int numRobotLocationSamples = 50;
	int numBallLocationSamples = 20;
	std::list<RobotLocationEstimate> RobotEstimateSet;
	std::list<BallEstimate> BallEstimateSet;
	void initRobotAndBallEstimates();
};



#endif /* RAOBLACKWELLPARTICLEFILTER_H_ */




