/*
 * LSPIWeight.cpp
 *
 *  Created on: Oct 26, 2013
 *      Author: juekun
 */

#include "LSPIWeight.h"
#include <stdlib.h>
#include <math.h>
using namespace std;


LSPIWeight::LSPIWeight(LSPI& L): lspi(L)
{
	this->abs_OriBall2Robot=lspi.getAbsOriBall2Robot();
	this->BallposEntropy=lspi.BallEntropy();
	this->RobotposEntropy=lspi.RobotEntropy();
	this->RobotOri2GoalEntropy=lspi.DirRob2GoalEntropy();
	this->sensecost=lspi.getSenseCost(1,1);//TO DO
	this->dim=lspi.getDim();
	this->A=new float[this->dim][this->dim];
	this->b=new float[this->dim];
}

float* LSPIWeight::EntryConcatenation(float abs_OriBall2Robot, list<float> BallposEntropy, list<float> RobotposEntropy, list<float> RobotOri2GoalEntropy, float sensecost)
{
	list<float> t;
	t.insert(t.end(),abs_OriBall2Robot);
	t.insert(t.end(),BallposEntropy.begin(),BallposEntropy.end());
	t.insert(t.end(),RobotposEntropy.begin(),RobotposEntropy.end());
	t.insert(t.end(),RobotOri2GoalEntropy.begin(),RobotOri2GoalEntropy.end());
	t.insert(t.end(),sensecost);

	float* arr = new float[t.size()];
	copy(t.begin(),t.end(),arr);
	this->dim=t.size();
	return arr;
}

float** LSPIWeight::getA(float* Fi_current, float* Fi_next)
{
	int dim=this->dim;
	float gamma=this->gamma;
	float** temp = this->A;
	for (int i=0;i<dim;i++)
		for(int j=0;j<dim;j++)
		{
			for(int k=0;k<dim;k++)
				temp[i][j]+=Fi_current[i]*Fi_current[j]-gamma*Fi_current[i]*Fi_next[j];
		}
	return temp;
}

float* LSPIWeight::getb(float* Fi_current)
{
	int dim=this->dim;
	float reward=this->reward;
	float* temp=this->b;
	for(int i=0;i<dim;i++)
		temp[i]+=Fi_current[i]*reward;
	return temp;
}

float* LSPIWeight::WeightUpdate(float** A, float* b)
{
	int dim=this->dim;
	float* weight=new float[dim];
	float** A_inv=inverse(A);
	for (int i=0;i<dim;i++)
		for (int j=0;j<dim;j++)
		{
			weight[i]+=A_inv[i][j]*b[j];
		}
	return weight;
}

float** inverse(float** A)
{
	int dim=this->dim;
	float** A_inv=new float[dim][dim];
	float ratio,a;
	int i,j,k;

	//copy A to A_inv
	for(i=0;i<dim;i++)
		for(j=0;j<dim;j++)
			A_inv[i][j]=A[i][j];
	//http://www.programming-techniques.com/2011/09/numerical-methods-inverse-of-nxn-matrix.html
	//Numerical method for inverse of n-n matrix
	for(i = 0; i < dim; i++)
	{
	        for(j = dim; j < 2*dim; j++){
	            if(i==(j-dim))
	                A_inv[i][j] = 1.0;
	            else
	                A_inv[i][j] = 0.0;
	        }
	}

	for(i = 0; i < dim; i++){
	        for(j = 0; j < dim; j++){
	            if(i!=j){
	                ratio = A_inv[j][i]/A_inv[i][i];
	                for(k = 0; k < 2*dim; k++){
	                	A_inv[j][k] -= ratio * A_inv[i][k];
	                }
	            }
	        }
	    }

	for(i = 0; i < dim; i++){
	        a = A_inv[i][i];
	        for(j = 0; j < 2*dim; j++){
	        	A_inv[i][j] /= a;
	        }
	    }
	return A_inv;
}


