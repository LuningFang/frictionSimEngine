#ifndef SPHERE_H
#define SPHERE_H

#include"vec3.h"
#include"quaternion.h"
#include<iostream>
#include<stdio.h>

class Sphere{
	private:
		vec3 position, velo, omic, acc, omicDot;
		quaternion eulerP;
		
		//		vec3 position = vec3(0,0,0);
	public:
		double radius, mass, inertia;
		vec3 orientationU, orientationW, orientationN;		
		// constructor
		Sphere(double r, double m);
		void setPos(vec3 newPos);		
		void setEulerPar(quaternion newEulerP);
		void setOrientation(vec3* U, vec3* W, vec3* N);
		void setOrientation(vec3  U, vec3  W, vec3  N);	
		void setVelo(vec3 newVelo);
		void setOmic(vec3 newOmic); 
		void setAcc(vec3 newAcc);
		void setOmicDot(vec3 newOmicDot); 
		
		vec3 getPos();
		quaternion getEulerPar();
		void getOrientation(vec3* U, vec3* W, vec3* N);
		vec3 getVelo();
		vec3 getAcc();
		vec3 getOmicDot();
		vec3 getOmic();
		double getKineticEnergy();
		double getRadius(); // get radius 


		bool orientationIsOrthogonal();

		void updateKinematicsFromAcc(vec3 newAcc,   vec3 newOmicDot, double dt);
		void updateKinematicsFromForce(vec3 forces, vec3 torque,     double dt);
		void updateAccFromForce(vec3 forces, vec3 torque);
		
		vec3 expressGlobalPtInLocalRF(vec3 pt_global);
		vec3 expressLocalPtInGlobalRF(vec3 pt_local);
		vec3 orientationTimesVec(vec3 s);	
		
		
		void printOutKinematics();
		void printOutOrientation();
};
#endif
