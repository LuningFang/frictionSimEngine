// friction engine in 3D
//
#include<iostream>
#include<math.h>
#include<stdlib.h>
#include<stdio.h>
#include<vector>
#include "vec3.h"
#include "quaternion.h"
#include "Sphere.h"
#include "contactFrame.h"
#include "Plane.h"
#include "spherePlaneContactKinematics.h"
#include "frictionComponent.h"
#include "frictionModel.h"

using namespace std;
	

int main(){

	// sphere parameter and initial conditions
	double mass, radius;
	mass = 5;
	radius = 0.2;
	vec3 initialPos(0.0, 0.0, radius);
	vec3 initialVelo(0.0, 0.5, 0.0);
	vec3 initialOmic(0.0,0.0,0.0);
	
	double phi = 0.2;
	double n1 = 0.1;
	double n2 = 0.05;
	double n3 = sqrt(1 - pow(n1,2) - pow(n2,2));

	quaternion initialQtn (phi, vec3(n1,n2,n3));


	Sphere mySphere (radius, mass);	
	mySphere.setPos(initialPos);	
	mySphere.setVelo(initialVelo);
	mySphere.setOmic(initialOmic);
	mySphere.setEulerPar(initialQtn);
	

	mySphere.printOutKinematics();
	mySphere.printOutOrientation();
	vec3 force  (0.0, 0.0, 0.0);
	vec3 torque (0.0, 0.0, 0.0);  

	// time step
	double dt = 0.0001;
	// define a plane
	vec3 plane_n  (0,0,1);
	vec3 offset   (0,0,0);
	Plane myPlane (plane_n, offset);

	// create pointers to the object
	Plane* planePtr   = &myPlane;
	Sphere* spherePtr = &mySphere;
	spherePlaneContactKinematics myContact(spherePtr, planePtr);
	spherePlaneContactKinematics* contactPtr = &myContact;
	myContact.printOutFrames();
	myContact.printOutContactPoints();
	// acceleration update
	mySphere.updateKinematicsFromForce(force, torque, dt);
	fprintf(stderr, "after force update. \n");
	mySphere.printOutKinematics();
	mySphere.printOutOrientation();

	contactPtr->updateContactKinematics();
	contactPtr->evaluateIncrementFromKinematics();


	// friction parameters
	double mu_s = 0.25;
	double mu_k = 0.2;
	double slidingK = 10000.0;
	double eta_r = 0.4;
	double eta_s = 0.06;
	double gravity = 9.8;
	vec3 normalForce (0.0, 0.0, spherePtr->mass * gravity);
	

	
	frictionModel myFriction(spherePtr, mu_s, mu_k, slidingK, eta_r, eta_s, normalForce, dt);  
	int ii;
	for (ii = 0; ii < 3; ii++){
		myFriction.updateFrictionParameter(contactPtr);
		myFriction.updateFrictionForces(dt);
		
		fprintf(stderr, "i = %d, vy = %.12f m/s,  Fe = %.12fN, Fp = %.6fN, pi = %f, pj = %f, history = %f\n", ii, (spherePtr->getVelo()).y, myFriction.slidingFr.elasticComponent.y, myFriction.slidingFr.plasticComponent.y, (contactPtr->pi).y, (contactPtr->pj).y, myFriction.slidingFr.history.y); 
		
		// don't forget to update!
		contactPtr->replacePreviousContactFrame();
		contactPtr->replacePreviousContactPoint();
		
		force = force + myFriction.slidingFr.totalFriction;
		fprintf(stderr, "force = [%f, %f, %f]\n", force.x, force.y, force.z);
		mySphere.updateKinematicsFromForce(force, torque, dt);
		contactPtr->updateContactKinematics();
		contactPtr->evaluateIncrementFromKinematics();
	
	
	
	}
	return 0;
}
