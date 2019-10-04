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

#define PI  3.1415926535897932 
using namespace std;
	

int main(){
	fprintf(stderr, "demo: sphere rolling up an incline \n");
	// sphere parameter and initial conditions
	double mass, radius;
	mass = 5;
	radius = 0.2;

	double slopeAngle = 30.0/180.0 * PI;
	double veloMag = 2;
	double forceMag = 10;
	double torqueMag = 10;
	vec3 initialPos(0.0, 0.0, 0.0);
	vec3 initialVelo = vec3(0, cos(slopeAngle), sin(slopeAngle)) * veloMag;
	vec3 initialOmic(1.0,0,0);
	
	double phi = 0.2;
	double n1 = 0.1;
	double n2 = 0.05;
	double n3 = sqrt(1 - pow(n1,2) - pow(n2,2));

	quaternion initialQtn = quaternion(phi, vec3(n1,n2,n3));


	Sphere mySphere = Sphere(radius, mass);	
	mySphere.setPos(initialPos);	
	mySphere.setVelo(initialVelo);
	mySphere.setOmic(initialOmic);
	mySphere.setEulerPar(initialQtn);
	

	mySphere.printOutKinematics();
	mySphere.printOutOrientation();
	vec3 force  = vec3(0.0, -cos(slopeAngle), -sin(slopeAngle)) * forceMag;
	vec3 torque = vec3(1.0, -sin(slopeAngle),  cos(slopeAngle)) * torqueMag;  

	// time step
	double dt = 0.001;
	// define a plane
	vec3 plane_n = vec3(0, -sin(slopeAngle),  cos(slopeAngle));
	vec3 offset  = vec3(0,  sin(slopeAngle), -cos(slopeAngle))*radius;
	Plane myPlane = Plane(plane_n, offset);

	// create pointers to the object
	Plane* planePtr   = &myPlane;
	Sphere* spherePtr = &mySphere;
	spherePlaneContactKinematics myContact(spherePtr, planePtr);
	myContact.printOutFrames();
	myContact.printOutContactPoints();
	// acceleration update
	mySphere.updateKinematicsFromForce(force, torque, dt);
	fprintf(stderr, "after force update. \n");
	mySphere.printOutKinematics();
	mySphere.printOutOrientation();

	myContact.updateContactKinematics();
	myContact.printOutFrames();
	myContact.printOutContactPoints();

		// frame rotation here!
	myContact.evaluateIncrementFromKinematics();
	myContact.printOutIncrements();	
	

	return 0;
}
