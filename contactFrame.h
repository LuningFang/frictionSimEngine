#ifndef CONTACTFRAME_H
#define CONTACTFRAME_H

#include "Sphere.h"

class contactFrame{
	public:
		vec3 u, w, n;

		contactFrame(vec3 u1, vec3 w1, vec3 n1);
		contactFrame(vec3*u1, vec3*w1, vec3*n1);
		contactFrame();
		

		// member function 
		
		// given local reference frame (orientation) [u_l,w_l,n_l],and local contact frame, express contact frame globally
		contactFrame expressContactFrameInGlobalRF(vec3* u_l, vec3* w_l, vec3* n_l);

		// given a sphere object sph, and local contact frame, express contact frame globally
		contactFrame expressContactFrameInGlobalRF(Sphere* sphere_ptr);

		// given local reference frame (orientation) [u_l,w_l,n_l],and global contact frame (itself), express contact frame locally
		contactFrame expressContactFrameInLocalRF(vec3* u_l, vec3* w_l, vec3* n_l);

		// given a sphere object sph, and local contact frame, express contact frame globally
		contactFrame expressContactFrameInLocalRF(Sphere* sphere_ptr);

		// given the normal direction, find randomized u and w to define tangential plane
		void setTangentialRandom(vec3 n1); 

		// given two vector a and b, find the rotation matrix such that [u w n]* a = b
		void getRotationMatrixFromAtoB(vec3 a, vec3 b);

		// return vector that is [u w n]*a
		vec3 multiplyVec(vec3 a);
		// return vctor that is [u w n]^T * a
		vec3 transposeMultiplyVec(vec3 a);
		// given contact frame [u,w,n] find [u1,w1,n1] such that u1*u + w1*w is maximized, n1 = [0;0;1]
		contactFrame getContactFrameSmallestRotation();
		
		// given contact frame [u w n] find new frame [u1 w1 n1] such that u1*u + w1*w is maximized, n1 can be any direction
		// note n1 does not have to be the same as n
		contactFrame getContactFrameSmallestRotation(vec3 n_curr);
		
		// smallest rotation angle from frame B to frame A (this)
		// normal direction for both frames have to be aligned
		double rotationFrom(contactFrame* B_ptr); 

		void printOut();

		void printOutMatStyle();

};

#endif


