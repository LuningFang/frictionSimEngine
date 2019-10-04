#ifndef QUATERNION_H
#define QUATERNION_H


#include "vec3.h"
class quaternion{
//	private:
//		double e0, e1, e2, e3;
	public:
	double e0, e1, e2, e3;	
	// constructor
	quaternion();
	quaternion(double v1, double v2, double v3, double v4);
	quaternion(vec3  U, vec3  W, vec3  N);  //quaternion from rotation matrix
	quaternion(double phi, vec3 n); // quaternion from definition
	
	
	//
	double getNorm();
	quaternion operator+(const quaternion& v2);
	quaternion operator-(const quaternion& v2);
	double &operator[](int i);
	void getAfromQuaternion(vec3* U, vec3* W, vec3* N);
	quaternion getQuaternionFromA(vec3 U, vec3 W, vec3 N);
	quaternion getQuaternionFromA(vec3*U, vec3*W, vec3*N);
	bool isOrthogonal();
	// return new quaternion, does not change member value
	quaternion updateQuaternion(vec3 omic, double dt);
	// update member value
	//void updateQuaternion(vec3 omic, double dt);	
	int findMaxAbs();
	
	void printOut();
};
#endif
