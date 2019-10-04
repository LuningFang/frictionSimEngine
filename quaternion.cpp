#include<iostream>
#include<math.h>
#include<stdio.h>
#include<stdlib.h>
#include "vec3.h"
#include "quaternion.h"
#include <cassert>

#define TOL 0.00000000001
quaternion::quaternion(){
	e0 = 1; e1 = 0; e2 = 0; e3 = 0;
}
		
quaternion::quaternion(double v1, double v2, double v3, double v4){
	e0 = v1; e1 = v2; e2 = v3; e3 = v4;
}		// 

// quaternion from rotation matrix
quaternion::quaternion(vec3 U, vec3 W, vec3 N){
	e0 = sqrt((U[0] + W[1] + N[2] + 1.0)/4.0);
	if (fabs(e0) > TOL){

		e1 = (W[2] - N[1])/(4.0*e0);
		e2 = (N[0] - U[2])/(4.0*e0);
		e3 = (U[1] - W[0])/(4.0*e0);
	}
	else{
		double a11 = U[0];
		double a22 = W[1];
		double a33 = N[2];
		e1 = sqrt((a11+1.0)/2.0);
		e2 = sqrt((a22+1.0)/2.0);
		e3 = sqrt((a33+1.0)/2.0);
	}
}

// quaternion from definition
quaternion::quaternion(double phi, vec3 n){
	
	assert(fabs(n.getNorm() - 1.0) < TOL);
	
	e0 = cos(phi/2.0);
	e1 = sin(phi/2.0) * n[0];
	e2 = sin(phi/2.0) * n[1];
	e3 = sin(phi/2.0) * n[2];
}

double quaternion::getNorm(){
	return sqrt(e0*e0 + e1*e1 + e2*e2 + e3*e3);
}

quaternion quaternion::operator+(const quaternion& v2){
	quaternion newQtn = quaternion();
	newQtn.e0 = this->e0 + v2.e0;
	newQtn.e1 = this->e1 + v2.e1;
	newQtn.e2 = this->e2 + v2.e2;
	newQtn.e3 = this->e3 + v2.e3;
	return newQtn;
}
		
quaternion quaternion::operator-(const quaternion& v2){
	quaternion newQtn = quaternion();
	newQtn.e0 = this->e0 - v2.e0;
	newQtn.e1 = this->e1 - v2.e1;
	newQtn.e2 = this->e2 - v2.e2;
	newQtn.e3 = this->e3 - v2.e3;
	return newQtn;
}

double& quaternion::operator[](int i){
	assert(i<=3);
	if (i == 0) return e0;
	if (i == 1) return e1;
	if (i == 2) return e2;
	if (i == 3) return e3;
}


void quaternion::getAfromQuaternion(vec3* U, vec3* W, vec3* N){
	vec3 u = vec3(e0*e0+e1*e1 - 0.5, e1*e2+e0*e3,     e1*e3-e0*e2)    *2;
	vec3 w = vec3(e1*e2-e0*e3,       e0*e0+e2*e2-0.5, e2*e3+e0*e1)    *2;
	vec3 n = vec3(e1*e3+e0*e2,       e2*e3-e0*e1,     e0*e0+e3*e3-0.5)*2;
	*U = u; *W = w; *N = n;
}


bool quaternion::isOrthogonal(){
	double val = e0*e0 + e1*e1 + e2*e2 + e3*e3;
	if (fabs(val-1) > TOL)
		return false;
	else
		return true;

}

// update quaternion given omic and dt
// p_n+1 = p_n + 0.5 * E(p)*omic*dt
quaternion quaternion::updateQuaternion(vec3 omic, double dt){
	double w0 = omic[0]; 
	double w1 = omic[1];
	double w2 = omic[2];

	double p0 = e0 + 0.5*(-e1*w0 - e2*w1 - e3*w2)*dt;
	double p1 = e1 + 0.5*( e0*w0 + e3*w1 - e2*w2)*dt;
	double p2 = e2 + 0.5*(-e3*w0 + e0*w1 + e1*w2)*dt;
	double p3 = e3 + 0.5*(+e2*w0 - e1*w1 + e0*w2)*dt;



	if (p0>=p1 && p0>=p2 && p0>=p3){
		p0 = p0/fabs(p0)*sqrt(1.0-p1*p1-p2*p2-p3*p3);
	}
	if (p1>=p0 && p1>=p2 && p1>=p3){
		p1 = p1/fabs(p1)*sqrt(1.0-p0*p0-p2*p2-p3*p3);
	}
	if (p2>=p0 && p2>=p1 && p2>=p3){
		p2 = p2/fabs(p2)*sqrt(1.0-p0*p0-p1*p1-p3*p3);
	}
	if (p3>=p0 && p3>=p1 && p3>=p2){
		p3 = p3/fabs(p3)*sqrt(1.0-p0*p0-p1*p1-p2*p2);
	}
	
	quaternion newQuaternion = quaternion(p0, p1, p2, p3);
	return newQuaternion;
}

//void quaternion::updateQuaternion(vec3 omic, double dt){	
//	double w0 = omic[0]; 
//	double w1 = omic[1];
//	double w2 = omic[2];
//
//	double p0 = e0 + 0.5*(-e1*w0 - e2*w1 - e3*w2)*dt;
//	double p1 = e1 + 0.5*( e0*w0 - e3*w1 + e2*w2)*dt;
//	double p2 = e2 + 0.5*( e3*w0 + e0*w1 - e1*w2)*dt;
//	double p3 = e3 + 0.5*(-e2*w0 + e1*w1 + e0*w2)*dt;
//	
//	if (p0>=p1 && p0>=p2 && p0>=p3){
//		p0 = p0/fabs(p0)*sqrt(1.0-p1*p1-p2*p2-p3*p3);
//	}
//	if (p1>=p0 && p1>=p2 && p1>=p3){
//		p1 = p1/fabs(p1)*sqrt(1.0-p0*p0-p2*p2-p3*p3);
//	}
//	if (p2>=p0 && p2>=p1 && p2>=p3){
//		p2 = p2/fabs(p2)*sqrt(1.0-p0*p0-p1*p1-p3*p3);
//	}
//	if (p3>=p0 && p3>=p1 && p3>=p2){
//		p3 = p3/fabs(p3)*sqrt(1.0-p0*p0-p1*p1-p2*p2);
//	}
//
//	e0 = p0; e1 = p1; e2 = p2; e3 = p3;
//}

// find index of the quaternion with largest magnitude
int quaternion::findMaxAbs(){
	double v0 = fabs(e0); double v1 = fabs(e1);
	double v2 = fabs(e2); double v3 = fabs(e3);

	if (v0 > v1 && v0 > v2 && v0 > v3)  return 0;
	if (v1 > v0 && v1 > v1 && v1 > v2)  return 1;
	if (v2 > v0 && v2 > v1 && v2 > v3)  return 2;
	if (v3 > v0 && v3 > v1 && v3 > v2)  return 3;
	
	fprintf(stderr, "can not find index for largest absolute val\n");
}

void quaternion::printOut(){
	fprintf(stderr, "[%f, %f, %f, %f] ", e0, e1, e2, e3);
}


