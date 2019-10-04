// vec3.cpp
// 3 by 1 vector of double

#include "vec3.h"
#include "quaternion.h"
#include<cmath>
#include<cstdlib>
#include<stdio.h>
#include<ctime>
#include<cassert>

#define TIGHT_TOL    0.0000000000000001  
#define TOL          0.000001  
#define PI           3.1415926535897932

vec3::vec3(){x = 0.0; y = 0.0; z = 0.0;}
		
vec3::vec3(double v1, double v2, double v3){
			x = v1; y = v2; z = v3;
}

// generate a vector with arbitrary value (from 0 to 1)
vec3::vec3(int num){
	if (num == 1){
		x = double(rand())/(double(RAND_MAX)+1.0); y = 0.0; z = 0.0;
	}
	if (num == 2){
		x = double(rand())/(double(RAND_MAX)+1.0); 
		y = double(rand())/(double(RAND_MAX)+1.0);
		z = 0;
	}
	if (num == 3){
		x = double(rand())/(double(RAND_MAX)+1.0); 
		y = double(rand())/(double(RAND_MAX)+1.0); 
		z = double(rand())/(double(RAND_MAX)+1.0); 
//		fprintf(stderr, "generate randomized vector [%f, %f, %f]\n", x, y, z);
	}

}
double vec3::getX(){return x;}
double vec3::getY(){return y;}
double vec3::getZ(){return z;}

void vec3::setX(double newx){x = newx;}
void vec3::setY(double newy){y = newy;}
void vec3::setZ(double newz){z = newz;}

vec3 vec3::operator+(const vec3& v2){
			vec3 newVal = vec3(0,0,0);
			newVal.x = this->x + v2.x;
			newVal.y = this->y + v2.y;
			newVal.z = this->z + v2.z;
			return newVal;
}
	
vec3 vec3::operator-(const vec3& v2){
		vec3 newVal = vec3(0,0,0);
		newVal.x = this->x - v2.x;
		newVal.y = this->y - v2.y;
		newVal.z = this->z - v2.z;
		return newVal;
}

vec3 vec3::operator-(){
	vec3 newVal = vec3(-(this->x), -(this->y), -(this->z));
	return newVal;
}

void vec3::operator=(const vec3& v){
	this->x = v.x;
	this->y = v.y;
	this->z = v.z;
}

vec3 vec3::operator*(const double& sc){
			vec3 newVal = vec3(0,0,0);
			newVal.x = this->x * sc;
			newVal.y = this->y * sc;
			newVal.z = this->z * sc;
			return newVal;
}

vec3 vec3::operator/(const double& sc){
			vec3 newVal = vec3(0,0,0);
			newVal.x = this->x / sc;
			newVal.y = this->y / sc;
			newVal.z = this->z / sc;
			return newVal;
}


double& vec3::operator[](int i){
			assert( i >= 0 && i <=2);
			if (i == 0) return x;
			if (i == 1) return y;
			if (i == 2) return z;
}

void vec3::setRandomUnit(){
	int num = 3;
	vec3 rand = vec3(num);
	this->x = (rand.x)/rand.getNorm();
	this->y = (rand.y)/rand.getNorm();
	this->z = (rand.z)/rand.getNorm();
}

vec3 vec3::normalize(){
	assert(!this->isZero());
	vec3 normalized = (*this)/this->getNorm();
	return normalized;
}

double vec3::innerProduct(vec3 t){
	return t[0]*x + t[1]*y + t[2]*z;
}

vec3 vec3::cross(vec3 t){
	vec3 ans = vec3();
	ans.x = (this->y)*t.z - (this->z)*t.y;
	ans.y = (this->z)*t.x - (this->x)*t.z;
	ans.z = (this->x)*t.y - (this->y)*t.x;
	return ans;
}

double vec3::angleMagFrom(vec3 t){
	double ratio = this->innerProduct(t)/(this->getNorm()*t.getNorm());
//	fprintf(stderr, "angle from [%.32f, %.32f, %.32f] \n", x, y, z);
//	fprintf(stderr, "rotate  to [%.32f, %.32f, %.32f]", t.x, t.y, t.z);
//	fprintf(stderr, "|t| = %.32f\n", t.getNorm());
//	fprintf(stderr, "|s| = %.32f\n", this->getNorm());
//	fprintf(stderr, "ratio = %.32f\n", ratio);
	assert(ratio <= 1.0);
	return acos(ratio);
}

double vec3::angleFrom(vec3 t, vec3 dir){
	double angle = this->angleMagFrom(t);
	
	//////////////////////////////////////////////////////////
	// what about product' * dir positve or negative check? //
	// so i don't have to deal with the case where rotation//
	// is zero? /////////////////////////////////////////////
	// /////////////////////////////////////////////////////
	
	if (fabs(angle)<TIGHT_TOL)
		return 0.0;

	vec3 product = t.cross(*this);  
	product = product.normalize();
	assert(dir.isUnit());
	
//	fprintf(stderr, "direction is\n");
//	dir.printOutFloat();
//	fprintf(stderr, "product is\n");
//	product.printOutFloat();
//	fprintf(stderr, "diff is %.32f\n", (dir-product).getNorm());	
	if (fabs((product-dir).getNorm()) < TOL)
		return angle;
	if (fabs((product+dir).getNorm()) < TOL)
		return -angle;
	assert(false);
}

vec3 vec3::rotationAboutAxis(vec3 n, double alpha){
	assert(n.isUnit());
	quaternion p = quaternion(alpha, n);
	vec3 U, W, N;
	vec3* U_ptr = &U;
	vec3* W_ptr = &W;
	vec3* N_ptr = &N;

	p.getAfromQuaternion(U_ptr, W_ptr, N_ptr);

	vec3 newV;
	newV.x = this->innerProduct(vec3(U[0], W[0], N[0]));
	newV.y = this->innerProduct(vec3(U[1], W[1], N[1]));
	newV.z = this->innerProduct(vec3(U[2], W[2], N[2]));
	newV.printOutFloat();
	this->printOutFloat();
	return newV;
}

bool vec3::isUnit(){
	if (fabs(this->getNorm() - 1.0) < TIGHT_TOL)
		return true;
	else
		return false;
}

bool vec3::isZero(){
	if (fabs(this->getNorm()) < TIGHT_TOL)
		return true;
	else
		return false;
}

bool vec3::sameAs(vec3 cmp){
	vec3 diff = (*this) - cmp;
	if (diff.isZero())
		return true;
	else
		return false;
}

void vec3::printOut(){
	fprintf(stderr, "[%f, %f, %f]\n", x, y, z);
}

void vec3::printOutFloat(){
	fprintf(stderr, "[%.32f, %.32f, %.32f]\n", x, y, z);
}

