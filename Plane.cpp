#include <math.h>
#include <stdlib.h>
#include <assert.h>
#include "vec3.h"
#include "Plane.h"
#define TOL        0.00000000001
#define TIGHT_TOL  0.0000000000000001

// create a plane with a point and normal direction
Plane::Plane(vec3 n, vec3 pt){
	offset = pt;
	normal = n/n.getNorm();
	A = normal.x;
	B = normal.y;
	C = normal.z;
	
	D = -normal.innerProduct(pt);
}

//inline double Plane::getDistanceFromPoint(vec3 pt){
//	}

void Plane::getTangentPlane(vec3* u, vec3* w){
	
	assert(fabs(normal.getNorm()-1) < TOL);
	*w = vec3(int(3));

	double absX, absY, absZ;
	absX = fabs(normal.x);
	absY = fabs(normal.y);
	absZ = fabs(normal.z);
	
	// n = [ a, b, 0 ];
	if (absZ < TOL){
		if (absX > TOL){
			w->x = (-(w->y) * normal.y - (w->z) * normal.z)/(normal.x);
		}
		// n = [0, 1, 0];
		else{
			w->y = 0.0;
		}
	}
	else{
		w->z = (-(w->x)*normal.x - (w->y)*normal.y)/(normal.z);
	}

	*w = (*w)/(w->getNorm());
	*u = w->cross(normal);
}

vec3 Plane::getProjectedPoint(vec3 pt){
	assert(!this->isInPlane(pt));
	
	vec3 pt_proj = pt - (this->normal) * (this->normal).innerProduct(pt - this->offset);


	return pt_proj;
}


bool Plane::isInPlane(vec3 pt){
	double val = A * pt.x + B * pt.y + C * pt.z + D;
	bool isInPlane;
	if (val < TIGHT_TOL)
		isInPlane = true;
	else
		isInPlane = false;
	return isInPlane;
}

void Plane::printOut(){
	fprintf(stderr, "plane normal = [%f, %f, %f]\n", normal.x, normal.y, normal.z);
	fprintf(stderr, "plane equation: %f*x + %f*y + %f*z + %f = 0\n", A, B, C, D);
}



