#ifndef PLANE_H
#define PLANE_H

#include"vec3.h"
#include<iostream>
#include<stdio.h>

class Plane{
	private:
		vec3 offset;
		double A, B, C, D;   // plance func: Ax+By+Cz+D = 0
	public: 
		vec3 normal;  // unit normal direction
		Plane(vec3 n, vec3 pt);

		inline double getDistanceFromPoint(vec3 pt){double dist = fabs(A*pt[0]+B*pt[1]+C*pt[2]+D)/sqrt(A*A+B*B+C*C); return dist;}
		void getTangentPlane(vec3* u, vec3* w);
		vec3 getArbitraryPoint();
		vec3 getProjectedPoint(vec3 pt);
		bool isInPlane(vec3 pt);
		void printOut();
};
#endif
