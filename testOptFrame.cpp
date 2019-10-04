// friction engine in 3D
//
#include<iostream>
#include<cmath>
#include<cstdlib>
#include<cstdio>
#include "vec3.h"
#include "contactFrame.h"

using namespace std;


int main(){

	double radius = 1.0;
	
	// create a point that is close to the bottom
	//double nx = 0.01; 
	//double ny = 0.001;
	double nx = 0.95;
	double ny = 0.01;
	double nz = sqrt(pow(radius,2) - pow(nx,2) - pow(ny,2));

	// form the point into contact normal (previous)
	vec3 prevCF_n = vec3(nx, ny, nz);
	vec3 prevCF_u, prevCF_w;

	// generate a random w vector
	double wx = -0.12;
	double wy =  0.09;
	double wz = (-nx*wx - ny*wy)/nz;
	prevCF_w = vec3(wx, wy, wz);
	prevCF_w = prevCF_w/prevCF_w.getNorm();

	// generate orthogonal u = w cross n
	prevCF_u = prevCF_w.cross(prevCF_n);

	// generate previous contact frame
	contactFrame CF_prev = contactFrame(prevCF_u, prevCF_w, prevCF_n);
	fprintf(stderr, "print out previous contact frame matrix:\n");
	CF_prev.printOutMatStyle();

	// test normal direction to be [0;0;1] first
	vec3 n_curr = vec3(1, 0, 0);

	contactFrame CF_curr;
	CF_curr = CF_prev.getContactFrameSmallestRotation(n_curr);
	fprintf(stderr, "print out current contact frame matrix:\n");
	CF_curr.printOutMatStyle();

	contactFrame CF_curr_cmp;
	CF_curr_cmp = CF_prev.getContactFrameSmallestRotation();
	fprintf(stderr, "print out current contact frame matrix (comparison):\n");
	CF_curr_cmp.printOutMatStyle();

	return 0;
}
