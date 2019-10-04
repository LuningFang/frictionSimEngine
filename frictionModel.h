// frictionModel.h
#ifndef FRICTIONMODEL_H 
#define FRICTIONMODEL_H 

#include "Sphere.h"
#include "contactFrame.h"
#include "Plane.h"
#include "string"
// friction force and displacement relation
class frictionModel{
	private:
		Sphere* sphere_ptr;
		Plane* plane_ptr;
	public:
		double mu_s, mu_k, sliding_stiffness, eta_r, eta_s;
		vec3 normalForce;
		frictionComponent<vec3>    slidingFr;
		frictionComponent<vec3>    rollingTr;
		frictionComponent<double> spinningTr;
		double dt;
		frictionModel(Sphere* sphere_ptr, double mu_s, double mu_k, double sliding_stiffness, double eta_r, double eta_s, vec3 normalForce, double dt);
		void updateFrictionParameter(spherePlaneContactKinematics* contactKinematics);
		void updateFrictionForces(double dt);


};

#endif


