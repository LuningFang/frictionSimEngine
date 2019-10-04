// frictionModel.cpp

#include "spherePlaneContactKinematics.h"
#include "Plane.h"
#include "frictionModel.h"
#include "frictionComponent.cpp"
#include "Sphere.h"
#include "frictionComponent.h"
#include <cmath>
#include <cassert>
#include <string>
#define TOL        0.00000000001
#define TIGHT_TOL  0.0000000000000001
#define PI         3.1415926535897932

frictionModel::frictionModel(Sphere* sphere_ptr, double mu_s, double mu_k, double sliding_stiffness, double eta_r, double eta_s, vec3 normalForce, double dt){
	this->sphere_ptr		 = sphere_ptr;
	this->mu_s      		 = mu_s;
	this->mu_k      		 = mu_k;
	this->sliding_stiffness  = sliding_stiffness;
	this->eta_r              = eta_r;
	this->eta_s              = eta_s;
	this->normalForce        = normalForce;
	this->dt                 = dt;
	
	double N_mag = normalForce.getNorm();
	
	// sliding friction parameters
	double slidingSlack_kinetic = mu_k * N_mag/sliding_stiffness;
	double slidingSlack_static  = mu_s * N_mag/sliding_stiffness;
	double dampingCo = 2.0*sqrt(sphere_ptr->mass * sliding_stiffness);
	// initialzie sliding friction
	slidingFr = frictionComponent<vec3> (std::string ("sliding"), slidingSlack_kinetic, slidingSlack_static, sliding_stiffness, dampingCo);
	
	// rolling friction parameters
	double rollingSlack_kinetic = slidingSlack_kinetic/(2.0*sphere_ptr->radius);
	double rollingSlack_static  = slidingSlack_static /(2.0*sphere_ptr->radius);
	double rolling_stiffness = 4.0*eta_r*pow(sphere_ptr->radius,2)*sliding_stiffness;
	double rolling_dampingCo = 2.0* sqrt(sphere_ptr->inertia * rolling_stiffness);
	// initialize rolling friction
	rollingTr = frictionComponent<vec3> (std::string("rolling"), rollingSlack_kinetic, rollingSlack_static, rolling_stiffness, rolling_dampingCo);
	// spinning friction parameters
	double spinningSlack_kinetic = spinningSlack_kinetic * 2.0;
	double spinningSlack_static  = spinningSlack_static  * 2.0;
	double spinning_stiffness = eta_s * rolling_stiffness;
	double spinning_dampingCo = 2.0*sqrt(sphere_ptr->inertia * spinning_stiffness);
	// initialize spinning friction
	spinningTr= frictionComponent<double> (std::string("spinning"), spinningSlack_kinetic, spinningSlack_static, spinning_stiffness, spinning_dampingCo);
}


void frictionModel::updateFrictionParameter(spherePlaneContactKinematics* contactKinematics){
	(this->slidingFr).updateFrictionParameter(contactKinematics->delta);
	(this->rollingTr).updateFrictionParameter(contactKinematics->excursion);
	(this->spinningTr).updateFrictionParameter(contactKinematics->psi);
}

void frictionModel::updateFrictionForces(double dt){
	(this->slidingFr).updateFrictionForces(dt);
	(this->rollingTr).updateFrictionForces(dt);
	(this->spinningTr).updateFrictionForces(dt);
}


