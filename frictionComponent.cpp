// frictionComponent.cpp

#include "frictionComponent.h"
#include <cmath>
#include <cassert>
#include <string>
#include <iostream>
#include <stdio.h>

template <typename T>
frictionComponent<T>::frictionComponent(std::string type, double ss, double sk, double K, double D){
	this->type = type;
	this->slackStatic = ss;
	this->slackKinetic = sk;
	this->stiffness = K;
	this->dampingCo = D;
}

template <typename T>
frictionComponent<T>::frictionComponent(){
	this->mode = "static";
}

template <typename T>
void frictionComponent<T>::updateFrictionParameter(T delta){
	this->increment = delta;
//	fprintf(stderr, "increment is: \n");
//	printOutHelper(this->increment);
//	fprintf(stderr, "delta is: \n");
//	printOutHelper(delta);
	this->history = this->history + delta;
//	fprintf(stderr, "history is: \n");
//	printOutHelper(this->history);
	double slack = normHelper(history);
	double alphaStatic, alphaKinetic; 
	if ((this->mode).compare("static") == 0){
		alphaStatic = slack/this->slackStatic;
		dampingSwitch = 1.0;
		if (alphaStatic > 1.0){
			this->history = (this->history)/alphaStatic;
			this->mode = "kinetic";
			dampingSwitch = 0.0;
		}
	}
	else{
		alphaKinetic = slack/this->slackKinetic;
		if (alphaKinetic >= 1.0){
			this->history = (this->history)/alphaKinetic;
			dampingSwitch = 0.0;
		}
		else{
			this->mode = "static";
			dampingSwitch = 1.0;
		}
	}
}

template <typename T>
void frictionComponent<T>::updateFrictionForces(double dt){
	this->elasticComponent = (this->stiffness) * (this->history);
	this->plasticComponent = (this->dampingCo) * (this->dampingSwitch) * (this->increment) / dt;
	this->totalFriction = this->elasticComponent + this->plasticComponent;
}

template <typename T>
void frictionComponent<T>::printOut(){
	std::cout << "friction type: " << this->type << "; ";
	std::cout << "friction mode: " << this->mode << std::endl;
	std::cout << "static slack = " << this->slackStatic << "; kinetic slack = " << this->slackKinetic << std::endl;
	std::cout << "K = " << this->stiffness << " D = " << this->dampingCo*dampingSwitch << std::endl;
	std::cout << "increment: " << std::endl;
	printOutHelper(this->increment);
	std::cout << "history: " << std::endl;
	printOutHelper(this->history);
	std::cout << "elastic component: " << std::endl;
	printOutHelper(this->elasticComponent);
	std::cout << "plastic component: " << std::endl;
	printOutHelper(this->plasticComponent);
	std::cout << "total friction force/torque: " << std::endl;
	printOutHelper(this->totalFriction);
}	


// private helper function dealing with vec3 and double operations
template <typename T>
void frictionComponent<T>::printOutHelper(vec3 t){
	t.printOut();
}

template <typename T>
void frictionComponent<T>::printOutHelper(double t){
	std::cout << " " << t <<  std::endl;
}

template <typename T>
double frictionComponent<T>::normHelper(vec3 t){
	return t.getNorm();
}

template <typename T>
double frictionComponent<T>::normHelper(double t){
	return fabs(t);
}

template class frictionComponent<vec3>;
template class frictionComponent<double>;
