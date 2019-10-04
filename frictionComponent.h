// frictionComponent.h

#ifndef FRICTIONCOMPONENT_H 
#define FRICTIONCOMPONENT_H 

#include "vec3.h"
#include <string>

template <typename T>
class frictionComponent{
	public:
		std::string type;     // type of friction: sliding, rolling, spinning
		std::string mode;     // friction mode: static or kinetic
		double slackStatic;
		double slackKinetic;
		T increment, history;
		double stiffness, dampingCo, dampingSwitch;
		T elasticComponent, plasticComponent, totalFriction;
		frictionComponent();
		frictionComponent(std::string type, double ss, double st, double K, double D_co);
		void updateFrictionParameter(T delta);
		void updateFrictionForces(double dt);
		void printOut();
		
	private:
		// helper function to print out vector or double quantity
		void printOutHelper(vec3 inc);
		void printOutHelper(double inc);
		double normHelper(double t);
		double normHelper(vec3 t);

};

#endif


