#include <cstdlib>
#include <iostream>
#include <cmath>
#include "vec3.h"
#include <stdio.h>
#include "frictionComponent.h"
#include <string>
using namespace std;

int main(){
	
	std::string type = "sliding";
	double ss = 0.001;
	double sk = 0.0009;

	double K = 10000.0;
	double mass = 5.0;
	double D = 2.0*sqrt(mass*K);

	frictionComponent<vec3>  slidingFriction (type, ss, sk, K, D);
	frictionComponent<vec3>* slidingFr_ptr = &slidingFriction;

	slidingFriction.printOut();

	vec3 delta (0.0005, 0, 0);
	double dt = 0.001;

	for (int i = 0; i < 5; i++){

		slidingFr_ptr->updateFrictionParameter(delta);
		slidingFr_ptr->updateFrictionForces(dt);
		fprintf(stderr, "after one time step");
		slidingFr_ptr->printOut();
	}

	for (int i = 0; i < 5; i++){
		slidingFr_ptr->updateFrictionParameter(-delta);
		slidingFr_ptr->updateFrictionForces(dt);
		fprintf(stderr, "after one time step");
		slidingFr_ptr->printOut();
	}
//	std::string type_s = "spinning";
//	double ss_s = ss * 0.1;
//	double sk_s = sk * 0.1;
//	double K_s = K * 0.1;
//	double D_s = D * 0.1;
//
//	frictionComponent<double> rollingFriction(type_s, ss_s, sk_s, K_s, D_s);
//	rollingFriction.printOutFrictionParameters();
//	
//	return 0;
}


