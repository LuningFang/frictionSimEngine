#ifndef SPHEREPLANECONTACTKINEMATICS_H
#define SPHEREPLANECONTACTKINEMATICS_H

#include "Sphere.h"
#include "contactFrame.h"
#include "Plane.h"
#include "frictionComponent.h"

// sphere and plane contact, only need body i
class spherePlaneContactKinematics{
	private:
		Sphere* sphere_ptr;
		Plane* plane_ptr;
		// projection of geodesic curve on sphere surface onto tangential contact plane
		vec3 sphereSurfaceGeodesicProjection();
//		vec3 nonSphereSurfaceGeodesicProjection();
	public:
		contactFrame CF_prev_global;		// contact frame in global RF at t0
		contactFrame CF_prev_local;			// contact frame in local  RF at t0
		contactFrame CF_prev_global_curr;   // previous contact frame in global RF at t1
		contactFrame CF_curr_global;        // current  contact frame in global RF at t1
		contactFrame CF_curr_local;         // current  contact frame in local  RF at t1 

		vec3 CP_prev_global;				// contact point in global RF at t0
		vec3 CP_prev_local;					// contact point in local  RF at t0
		vec3 CP_prev_global_curr;   		// previous contact point in global RF at t1
		vec3 CP_curr_global;        		// current  contact point in global RF at t1
		vec3 CP_curr_local;         		// current  contact point in local  RF at t1 

		double psi;							// relative spin
		vec3 pi, pj, pj_bar;				// contact point trajectory on body i and j (ground)
		vec3 delta, excursion;		    	// relative slide and roll of body i

		bool initiationOfContact;

		// constructor from sphere and plane object
		spherePlaneContactKinematics(Sphere* sphere_ptr, Plane* plane_ptr);
		// update contact kinemtaics after each time step
		void updateContactKinematics();
		// evaluate the increment, pi, pj and psi, ingrdients that enter into force model
		void evaluateIncrementFromKinematics();
		// replace previous contact frame and contact points after increment evaluation
		void replacePreviousContactFrame();
		void replacePreviousContactPoint();
		
		void printOutFrames();
		void printOutContactPoints();
		void printOutIncrements();
};

#endif


