#include "spherePlaneContactKinematics.h"
#include "Plane.h"
#include "Sphere.h"
#include <cmath>
#include <cassert>

#define TOL        0.00000000001
#define TIGHT_TOL  0.0000000000000001
#define PI         3.1415926535897932


// initialize contact
spherePlaneContactKinematics::spherePlaneContactKinematics(Sphere* SpherePtr, Plane* PlanePtr){
	initiationOfContact = true;
	this->sphere_ptr = SpherePtr;
	this->plane_ptr = PlanePtr;
	// sphere plane contact, normal direction the same as the the normal of the plane
	CF_prev_global.n = plane_ptr->normal;
	// randomize the tangential plane at the initiation of the contact
	CF_prev_global.setTangentialRandom(CF_prev_global.n);
	CF_prev_global.printOutMatStyle();
	// express previou global contact frame in local coordinate
	CF_prev_local = CF_prev_global.expressContactFrameInLocalRF(sphere_ptr);
	// initialzie CF at current time to be the same as previous ones
	CF_prev_global_curr = CF_prev_global;
	CF_curr_global = CF_prev_global;
	CF_curr_local  = CF_prev_local;
	// sphere plane contact, contact point is projecting the center of the sphere to the plane
	CP_prev_global = plane_ptr->getProjectedPoint(sphere_ptr->getPos());
	CP_prev_local  = sphere_ptr->expressGlobalPtInLocalRF(CP_prev_global);
	// initialzie contact point at current time to be the same as previous ones
	CP_prev_global_curr = CP_prev_global;
	CP_curr_global = CP_prev_global;
	CP_curr_local  = CP_prev_local;
}                 

void spherePlaneContactKinematics::updateContactKinematics(){
	// express previous contact frame globally
	CF_prev_global_curr = CF_prev_local.expressContactFrameInGlobalRF(sphere_ptr);
	// express previous contact point globally
	CP_prev_global_curr = sphere_ptr->expressLocalPtInGlobalRF(CP_prev_local);
	// get current contact point
	CP_curr_global = plane_ptr->getProjectedPoint(sphere_ptr->getPos());
	// use optimization method to find current contact frame with minimum rotation
	CF_curr_global = CF_prev_global_curr.getContactFrameSmallestRotation(plane_ptr->normal);
	// find current contact point in local reference frame
	CP_curr_local = sphere_ptr->expressGlobalPtInLocalRF(CP_curr_global);
	// find current contac
	CF_curr_local = CF_curr_global.expressContactFrameInLocalRF(sphere_ptr);
}

void spherePlaneContactKinematics::replacePreviousContactFrame(){
	this->CF_prev_global = this->CF_curr_global;
	this->CF_prev_local  = this->CF_curr_local;
}

void spherePlaneContactKinematics::replacePreviousContactPoint(){
	this->CP_prev_global = this->CP_curr_global;
	this->CP_prev_local  = this->CP_curr_local;
}

vec3 spherePlaneContactKinematics::sphereSurfaceGeodesicProjection(){
	vec3 s_proj, chord, Ra, Rb, proj_chord_n;
	double ratio, angle, arc_length;
	chord = CP_curr_global - CP_prev_global_curr;
	Ra = CP_prev_global_curr - sphere_ptr->getPos();
	Rb = CP_curr_global - sphere_ptr->getPos();
	// use |Ra|*|Rb| instead of R^2
	// because when deformation is allowed in normal
	// direction, |Ra|!=R
	ratio = Ra.innerProduct(Rb)/(Ra.getNorm() * Rb.getNorm());
	assert(ratio <= 1.0);
	angle = acos(ratio);
	arc_length = Ra.getNorm() * angle;
	assert(fabs(CF_curr_global.n.getNorm() - 1) < TOL);
	proj_chord_n = chord - chord.innerProduct(CF_curr_global.n)*CF_curr_global.n;
	
	if (fabs(proj_chord_n.getNorm()) < TIGHT_TOL){
		s_proj = vec3(0,0,0);
	}
	else{
		s_proj = proj_chord_n/proj_chord_n.getNorm() * arc_length;
	}
	return s_proj;
}

void spherePlaneContactKinematics::evaluateIncrementFromKinematics(){
	pi = this->sphereSurfaceGeodesicProjection();
	pj_bar = CP_curr_global - CP_prev_global;
	// find relative rotation of the tangetial plane (psi)
	// body i CF at current time (u1, w1, n1)
	// body j CF at current time (u1_bar, w1_bar, n1)
	// share same n1
	// when j is the ground [u1_bar, w1_bar] = body i global CF at previous time
	psi = CF_curr_global.rotationFrom(&CF_prev_global);	
	pj = pj_bar.rotationAboutAxis(CF_curr_global.n, -psi);
	delta  = pj - pi;
	excursion = pi/sphere_ptr->getRadius();
}

void spherePlaneContactKinematics::printOutFrames(){
	fprintf(stderr, "------contact frame info------\n");	
	fprintf(stderr, "previous contact frame in GRF\n");
	CF_prev_global.printOutMatStyle();
	fprintf(stderr, "previous contact frame in LRF\n");
	CF_prev_local.printOutMatStyle();
	fprintf(stderr, "previous contact frame in GRF after rotation\n");
	CF_prev_global_curr.printOutMatStyle();
	fprintf(stderr, "current contact frame in GRF\n");
	CF_curr_global.printOutMatStyle();
	fprintf(stderr, "current contact frame in LRF\n");
	CF_curr_local.printOutMatStyle();
}


void spherePlaneContactKinematics::printOutContactPoints(){
	fprintf(stderr, "------contact point info------\n");	
	fprintf(stderr, "previous contact point in GRF\n");
	CP_prev_global.printOut();
	fprintf(stderr, "previous contact point in LRF\n");
	CP_prev_local.printOut();
	fprintf(stderr, "previous contact point in GRF after rotation\n");
	CP_prev_global_curr.printOut();
	fprintf(stderr, "current contact point in GRF\n");
	CP_curr_global.printOut();
	fprintf(stderr, "current contact point in LRF\n");
	CP_curr_local.printOut();
}

void spherePlaneContactKinematics::printOutIncrements(){
	fprintf(stderr, "------increments from contacts-----\n");
	fprintf(stderr, "relative slide is:\n");
	delta.printOutFloat();
	fprintf(stderr, "shithole\n");
	pj_bar.printOutFloat();
	fprintf(stderr, "pi = [%.32f, %.32f, %.32f], pj = [%.32f, %.32f, %.32f], pj_bar = [%.32f, %.32f, %.32f]\n", pi.x, pi.y, pi.z, pj.x, pj.y, pj.z, pj_bar.x, pj_bar.y, pj_bar.z);
	fprintf(stderr, "relative spin is: %f\n", psi);
	fprintf(stderr, "relative roll is:\n");
	excursion.printOutFloat();
}
