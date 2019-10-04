#include <math.h>
#include <stdlib.h>

#include "Sphere.h"
#define TOL 0.00000000001

Sphere::Sphere(double r, double m){
	radius = r;
	mass = m;
//	position = vec3(0,0,0)
	orientationU = vec3(1,0,0);
	orientationW = vec3(0,1,0);
	orientationN = vec3(0,0,1);
	eulerP = quaternion(1.0, 0.0, 0.0, 0.0);	
	velo = vec3(0,0,0);
	omic = vec3(0,0,0);
	acc = vec3(0,0,0);
	omicDot = vec3(0, 0, 0);
	inertia = 0.4*mass*radius*radius;
	fprintf(stderr, "sphere object created\n");
}

void Sphere::setPos(vec3 newPos){position = newPos;}  // set position
vec3 Sphere::getPos(){return position;} // get position

void Sphere::setVelo(vec3 newVelo){this->velo = newVelo;} // set velocity
vec3 Sphere::getVelo(){ return this->velo;} //get velocity

void Sphere::setAcc(vec3 newAcc){this->acc = newAcc;} // set acceleration`
vec3 Sphere::getAcc(){return this->acc;} // get acceleration

void Sphere::setOmic(vec3 newOmic){this->omic = newOmic;} // set omic
vec3 Sphere::getOmic(){return this->omic;} // get omic


void Sphere::setOmicDot(vec3 newOmicDot){this->omicDot = newOmicDot;} // set omic dot
vec3 Sphere::getOmicDot(){return this->omicDot;} // get omic  dot

double Sphere::getRadius(){return this->radius;} // get radius 

// set euler parameter and update orientation matrix
void Sphere::setEulerPar(quaternion newEulerP){
	this->eulerP = newEulerP;
	newEulerP.getAfromQuaternion(&orientationU, &orientationW, &orientationN);	
	if ((*this).orientationIsOrthogonal())
		fprintf(stderr, "new orientation is healthy\n");
	else
		fprintf(stderr, "new orientation is not healthy, need a different euler parameter.\n");

} 
// get euler parameter
quaternion Sphere::getEulerPar(){return this->eulerP;} // get omic  dot

// set orientation matrix and update euler parameter
void Sphere::setOrientation(vec3* U, vec3* W, vec3* N){
	orientationU = *U; orientationW = *W; orientationN = *N;  	
	eulerP = quaternion(*U, *W, *N);	
}

// set orientation matrix and update euler parameter
void Sphere::setOrientation(vec3 U, vec3 W, vec3 N){
	orientationU = U; orientationW = W; orientationN = N;  	
	eulerP = quaternion(U, W, N);

}

void Sphere::getOrientation(vec3* U, vec3* W, vec3* N){
	(*U) = orientationU; (*W) = orientationW; (*N) = orientationN;
}

double Sphere::getKineticEnergy(){
	return (0.5*mass*velo.getNorm2() + 0.5*inertia*omic.getNorm2());
}


bool Sphere::orientationIsOrthogonal(){
	double uu = fabs(orientationU.getNorm()-1);
	double ww = fabs(orientationW.getNorm()-1);
	double nn = fabs(orientationN.getNorm()-1);
	double uw = fabs(orientationU.innerProduct(orientationW));
	double un = fabs(orientationU.innerProduct(orientationN));
	double wn = fabs(orientationW.innerProduct(orientationN));

	if (false)
	fprintf(stderr, "uu=%f, ww=%f, nn=%f, uw=%f, un=%f, wn=%f\n", uu, uw, nn, uw, un, wn);

	if (uu < TOL && ww < TOL && nn < TOL && uw < TOL && un < TOL && wn < TOL)
		return true;
	else
		return false;

}

void Sphere::updateKinematicsFromAcc(vec3 newAcc, vec3 newOmicDot, double dt){
	this->acc      = newAcc;
	this->omicDot  = newOmicDot;
	this->velo     = this->velo     + this->acc      * dt;
	this->omic     = this->omic     + this->omicDot  * dt;
	this->position = this->position + this->velo * dt;

	quaternion newP = (this->eulerP).updateQuaternion(this->omic, dt);
	this->eulerP = newP;
	(newP).getAfromQuaternion(&orientationU, &orientationW, &orientationN);

}


void Sphere::updateKinematicsFromForce(vec3 forces, vec3 torque, double dt){
	updateAccFromForce(forces, torque);
	updateKinematicsFromAcc(this->acc, this->omicDot, dt);

}

void Sphere::updateAccFromForce(vec3 forces, vec3 torque){
	vec3 newAcc     = vec3(forces[0]/mass,    forces[1]/mass,    forces[2]/mass);
	vec3 newOmicDot = vec3(torque[0]/inertia, torque[1]/inertia, torque[2]/inertia);
	this->acc     = newAcc;
	this->omicDot = newOmicDot;

}

// given any global point, find coordinates expressed in local orientation (reference frame)
vec3 Sphere::expressGlobalPtInLocalRF(vec3 pt_global){
	vec3 diff = pt_global - position;
	double px = orientationU.innerProduct(diff);
	double py = orientationW.innerProduct(diff);
	double pz = orientationN.innerProduct(diff);
	vec3 pt_local = vec3(px, py, pz);
	return pt_local;
}

// given any local point, find the point expressed in global reference frame
vec3 Sphere::expressLocalPtInGlobalRF(vec3 pt_local){
	
	vec3 As = orientationTimesVec(pt_local);
	return (position + As);
	
}

// return A*s = [U W N]*s
vec3 Sphere::orientationTimesVec(vec3 s){
	
	double px = orientationU[0]*s[0] + orientationW[0]*s[1] + orientationN[0]*s[2];
	double py = orientationU[1]*s[0] + orientationW[1]*s[1] + orientationN[1]*s[2];
	double pz = orientationU[2]*s[0] + orientationW[2]*s[1] + orientationN[2]*s[2];
	return vec3(px, py, pz);
}


void Sphere::printOutKinematics(){
	fprintf(stderr, "pos  =");   this->position.printOut(); fprintf(stderr, "\n");
	fprintf(stderr, "velo =");   this->velo.printOut();     fprintf(stderr, "\n");
	fprintf(stderr, "acc  =");   this->acc.printOut();      fprintf(stderr, "\n");
	fprintf(stderr, "qtn  =");   this->eulerP.printOut();     fprintf(stderr, "\n");
	fprintf(stderr, "omic =");   this->omic.printOut();     fprintf(stderr, "\n");
	fprintf(stderr, "omicDot =");this->omicDot.printOut();  fprintf(stderr, "\n");
}


void Sphere::printOutOrientation(){
	fprintf(stderr, "orientation matrix:\n");
	fprintf(stderr, "A_u = "); orientationU.printOut(); fprintf(stderr, "\n");
	fprintf(stderr, "A_w = "); orientationW.printOut(); fprintf(stderr, "\n");
	fprintf(stderr, "A_n = "); orientationN.printOut(); fprintf(stderr, "\n");
}


