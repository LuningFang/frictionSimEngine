#include "contactFrame.h"
#include<math.h>
#include<cassert>
#define TOL        0.00000000001
#define TIGHT_TOL  0.0000000000000001
#define PI         3.1415926535897932
contactFrame::contactFrame(){
	u = vec3(1,0,0);
	w = vec3(0,1,0);
	n = vec3(0,0,1);
}

contactFrame::contactFrame(vec3 u1, vec3 w1, vec3 n1){
	u = u1; w = w1; n = n1;
}

contactFrame::contactFrame(vec3* u1, vec3* w1, vec3* n1){
	u = *u1; w = *w1; n = *n1;
}


// given local reference frame [u w n]_l and local contact frame (this), return a global contact frame
contactFrame contactFrame::expressContactFrameInGlobalRF(vec3* u_l, vec3* w_l, vec3* n_l){
	vec3 row1 = vec3(u_l->x, w_l->x, n_l->x);
	vec3 row2 = vec3(u_l->y, w_l->y, n_l->y);
	vec3 row3 = vec3(u_l->z, w_l->z, n_l->z);

	vec3 u_new = vec3(row1.innerProduct(u), row2.innerProduct(u), row3.innerProduct(u));
	vec3 w_new = vec3(row1.innerProduct(w), row2.innerProduct(w), row3.innerProduct(w));
	vec3 n_new = vec3(row1.innerProduct(n), row2.innerProduct(n), row3.innerProduct(n));

	contactFrame newFrame = contactFrame(u_new, w_new, n_new);
	return newFrame;

}

// given an object pointer and local contact frame, return a global contact frame
contactFrame contactFrame::expressContactFrameInGlobalRF(Sphere* spherePtr){
	contactFrame newFrame = expressContactFrameInGlobalRF(&(spherePtr->orientationU), &(spherePtr->orientationW), &(spherePtr->orientationN));
	return newFrame;
}

contactFrame contactFrame::expressContactFrameInLocalRF(vec3* u_l, vec3* w_l, vec3* n_l){
	vec3 u_new = vec3(u_l->innerProduct(u), w_l->innerProduct(u), n_l->innerProduct(u));
	vec3 w_new = vec3(u_l->innerProduct(w), w_l->innerProduct(w), n_l->innerProduct(w));
	vec3 n_new = vec3(u_l->innerProduct(n), w_l->innerProduct(n), n_l->innerProduct(n));
	contactFrame newFrame = contactFrame(u_new, w_new, n_new);
	return newFrame;
}

contactFrame contactFrame::expressContactFrameInLocalRF(Sphere* spherePtr){
	contactFrame newFrame = expressContactFrameInLocalRF(&(spherePtr->orientationU), &(spherePtr->orientationW), &(spherePtr->orientationN));
	return newFrame;
}


void contactFrame::setTangentialRandom(vec3 n1){
	this->n = n1;
	
	vec3 w1 = vec3(int(3));
	double absX, absY, absZ;
	absX = fabs(n1.x);
	absY = fabs(n1.y);
	absZ = fabs(n1.z);
	// n = [ a, b, 0 ];
	if (absZ < TOL){
		if (absX > TOL){
			w1.x = (-(w1.y) * n1.y - (w1.z) * n1.z)/(n1.x);
		}
		// n = [0, 1, 0];
		else{
			w1.y = 0.0;
		}
	}
	else{
		w1.z = (-(w1.x)*n1.x - (w1.y)*n1.y)/(n1.z);
	}
	

	w = w1/w1.getNorm();
	this->u = w.cross(n);
}

void contactFrame::getRotationMatrixFromAtoB(vec3 a, vec3 b){
	assert(a.isUnit() && b.isUnit());
	// a same direction as b
	if ((a-b).getNorm() < TIGHT_TOL){
		this->u = vec3( 1, 0, 0);
		this->w = vec3( 0, 1, 0);
		this->n = vec3( 0, 0, 1);
		return;
	}
	
	// a opposite direction as b
	if ((a+b).getNorm() < TIGHT_TOL){
		this->u = vec3(-1, 0, 0);
		this->w = vec3( 0,-1, 0);
		this->n = vec3( 0, 0,-1);
		return;
	}

	// none above
	vec3 v = a.cross(b);
	double v1, v2, v3, v1v2, v1v3, v2v3, v1_s, v2_s, v3_s;
	v1 = v.x; v2 = v.y; v3 = v.z;
	v1v2 = v1*v2; v2v3 = v2*v3; v1v3 = v1*v3;
	v1_s = pow(v1,2); v2_s = pow(v2,2); v3_s = pow(v3,2);
	double s, c, p;	
	s = v.getNorm();
	c = a.innerProduct(b);
	p = (1.0 - c)/pow(s,2);
	this->u = vec3(1.0 - (v3_s+v2_s)*p,  v3 +        v1v2*p, -v2 +        v1v3*p);
	this->w = vec3(-v3 +        v1v2*p, 1.0 - (v3_s+v1_s)*p,  v1 +        v2v3*p);
	this->n = vec3( v2 +        v1v3*p, -v1 +        v2v3*p, 1.0 - (v2_s+v1_s)*p); 
}

vec3 contactFrame::multiplyVec(vec3 a){
	vec3 row1 = vec3(u.x, w.x, n.x);
	vec3 row2 = vec3(u.y, w.y, n.y);
	vec3 row3 = vec3(u.z, w.z, n.z);
	vec3 b = vec3(row1.innerProduct(a), row2.innerProduct(a), row3.innerProduct(a));
	return b;
}

vec3 contactFrame::transposeMultiplyVec(vec3 a){
	vec3 b = vec3((this->u).innerProduct(a), (this->w).innerProduct(a), (this->n).innerProduct(a));
	return b;
}

contactFrame contactFrame::getContactFrameSmallestRotation(){
	double a1, a2, b1, b2, theta_star, costFunc;
	a1 = (this->u).x; a2 = (this->u).y;
	b1 = (this->w).x; b2 = (this->w).y;

	if (fabs(a1+b2) < TIGHT_TOL){
		theta_star = PI/2.0;
	}
	else{
		theta_star = atan((a2-b1)/(a1+b2));
	}
	
	//***************************************************//
	//figure out a way to optimize this, seems expenseive//
	//***************************************************//
	costFunc = (a1+b2)*cos(theta_star) + (a2-b1)*sin(theta_star);

	if (costFunc < 0)
		theta_star = theta_star + PI;
	
	contactFrame newFrame;
	newFrame.u = vec3( cos(theta_star), sin(theta_star), 0);
	newFrame.w = vec3(-sin(theta_star), cos(theta_star), 0);
	newFrame.n = vec3( 0, 0, 1);
	return newFrame;
}

contactFrame contactFrame::getContactFrameSmallestRotation(vec3 n_curr){
	contactFrame rotationA;
	vec3 global_normal = vec3(0,0,1);
	// rotation matrix A*n_curr = [0;0;1];
	rotationA.getRotationMatrixFromAtoB(n_curr, global_normal);
	contactFrame rotatedFrame_prev;
	rotatedFrame_prev.u = rotationA.multiplyVec(this->u);
	rotatedFrame_prev.w = rotationA.multiplyVec(this->w);

	contactFrame CF_curr, CF_rotated;
	CF_curr = rotatedFrame_prev.getContactFrameSmallestRotation();
	CF_rotated.u = rotationA.transposeMultiplyVec(CF_curr.u);
	CF_rotated.w = rotationA.transposeMultiplyVec(CF_curr.w);
	CF_rotated.n = rotationA.transposeMultiplyVec(CF_curr.n);
	
	return CF_rotated;	
}

double contactFrame::rotationFrom(contactFrame* B){
	assert((B->n).sameAs(this->n));
	// scenatrio where this->u and B->i align
		
	double angle = (this->u).angleFrom(B->u, B->n);
	return angle;
	
}
void contactFrame::printOut(){
	fprintf(stderr, "contact frame:\n");
	fprintf(stderr, "u = "); u.printOut();
	fprintf(stderr, "w = "); w.printOut();
	fprintf(stderr, "n = "); n.printOut();
}

void contactFrame::printOutMatStyle(){
	fprintf(stderr, "contact frame matrix:\n");
	fprintf(stderr, "[%f, %f, %f]\n", u.x, w.x, n.x);
	fprintf(stderr, "[%f, %f, %f]\n", u.y, w.y, n.y);
	fprintf(stderr, "[%f, %f, %f]\n", u.z, w.z, n.z);
}
