#ifndef VEC3_H
#define VEC3_H

#include<math.h>
class vec3{
//	private:
//		double x, y, z;
	public:

		double x, y, z;
		
		
		vec3();
		vec3(double v1, double v2, double v3);
		vec3(int num); // generate a 3*1 vector with "num" of random elements
		double getX();	
		double getY();
		double getZ();
		void setX(double newx);
		void setY(double newy);
		void setZ(double newz);
		inline double getNorm(){return sqrt(x*x + y*y + z*z);}		
		inline double getNorm2(){return (x*x + y*y + z*z);}		
		vec3 operator+(const vec3& v2);
		vec3 operator-(const vec3& v2);
		vec3 operator/(const double& sc);
		vec3 operator-();
		void operator=(const vec3& v);
		
		vec3 operator*(const double& sc); // vector * scalar
		double &operator[](int i);
		

		void setRandomUnit(); // set vector to be a random unit vector
		vec3 normalize();
		double innerProduct(vec3 t); // this inner product t
		vec3 cross(vec3 t); // this cross (outer-product) t	
		double angleMagFrom(vec3 t); // return angle between this and t 
		double angleFrom(vec3 t, vec3 dir); // return rotationg angle from t to this, positive if cross product same direction as n 
		vec3 rotationAboutAxis(vec3 n, double alpha); // rotate *this* vector with respect to direction n (normalized) with an angle of alpha
		
		
		bool isUnit();   // check to see if the vector is a unit vector
	
		bool isZero();   // check to see if the vector is zero
		bool sameAs(vec3 t); // check if this vector same as t
			
		void printOut();
		void printOutFloat(); // print out with more precision		
};
// scalar * vector

inline vec3 operator*(const double& sc, const vec3& v){
			return vec3(v.x*sc, v.y*sc, v.z*sc);
}



#endif


