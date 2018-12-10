#include "lab3pkg/lab3.h"

/** translate DH parameters to Homogeneous Transformation Matrix
 *	Matrix4f is a type of 4x4 float numbered matrix.
 *  angles are in radian.
 *  distance are in meters.
 */
Matrix4f DH2HT(float a, float alpha, float d, float theta)
{
	Matrix4f HT;
	//you will need to write the HT matrix given a, alpha, d and theta
	HT<<cos(theta),-sin(theta),0,a,
		sin(theta)*cos(alpha),cos(theta)*cos(alpha),-sin(alpha),-d*sin(alpha),
		sin(theta)*sin(alpha),cos(theta)*sin(alpha),cos(alpha),d*cos(alpha),
		0,0,0,1;
	//print out a, alpha, d, theta and HT matrix.
	#ifdef DEBUG
		cout<<"HT matrix: "<<HT<<endl;
	#endif
	return HT; 
}


/** 
 * function that calculates encoder numbers for each motor.
 */
std::vector<double> lab_fk(float theta1, float theta2, float theta3, float theta4, float theta5, float theta6)
{
	std::vector<double> return_value(6);
    std::vector<double> link_length = {0,0.224,0.213,0,0.0533};
	std::vector<double> link_twist = {-pi/2,0,0,-pi/2,pi/2,-pi};
	std::vector<double> link_offset = {152,120,-93,83,83,138};
//Make sure to print text explaining all the parts of the Transformation Matrix
	//===========Implement joint angle to encoder expressions here.====================
// Uncomment these when you have enter the correct value in each Question mark.
    T = DH2HT(link_length[0],link_twist[0],link_offset[0],theta1)*
			DH2HT(link_length[1],link_twist[1],link_offset[1],theta2)*
			DH2HT(link_length[2],link_twist[2],link_offset[2],theta3)*
			DH2HT(link_length[3],link_twist[3],link_offset[3],theta4)*
			DH2HT(link_length[4],link_twist[4],link_offset[4],theta5)*
			DH2HT(link_length[5],link_twist[5],link_offset[5],theta6);
  
	
	cout<<"X="<<T(4,1)<<endl;
	cout<<"Y="<<T(4,2)<<endl;
	cout<<"Z="<<T(4,3)<<endl;
	cout<<"This should be 1"<<T(4,4)<<endl;

	return_value[0]=theta1+PI;  // What the lab considers zero for Theta1 is 180 on the UR3 robot.
	return_value[1]=theta2;
	return_value[2]=theta3;
	return_value[3]=theta4;
	return_value[4]=theta5;
	return_value[5]=theta6;
	return return_value;
}
