#include "lab3pkg/lab3.h"
#include <cmath>
/** translate DH parameters to Homogeneous Transformation Matrix
 *	Matrix4f is a type of 4x4 float numbered matrix.
 *  angles are in radian.
 *  distance are in meters.
 */
Matrix4f DH2HT(float a, int alpha, float d, float theta)
{
	Matrix4f HT;
	//you will need to write the HT matrix given a, alpha, d and theta
	cout<<endl;
        //cout<<"a="<<a<<" alpha="<<alpha<<" d="<<d<<" theta="<<theta<<endl;

	/*
	HT<<cos(theta),-sin(theta)*cos(alpha),sin(theta)*sin(alpha),a*cos(theta),
		sin(theta),cos(theta)*cos(alpha),-cos(theta)*sin(alpha),a*sin(theta),
		0,sin(alpha),cos(alpha),d,
		0,0,0,1;
	*/
	float sin_alpha, cos_alpha;
	/*
	if(alpha == 0){
		sin_alpha = 0;
		cos_alpha = 1;
	}
	else if(alpha == -PI/2){
		sin_alpha = -1;
		cos_alpha = 0;
	}
	else if(alpha == PI/2){
		sin_alpha = 1;
		cos_alpha = 0;
	}
	else{
		sin_alpha = sin(alpha);
		cos_alpha = cos(alpha);
	}
	*/
	switch(alpha){
		case -1: {
			sin_alpha = -1.0;
			cos_alpha = 0.0;
			break;
		}
		case 0:{
			sin_alpha = 0.0;
			cos_alpha = 1.0;
                        break;
		}
		case 1:{
			sin_alpha = 1.0;
			cos_alpha = 0.0;
                        break;
		}
		case 2:{
			sin_alpha = 0.0;
			cos_alpha = -1.0;
                        break;
		}
	}
	HT<<cos(theta),-sin(theta)*cos_alpha,sin(theta)*sin_alpha,a*cos(theta),
		sin(theta),cos(theta)*cos_alpha,-cos(theta)*sin_alpha,a*sin(theta),
		0,sin_alpha,cos_alpha,d,
		0,0,0,1;

	//cout<<"Tag1"<<endl;
	//print out a, alpha, d, theta and HT matrix.
	//#ifdef DEBUG
                std::cout<<"HT matrix: \n"<<HT<<std::endl;
	//#endif
	return HT; 
}


/** 
 * function that calculates encoder numbers for each motor.
 */
std::vector<double> lab_fk(float theta1, float theta2, float theta3, float theta4, float theta5, float theta6)
{
	std::vector<double> return_value(6);
	std::cout<<"lab_fk called"<<std::endl;
	/*
	std::vector<double> link_length();
	std::vector<double> link_twist = {};
	std::vector<double> link_offset = {};
    */
    double link_length[6] = {0,0.224,0.213,0,0,0.0535};
    double link_twist[6] = {-PI/2,0,0,-PI/2,PI/2,PI};
	int c_link_twist[6] = {-1,0,0,-1,1,2};
    double link_offset[6] = {0.152,.120,-.093,.083,.083,0.138};

//Make sure to print text explaining all the parts of the Transformation Matrix
	//===========Implement joint angle to encoder expressions here.====================

// Uncomment these when you have enter the correct value in each Question mark.
  Matrix4f T,T1,T2,T3,T4,T5,T6;
    T1 = DH2HT(link_length[0],c_link_twist[0],link_offset[0],theta1);
    T2 = DH2HT(link_length[1],c_link_twist[1],link_offset[1],theta2);
    T3 = DH2HT(link_length[2],c_link_twist[2],link_offset[2],theta3);
    T4 = DH2HT(link_length[3],c_link_twist[3],link_offset[3],theta4);
    T5 = DH2HT(link_length[4],c_link_twist[4],link_offset[4],theta5);
    T6 = DH2HT(link_length[5],c_link_twist[5],link_offset[5],theta6);

    T = T1 * T2 * T3 * T4 * T5 * T6;

	return_value[0]=theta1+PI;  // What the lab considers zero for Theta1 is 180 on the UR3 robot.
	return_value[1]=theta2;
	return_value[2]=theta3;
	return_value[3]=theta4;
	return_value[4]=theta5;
	return_value[5]=theta6;
	return return_value;
}
