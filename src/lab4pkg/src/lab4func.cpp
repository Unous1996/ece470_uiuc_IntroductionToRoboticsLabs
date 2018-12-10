#include "lab4pkg/lab4.h"

/*
 ** Helper functions
 */

std::vector<double> solve_2order_equations(double A, double B, double C){
    std::vector<double> rtn(2);
    if(B*B < 4*A*C){
        std::cout<<"Complex Solution"<<std::endl;
    }
    double delta = sqrt(B*B - 4*A*C);
    rtn[0] = (-B + delta)/2/A;
    rtn[1] = (-B - delta)/2/A;
    return rtn;
}

double degree2radian(double degree){
    return degree * PI / 180;
}

double radian2degree(double radian){
    return radian * 180 / PI;
}

/** 
 * function that calculates an elbow up Inverse Kinematic solution for the UR3
 */

Matrix4f DH2HT(float a, int alpha, float d, float theta)
{
    Matrix4f HT;
    cout<<endl;
    float sin_alpha, cos_alpha;
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
               // std::cout<<"HT matrix: \n"<<HT<<std::endl;
    //#endif
    return HT; 
}


/** 
 * function that calculates encoder numbers for each motor.
 */
std::vector<double> lab_fk_temp(float theta1, float theta2, float theta3, float theta4, float theta5, float theta6)
{
    std::vector<double> return_value(6);
    //std::cout<<"lab_fk called"<<std::endl;
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
    
    std::cout<<"X_actual: \n"<<(T(3,0) - 148.685/1000)*100<<std::endl;
    std::cout<<"Y_actual: \n"<<(T(3,1) + 147.8/1000)*100<< std::endl;
    std::cout<<"T="<<T<<std::endl;

    return_value[0]=theta1+PI;  // What the lab considers zero for Theta1 is 180 on the UR3 robot.
    return_value[1]=theta2;
    return_value[2]=theta3;
    return_value[3]=theta4;
    return_value[4]=theta5;
    return_value[5]=theta6;
    return lab_fk((float)theta1,(float)theta2,(float)theta3,(float)theta4,(float)theta5,(float)theta6);
}


std::vector<double> lab_invk(float xWgrip, float yWgrip, float zWgrip, float yaw_WgripDegree)
{
    bool debug = false;
	double xcen,ycen,zcen,theta6,theta5,theta4,theta3,theta2,theta1,x3end,y3end,z3end; 
	double xgrip,ygrip,zgrip;
	double a1,a2,a3,a4,a5,a6;
	double d1,d2,d3,d4,d5,d6;

    double ratio1;
    double sin_theta3, cos_theta3;
    double theta1_alpha, theta1_beta;
    double theta_b, cos_thetab;
    double theta_d;
    double Lc_square;
    double Lc;

	a1 = 0;
	d1 = 0.152;
	a2 = 0.244;
        d2 = 0.120;
	a3 = 0.213;
	d3 = -0.093;
	a4 = 0;
	d4 = 0.083;
	a5 = 0;
	d5 = 0.083;
	a6 = 0.0535;
	d6 = (0.082+0.056); 

        /*
        if(debug == true){
            xWgrip /= 1000;
            yWgrip /= 1000;
            zWgrip /= 1000;
        }
        */

        xWgrip /= 1000;
        yWgrip /= 1000;
        zWgrip /= 1000;

        yaw_WgripDegree = yaw_WgripDegree * PI / 180;
        Eigen::Matrix4d Tow;
        Eigen::Vector4d Wgrip(xWgrip, yWgrip, zWgrip, 1);


        if(debug == false){
            Tow << 1,0,0,148.685/1000,
                    0,1,0,-147.8/1000,
                    0,0,1,0,
                    0,0,0,1;
            Eigen::Vector4d grip = Tow * Wgrip;
            xgrip = grip[0];
            ygrip = grip[1];
            zgrip = grip[2];

        }

        else{
            std::cout<<"Debug mode!"<<std::endl;
            xgrip = xWgrip;
            ygrip = yWgrip;
            zgrip = zWgrip;
        }
        /*
        std::cout<<"xgrip = "<<xgrip<<std::endl;
        std::cout<<"ygrip = "<<ygrip<<std::endl;
        std::cout<<"zgrip = "<<zgrip<<std::endl;
        */
        xcen = xgrip - a6 * cos(yaw_WgripDegree);
        ycen = ygrip - a6 * sin(yaw_WgripDegree);
        zcen = zgrip;

        std::cout<<std::endl;
        /*
        std::cout<<"xcen = " << xcen << std::endl;
        std::cout<<"ycen = " << ycen << std::endl;
        std::cout<<"zcen = " << zcen << std::endl;
        */

        theta1_alpha = atan2(d2-0.01, sqrt(xcen*xcen + ycen*ycen - (d2-0.01)*(d2-0.01)));
        theta1_beta = atan2(ycen, xcen);

        //std::cout<<"theta1_beta = "<<radian2degree(theta1_beta)<<std::endl;
        //std::cout<<"theta1_alpha = "<<radian2degree(theta1_alpha)<<std::endl;

        theta1 = theta1_beta - theta1_alpha; // Default value Need to Change
        theta6 = theta1 - yaw_WgripDegree + PI/2;

        Eigen::Matrix2d Rsn;
        Rsn << cos(theta1), -sin(theta1),
               sin(theta1), cos(theta1);

        Eigen::Vector2d sendn(-d5, -d2);
        Eigen::Vector2d send;

        send = Rsn * sendn;

        x3end = send[0] + xcen;
        y3end = send[1] + ycen;
        z3end = zcen + d6;
        /*
        cout <<"x3end = " << x3end << endl;
        cout <<"y3end = " << y3end << endl;
        cout <<"z3end = " << z3end << endl;
        */

        Lc_square = (z3end - d1) * (z3end - d1) + x3end * x3end;
        Lc = sqrt(Lc_square);
        cos_theta3 = (a2 * a2 + a3 * a3 - Lc_square)/(2 * a2 * a3);
        theta3 = PI - acos(cos_theta3);
        cos_thetab = (a2 * a2 + Lc_square - a3 * a3)/(2 * a2 * Lc);
        theta_b = acos(cos_thetab);
        theta_d = atan2(z3end - d1 , x3end);
        /*
        cout<<"theta_d="<<radian2degree(theta_d)<<endl;
        cout<<"theta_b="<<radian2degree(theta_b)<<endl;
        */
        theta2  = -(theta_b + theta_d);
        theta4= -PI/2 - theta2 - theta3; // Default value Need to Change
	   theta5=-PI/2;  // Default value Need to Change

	// View values
	//use cout

        cout<<"theta1: "<< theta1*180/PI<<endl;
        cout<<"theta2: "<< theta2*180/PI<<endl;
        cout<<"theta3: "<< theta3*180/PI<<endl;
        cout<<"theta4: "<< theta4*180/PI<<endl;
        cout<<"theta5: "<< theta5*180/PI<<endl;
        cout<<"theta6: "<< theta6*180/PI<<endl;

    std::vector<double> return_value(6);
	
    return_value[0]=theta1+PI;  // What the lab considers zero for Theta1 is 180 on the UR3 robot.
    return_value[1]=theta2;
    return_value[2]=theta3;
    return_value[3]=theta4;
    return_value[4]=theta5;
    return_value[5]=theta6;// check that your values are good BEFORE sending commands to UR3
	
    return lab_fk_temp((float)theta1,(float)theta2,(float)theta3,(float)theta4,(float)theta5,(float)theta6);
}

