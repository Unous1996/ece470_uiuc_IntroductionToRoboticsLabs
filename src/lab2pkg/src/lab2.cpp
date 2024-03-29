#include "lab2pkg/lab2.h" 
#define PI 3.14159265359
#define SPIN_RATE 20  /* Hz */

//global status variable
int Status[] = {0,0,0};

//arrays defining Waypoints
double home[]={120*PI/180,-90*PI/180,90*PI/180,-90*PI/180,-90*PI/180,0*PI/180};

double arr11[]={120*PI/180,-56*PI/180,124*PI/180,-158*PI/180,-90*PI/180,0*PI/180};
double arr12[]={120*PI/180,-64*PI/180,123*PI/180,-148*PI/180,-90*PI/180,0*PI/180};
double arr13[]={120*PI/180,-72*PI/180,120*PI/180,-137*PI/180,-90*PI/180,0*PI/180};
double arr14[]={120*PI/180,-78*PI/180,115*PI/180,-125*PI/180,-90*PI/180,0*PI/180};

double A3[]={140.62*PI/180,-65.71*PI/180,113.97*PI/180,-140.63*PI/180,-90.22*PI/180,46.41*PI/180};
double A2[]={140.62*PI/180,-59.06*PI/180,113.76*PI/180,-144.36*PI/180,-90.24*PI/180,46.41*PI/180};
double A1[]={140.62*PI/180,-51.98*PI/180,111.07*PI/180,-145.84*PI/180,-90.28*PI/180,46.41*PI/180};
double AH[]={138.75*PI/180,-83.72*PI/180,108.48*PI/180,-121.61*PI/180,-90.72*PI/180,46.40*PI/180};

double B3[]={153.54*PI/180,-69.70*PI/180,118.32*PI/180,-138.62*PI/180,-90.25*PI/180,52.08*PI/180};
double B2[]={154.73*PI/180,-62.66*PI/180,121.66*PI/180,-150.36*PI/180,-91.17*PI/180,52.11*PI/180};
double B1[]={155.23*PI/180,-55.59*PI/180,120.21*PI/180,-154.62*PI/180,-91.17*PI/180,52.11*PI/180};
double BH[]={154.10*PI/180,-80.05*PI/180,115.96*PI/180,-126.52*PI/180,-89.04*PI/180,52.11*PI/180};

double C3[]={178.18*PI/180,-67.48*PI/180,114.95*PI/180,-138.09*PI/180,-90.02*PI/180,80.67*PI/180};
double C2[]={177.97*PI/180,-61.45*PI/180,114.93*PI/180,-140.98*PI/180,-91.06*PI/180,80.67*PI/180};
double C1[]={176.78*PI/180,-54.60*PI/180,119.82*PI/180,-155.42*PI/180,-89.03*PI/180,80.67*PI/180};
double CH[]={177.61*PI/180,-82.48*PI/180,107.85*PI/180,-114.76*PI/180,-89.46*PI/180,87.13*PI/180};

// array to define final velocity of point to point moves.  For now slow down to zero once 
// each point is reached
double arrv[]={0,0,0,0,0,0};

//vectors to be used to publish commands to UR3 ROS Driver (ece470_ur3_driver)
std::vector<double> QH (home,home+sizeof(home) / sizeof(home[0]));

std::vector<double> Q11 (arr11,arr11+sizeof(arr11) / sizeof(arr11[0]));
std::vector<double> Q12 (arr12,arr12+sizeof(arr12) / sizeof(arr12[0]));
std::vector<double> Q13 (arr13,arr13+sizeof(arr13) / sizeof(arr13[0]));
std::vector<double> Q14 (arr14,arr14+sizeof(arr14) / sizeof(arr14[0]));

std::vector<double> QA1 (A1,A1+sizeof(A1) / sizeof(A1[0]));
std::vector<double> QA2 (A2,A2+sizeof(A2) / sizeof(A2[0]));
std::vector<double> QA3 (A3,A3+sizeof(A3) / sizeof(A3[0]));
std::vector<double> QAH (AH,AH+sizeof(AH) / sizeof(AH[0]));

std::vector<double> QB1 (B1,B1+sizeof(B1) / sizeof(B1[0]));
std::vector<double> QB2 (B2,B2+sizeof(B2) / sizeof(B2[0]));
std::vector<double> QB3 (B3,B3+sizeof(B3) / sizeof(B3[0]));
std::vector<double> QBH (BH,BH+sizeof(BH) / sizeof(BH[0]));

std::vector<double> QC1 (C1,C1+sizeof(C1) / sizeof(C1[0]));
std::vector<double> QC2 (C2,C2+sizeof(C2) / sizeof(C2[0]));
std::vector<double> QC3 (C3,C3+sizeof(C3) / sizeof(C3[0]));
std::vector<double> QCH (CH,CH+sizeof(CH) / sizeof(CH[0]));

std::vector<double> v (arrv,arrv+sizeof(arrv) / sizeof(arrv[0]));

// creating an array of these vectors allows us to iterate through them
// and programatically choose where to go.
std::vector<double> Q [3][3] = {
    {Q11, Q12, Q13},
    {Q11, Q12, Q13},
    {Q11, Q12, Q13}
};

std::vector<double> Spots[3][4] = {
    {QA1, QA2, QA3, QAH},
    {QB1, QB2, QB3, QBH},
    {QC1, QC2, QC3, QCH}
};

// Global bool variables that are assigned in the callback associated when subscribed 
// to the "ur3/position" topic
bool isReady=1;
bool pending=0;

// Whenever ur3/position publishes info this callback function is run.
void position_callback(const ece470_ur3_driver::positions::ConstPtr& msg)
{
	isReady=msg->isReady; // When isReady is True the robot arm has made it to its desired position
						  // and is ready to be told to go to another point if desired.
	pending=msg->pending; // pending is the opposite of isReady, pending is true until a new position is reached
//	ROS_INFO("Debug isRdy = %d, pending = %d",(int)isReady,(int)pending);
}


int setOn(ur_msgs::SetIO srv, ros::ServiceClient srv_SetIO){
    ROS_INFO("Setting On the digital output");
    srv.request.fun = 1;
    srv.request.pin = 0;  //Digital Output 0
    srv.request.state = 1.0; //Set DO0 on
    if (srv_SetIO.call(srv)) {
            ROS_INFO("True: Switched Suction ON");
    } else {
            ROS_INFO("False");
    }
    return 0;
}

int setOff(ur_msgs::SetIO srv, ros::ServiceClient srv_SetIO){
    ROS_INFO("Setting off the digital output");
    srv.request.fun = 1;
    srv.request.pin = 0;  //Digital Output 0
    srv.request.state = 0.0; //Set DO0 on
    if (srv_SetIO.call(srv)) {
            ROS_INFO("True: Switched Suction OFF");
    } else {
            ROS_INFO("False");
    }
    return 0;
}

int move_arm(	ros::Publisher pub_command , ros::Rate loop_rate, std::vector<double> dest, float duration)
{
    ece470_ur3_driver::command driver_msg;
    driver_msg.destination = dest;
    pub_command.publish(driver_msg);
    ROS_INFO("Moving to a waypoint");
    int spincount = 0;
    while (isReady) { // Waiting for isReady to be false meaning that the driver has the new command
            ros::spinOnce();  // Allow other ROS functionallity to run
            loop_rate.sleep(); // Sleep and wake up at 1/20 second (1/SPIN_RATE) interval
            if (spincount > SPIN_RATE) {  // if isReady does not get set within 1 second re-publish
                    pub_command.publish(driver_msg);
                    ROS_INFO("Just Published again driver_msg");
                    spincount = 0;
            }
            spincount++;  // keep track of loop count
    }

    while(!isReady)
    {
            ros::spinOnce();
            loop_rate.sleep();
    }
    return 0;
}

int move_block(ros::Publisher pub_command ,
                ros::Rate loop_rate,
                ros::ServiceClient srv_SetIO,
                ur_msgs::SetIO srv,
                int start_loc,int end_loc)
{
    ROS_INFO("Moving blocks");
    move_arm(pub_command, loop_rate, Spots[start_loc][3], 1.0);
    setOn(srv, srv_SetIO);
    move_arm(pub_command, loop_rate, Spots[start_loc][Status[start_loc]-1], 1.0);
    move_arm(pub_command, loop_rate, Spots[start_loc][3], 1.0);
    Status[start_loc]--;

    move_arm(pub_command, loop_rate, Spots[end_loc][3], 1.0);
    ROS_INFO("Moving to ");
    move_arm(pub_command, loop_rate, Spots[end_loc][Status[end_loc]], 1.0);
    setOff(srv, srv_SetIO);
    move_arm(pub_command, loop_rate, Spots[end_loc][3], 1.0);
    Status[end_loc]++;
}

int main(int argc, char **argv)
{
    
	int inputdone = 0;
	int Loopcnt = 0;
        ros::init(argc, argv, "lab2node");
        ur_msgs::SetIO srv;
        ros::NodeHandle nh;
        ros::ServiceClient srv_SetIO = nh.serviceClient<ur_msgs::SetIO>("ur_driver/set_io");
    int startLocation = -1,endLocation = -1, midLocation;
//initialization & variable definition

	
	//initialized publisher ur3/command, buffer size of 10.
	ros::Publisher pub_command=nh.advertise<ece470_ur3_driver::command>("ur3/command",10);
	// initialize subscriber to ur3/position and call function position_callback each time data is published
	ros::Subscriber sub_position=nh.subscribe("ur3/position",1,position_callback);
	
	ece470_ur3_driver::command driver_msg;

	std::string inputString;
	while (!inputdone) {
		std::cout << "Enter Number of Loops <Either 1 2 or 3>";
		std::getline(std::cin, inputString);
		std::cout << "You entered " << inputString << "\n";
		if (inputString == "1") {
			inputdone = 1;
			Loopcnt = 1;
		} else if (inputString == "2") {
			inputdone = 1;
			Loopcnt = 2;
		} else if (inputString == "3") {
			inputdone = 1;
			Loopcnt = 3;
		} else {
			std:cout << "Please just enter the character 1 2 or 3\n\n";
		}
	}
    inputdone = 0;
    std::string modeString;
    	while (inputdone == 0 || inputdone == 1) {
		std::cout << "Enter the Start Location A B C:";
		std::getline(std::cin, modeString);
		std::cout << "You entered " << modeString << "\n";
		if (modeString == "A") {
            startLocation = 0;  
			inputdone += 1;
		} else if (modeString == "B") {
            startLocation = 1;
			inputdone += 1;
		} else if (modeString == "C") {
            startLocation = 2; 
			inputdone += 1;
		} else {
			std::cout << "Please just enter the character A B or C\n\n";
            continue;    		
		}
        Status[startLocation] = 3;
 FIRST:
        modeString == "";
        std::cout << "Enter the End Location A B C:";
        std::getline(std::cin, modeString);
        std::cout << "You entered " << modeString << "\n";
        if (modeString == "A") {
            endLocation = 0;  
			inputdone += 1;
		} else if (modeString == "B") {
            endLocation = 1;
			inputdone += 1;
		} else if (modeString == "C") {
            endLocation = 2; 
			inputdone += 1;
		} else {
			std::cout << "Please just enter the character A B or C\n\n";
            goto FIRST;    		
		}       
	}

    //determine the midLocation
    if(startLocation == 0){
       midLocation = 3 - endLocation;
    }
    else if(startLocation == 1){
       midLocation = 2 - endLocation;
    }
    else{
       midLocation = 1 - endLocation;
    }
    
    std::cout<<"StartLocation="<<startLocation<<std::endl;
    std::cout<<"midLocation="<<midLocation<<std::endl;
    std::cout<<"endLocation="<<endLocation<<std::endl;

	while(!ros::ok()){};	//check if ros is ready for operation
		
	ROS_INFO("sending Goals");

	ros::Rate loop_rate(SPIN_RATE); // Initialize the rate to publish to ur3/command
	int spincount = 0;
 
	while(Loopcnt > 0) {
		driver_msg.destination=QH;  // Set desired position to move home 
		pub_command.publish(driver_msg);  // publish command, but note that is possible that
												  // the subscriber will not receive this message.
		spincount = 0;
		while (isReady) { // Waiting for isReady to be false meaning that the driver has the new command
			ros::spinOnce();  // Allow other ROS functionallity to run
			loop_rate.sleep(); // Sleep and wake up at 1/20 second (1/SPIN_RATE) interval
			if (spincount > SPIN_RATE) {  // if isReady does not get set within 1 second re-publish
				pub_command.publish(driver_msg);
				ROS_INFO("Just Published again driver_msg");
				spincount = 0;
			}
			spincount++;  // keep track of loop count
		}

		ROS_INFO("waiting for rdy");  // Now wait for robot arm to reach the commanded waypoint.
		while(!isReady)
		{
			ros::spinOnce();
			loop_rate.sleep();
		}

        ROS_INFO("Step 1");
        move_block(pub_command, loop_rate, srv_SetIO, srv, startLocation, endLocation);  
        ROS_INFO("Step 2");
        move_block(pub_command, loop_rate, srv_SetIO, srv, startLocation, midLocation);
        ROS_INFO("Step 3");
        move_block(pub_command, loop_rate, srv_SetIO, srv, endLocation, midLocation);  
        ROS_INFO("Step 4");
        move_block(pub_command, loop_rate, srv_SetIO, srv, startLocation, endLocation);
        ROS_INFO("Step 5");
        move_block(pub_command, loop_rate, srv_SetIO, srv, midLocation, startLocation);
        ROS_INFO("Step 6");
        move_block(pub_command, loop_rate, srv_SetIO, srv, midLocation, endLocation);  
        ROS_INFO("Step 7");
        move_block(pub_command, loop_rate, srv_SetIO, srv, startLocation, endLocation);
		ROS_INFO("Moving back to initial position");
        driver_msg.destination=Q[0][0];
		pub_command.publish(driver_msg);  // publish command, but note that is possible that
												  // the subscriber will not receive this message.
		spincount = 0;
		while (isReady) { // Waiting for isReady to be false meaning that the driver has the new command
			ros::spinOnce();  // Allow other ROS functionallity to run
			loop_rate.sleep(); // Sleep and wake up at 1/20 second (1/SPIN_RATE) interval
			if (spincount > SPIN_RATE) {  // if isReady does not get set within 1 second re-publish
				pub_command.publish(driver_msg);
				ROS_INFO("Just Published again driver_msg");
				spincount = 0;
			}
			spincount++;  // keep track of loop count
		}

        //ROS_INFO("sending Goals 1");
          //      move_block(pub_command, loop_rate,srv_SetIO, srv, 0, 2);
                //move_arm(pub_command, loop_rate, Q[0][0],1.0);
                /*
		driver_msg.destination=Q[0][0];
		pub_command.publish(driver_msg);  // publish command, but note that is possible that
												  // the subscriber will not receive this message.
		spincount = 0;
		while (isReady) { // Waiting for isReady to be false meaning that the driver has the new command
			ros::spinOnce();  // Allow other ROS functionallity to run
			loop_rate.sleep(); // Sleep and wake up at 1/20 second (1/SPIN_RATE) interval
			if (spincount > SPIN_RATE) {  // if isReady does not get set within 1 second re-publish
				pub_command.publish(driver_msg);
				ROS_INFO("Just Published again driver_msg");
				spincount = 0;
			}
			spincount++;  // keep track of loop count
		}

		ROS_INFO("waiting for rdy");  // Now wait for robot arm to reach the commanded waypoint.
		while(!isReady)
		{
			ros::spinOnce();
			loop_rate.sleep();
		}
                */
        /*
		srv.request.fun = 1;
		srv.request.pin = 0;  //Digital Output 0
		srv.request.state = 1.0; //Set DO0 on
		if (srv_SetIO.call(srv)) {
			ROS_INFO("True: Switched Suction ON");
		} else {
			ROS_INFO("False");
		}

		ROS_INFO("sending Goals 2");
		driver_msg.destination=Q[0][1];
		driver_msg.duration=2.0;
		pub_command.publish(driver_msg);  // publish command, but note that is possible that
												  // the subscriber will not receive this message.
		spincount = 0;
		while (isReady) { // Waiting for isReady to be false meaning that the driver has the new command
			ros::spinOnce();  // Allow other ROS functionallity to run
			loop_rate.sleep(); // Sleep and wake up at 1/20 second (1/SPIN_RATE) interval
			if (spincount > SPIN_RATE) {  // if isReady does not get set within 1 second re-publish
				pub_command.publish(driver_msg);
				ROS_INFO("Just Published again driver_msg");
				spincount = 0;
			}
			spincount++;  // keep track of loop count
		}

		ROS_INFO("waiting for rdy");  // Now wait for robot arm to reach the commanded waypoint.
		while(!isReady)
		{
			ros::spinOnce();
			loop_rate.sleep();
		}

		srv.request.fun = 1;
		srv.request.pin = 0; // Digital Output 0
		srv.request.state = 0.0; //Set DO0 off
		if (srv_SetIO.call(srv)) {
			ROS_INFO("True: Switched Suction OFF");
		} else {
			ROS_INFO("False");
		}

		ROS_INFO("sending Goals 3");
		driver_msg.destination=Q[0][2];
		driver_msg.duration=1.0;
		pub_command.publish(driver_msg);  // publish command, but note that is possible that
												  // the subscriber will not receive this message.
		spincount = 0;
		while (isReady) { // Waiting for isReady to be false meaning that the driver has the new command
			ros::spinOnce();  // Allow other ROS functionallity to run
			loop_rate.sleep(); // Sleep and wake up at 1/20 second (1/SPIN_RATE) interval
			if (spincount > SPIN_RATE) {  // if isReady does not get set within 1 second re-publish
				pub_command.publish(driver_msg);
				ROS_INFO("Just Published again driver_msg");
				spincount = 0;
			}
			spincount++;  // keep track of loop count
		}

		ROS_INFO("waiting for rdy");  // Now wait for robot arm to reach the commanded waypoint.
		while(!isReady)
		{
			ros::spinOnce();
			loop_rate.sleep();
		}
        */ 
		Loopcnt--;
	}

	return 0;
}
