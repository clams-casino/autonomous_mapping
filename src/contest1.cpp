#include <ros/console.h>
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/BumperEvent.h>
#include <sensor_msgs/LaserScan.h>
#include <eStop.h>

#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

// to get info from map
#include <nav_msgs/MapMetaData.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/Header.h>



#include <stdio.h>
#include <cmath>

#include <ctime>

#include <string>

using namespace std;


const double pi = 3.14159;




double angular;
double linear;


const double max_lin_speed = 0.25;
const double max_ang_speed = pi/5;



class positionVector
{
public:
	double x;
	double y;
	double yaw;

	void odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
		x = msg->pose.pose.position.x;
		y = msg->pose.pose.position.y;
		yaw = tf::getYaw(msg->pose.pose.orientation);
	}
};



class Timer
{
    private:
    clock_t startTime;

    public:
    Timer ()
    {
        startTime = std::clock();
    }

    double getElapsedTime()
    {
        return (double) (std::clock() - startTime) / CLOCKS_PER_SEC;
    }
};





// used by planner to issues commands into the main while loop
class command{
	public:
		double angularSpeed;
		double linearSpeed;
		double duration;
		double start_time;
		bool turn_avoid; //true if the last command was a turn to get away from a wall
		int type; //basically a flag for if the request command is critical for avoidance
	


	command(double ang, double lin, double dur, bool avoid, int command_type){
		this->duration = dur;
		this->angularSpeed = ang;
		this->linearSpeed = lin;
		this->start_time = ros::Time::now().toSec();
		this->turn_avoid = avoid;
		this->type = command_type;
	}
	
};



class bumperStates{
	// 0 is left      1 is center     2 is right
	public:
		bool left;
		bool center;
		bool right;

	void bumperCallback(const kobuki_msgs::BumperEvent msg){
		if(msg.bumper == 0){
			this->left = !this->left;		
		}
		else if(msg.bumper == 1){
			this->center = !this->center;
		}
		else if(msg.bumper == 2){
			this->right = !this->right;
		}
	}

	bool isBumped(){
		return left || center || right;
	}
};







// randomly pick either 1 or -1
int randomSign(){
	return 2*(rand()%2)-1;
}







const int laserSize = 639;

const int angleSectors = 5; // try to divide cone into 3 approximately equal sized angle sectors
double sectorRange[angleSectors];
int sectorIndicies[angleSectors];
int del_sector = laserSize/angleSectors;
// Increasing indicies means going from right to left
// if looking at it from a top down view

double sectorWeights[angleSectors];




void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
	//fill with your 


	// return the average laser range in each sector 
	for(int i=0; i<angleSectors; i++){
		double sum = 0;
		for(int j=sectorIndicies[i]; j<sectorIndicies[i]+del_sector; j++){

			if(!isnan(msg->ranges[j])){
				sum = sum + msg->ranges[j];
			}
		}
		sectorRange[i] = sum/del_sector;
	}
	
	
}




void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg){

}







// simple plan based on sector ranges
double turnState[angleSectors];

// below wallThreshold is critical, move away
// beyond exploreThreshold means that that side is pretty open, take on an exploratory behaviour

command avoidancePlanner(double wallThreshold, command prevCommand, bumperStates bump){


	command plannedCommand = command(0,0,0,false,0);




	// stats about the sectors
	double max_range = 0; //can see how far one way is one next obstacle
	double min_range = sectorRange[1]; //can use to see if atleast one section is at a wall
	double avgRange = 0;  //average range value

	double turnNumber = 0;


	for(int i=0; i<angleSectors; i++){
		if(sectorRange[i]<wallThreshold){

			//set to negative number if near a wall
			turnState[i] = -10; 
          
		}
		else{
			turnState[i] = sectorRange[i];
		}

		if(sectorRange[i] < min_range){ min_range = sectorRange[i]; }
		if(sectorRange[i] > max_range){ max_range = sectorRange[i]; }

		avgRange = avgRange + sectorRange[i];
	}

	avgRange = avgRange / angleSectors;
 

	for(int i=0; i<angleSectors; i++){ turnNumber = turnNumber + turnState[i]*sectorWeights[i]; }
	turnNumber = -1*turnNumber / angleSectors; //switch sign of turnNumber around
	if(turnNumber > 5){ turnNumber = 5; }
	if(turnNumber < -5){ turnNumber = -5; }



/*------------------------------------------- Check conditions -------------------------------------------- */


	//check bumper first since its the most critical
	if(bump.isBumped()){

		//turn_avoid must be false to prevent
		//just running back into the bumper
		//back up at max speed for 0.35 seconds
		plannedCommand = command(0, -max_lin_speed, 0.25/max_lin_speed, false, 3);
		//printf("got bumped, backing up \n");
		goto endPlanning;
	}


	//if the last command was to backup due to bumper
	//turn a bit in a random direction
	if(prevCommand.type == 3){
		plannedCommand = command(randomSign() * max_ang_speed, 0, (pi/4)/max_ang_speed, false, 0);
		goto endPlanning;
	}


	// if last command was a turn to turn away from a wall
	if(prevCommand.turn_avoid == true){

		//go straight for a bit after turning away from the wall
		// cannot be overwritten by laser avoidance command
		// to avoid condition of getting stuck turning in a corner
		return command(0,0.2,0.1,false,2);
		goto endPlanning;

		//printf("going straight after turning from wall\n");
	}



	// this is true if the full 60 degree cone is within a wall - F
	// since this command has override, only do it if the previous command
	// was not a turn avoid command
	if(max_range < wallThreshold &&  prevCommand.turn_avoid == false){

		//if left slightly more range than right, turn left90
		if(sectorRange[4]/sectorRange[0] > 1.2 ){
			plannedCommand = command(max_ang_speed, 0, (2*pi/3)/max_ang_speed, true, 2);
			goto endPlanning;
		}
		//if right slightly more range than left, turn right 90
		else if(sectorRange[4]/sectorRange[0] < 1/1.2){
			plannedCommand = command(-max_ang_speed, 0, (2*pi/3)/max_ang_speed, true, 2);
			goto endPlanning;
		}
		//if mostly straight on the wall, then turn in a random direction
		else{
			plannedCommand = command(randomSign()*max_ang_speed, 0, (2*pi/3)/max_ang_speed, true, 2);
			goto endPlanning;
		}

	} 



	//if right corner is near wall, but left corner not near wall
	if(sectorRange[0] < wallThreshold && sectorRange[4] > wallThreshold  &&  prevCommand.turn_avoid == false){
		
		if(sectorRange[1] < wallThreshold){
			// if right also near wall, then left turn by more, say ~45 degrees
			plannedCommand = command(max_ang_speed, 0, (pi/4)/max_ang_speed, true, 2);
			goto endPlanning;
		}
		else{
			// if only right corner near wall, then just turn left by less, ~ 22 degrees
			plannedCommand = command(max_ang_speed, 0 ,(pi/8)/max_ang_speed, true, 2);
			goto endPlanning;
		}
	}

	//if left corner is near wall, but right corner not near wall
	if(sectorRange[4] < wallThreshold && sectorRange[0] > wallThreshold  &&  prevCommand.turn_avoid == false){
		
		if(sectorRange[3] < wallThreshold){
			// if left also near wall, then right turn by more, say ~45 degrees
			plannedCommand = command(-max_ang_speed, 0, (pi/4)/max_ang_speed, true, 2);
			goto endPlanning;			
		}
		else{
			// if only right corner near wall, then just turn right by less, ~ 22 degrees
			plannedCommand = command(-max_ang_speed, 0 ,(pi/8)/max_ang_speed, true, 2);
			goto endPlanning;
		}
	}


	//------------------------------------------------------------------------------------------------------------//

	//relative measure of which side has more range
	//more positive -> right side comparatively more range than left
	//more negative -> left side comparatively more range than right
	// ~0 -> both sides about the same





	//explore via turnNumber
	plannedCommand = command((turnNumber/5)*1*max_ang_speed, max_lin_speed, 0, false, 0);


	endPlanning:


	//plannedCommand =  command(randomSign()*max_ang_speed, 0, (pi/4)/max_ang_speed, true, false);

	return plannedCommand;
}



// current command MUST be passed by REFERENCE

void highLevelPlanner(command newAvoidanceCommand, command& currentCommand, double timeNow, double timeBetweenSpins, double& timeLastSpin){

		
	// speed of 360
	double spinSpeed = 1*max_ang_speed;



	// if the avoidance says we're in danger, do what we always do
	// but if we're already doing a 360 then just go ahead, unless we get a bumper avoidance
	if(newAvoidanceCommand.type != 0 && currentCommand.type != 1){
		if(timeNow - currentCommand.start_time > currentCommand.duration || newAvoidanceCommand.type > currentCommand.type){

			currentCommand = newAvoidanceCommand;
			printf("Doing new avoidance command - start time: %f - turn avoid? %i \n",currentCommand.start_time,currentCommand.turn_avoid);
			printf("angular: %f   linear: %f \n",currentCommand.angularSpeed, currentCommand.linearSpeed);
			printf("command type: %i \n\n ", currentCommand.type);

			goto endHighLevelPlanning;

		}
	}

	// should override spin if we hit a bumper
	else if(newAvoidanceCommand.type == 3 && currentCommand.type == 1){
		currentCommand = newAvoidanceCommand;
		printf("Hit bumper during spin, performing bumper avoidance manuever \n\n");

		goto endHighLevelPlanning;
	}
    


	// since we're past the avoidance conditions check the explore conditions



	// is it about time for us to spin again?
    if(timeNow - timeLastSpin > timeBetweenSpins){
    	currentCommand = command(spinSpeed, 0 , 2*pi/spinSpeed, false, 1);
    	printf("time to do a do a xbox 360 \n\n");

    	timeLastSpin = ros::Time::now().toSec() + 2*pi/spinSpeed;

    	goto endHighLevelPlanning;
    }


    // if we don't have to do a spin what to do?

    if(timeNow - currentCommand.start_time > currentCommand.duration){
    	currentCommand = newAvoidanceCommand;
    	printf("Doing new exploration command - start time: %f - turn avoid? %i \n",currentCommand.start_time,currentCommand.turn_avoid);
		printf("angular: %f   linear: %f \n",currentCommand.angularSpeed, currentCommand.linearSpeed);
		printf("command type: %i \n\n ", currentCommand.type);   
	}



    endHighLevelPlanning:
    ; //null statement to close the function after the label

}










int main(int argc, char **argv)
{
	ros::init(argc, argv, "image_listener");
	ros::NodeHandle nh;
	teleController eStop;


	// laser subscriber
	ros::Subscriber laser_sub = nh.subscribe("scan", 10, &laserCallback);


	// bumper subscriber
	bumperStates bumper;
	bumper.left = false; bumper.center = false; bumper.right = false;
	ros::Subscriber bumper_sub = nh.subscribe("mobile_base/events/bumper", 10, &bumperStates::bumperCallback, &bumper);


	// odom position subscriber
	positionVector currPosition;
	ros::Subscriber odom = nh.subscribe("/odom", 1, &positionVector::odomCallback, &currPosition);



	// map occupany grid subscriber
	ros::Subscriber occupany_map = nh.subscribe("map", 1, &mapCallback);





	ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);
	angular = 0.0;
	linear = 0.0;
	geometry_msgs::Twist vel;



	Timer timer = Timer();


	// for getting indices of scan sectors
	for(int s=0; s < angleSectors; s++){
		sectorIndicies[s] = del_sector * s;
	}

	// setting up sectorWeights
	for(int s=0; s < angleSectors; s++){
		sectorWeights[s] = (angleSectors - s) - ceil((double)angleSectors/2);
	}







	command currCommand = command(0,0,0,false,0);

	//time stamps for 360
	double betweenSpin = 10;
	double lastSpin = 0;


	while(ros::ok() && timer.getElapsedTime() < 480){

		ros::spinOnce();
		//.....**E-STOP DO NOT TOUCH**.......
		eStop.block();
		//...................................
		//fill with your code




		// threshold parameters for planner
		double wallThresh = 0.4;


		command avoidanceCommand = avoidancePlanner(wallThresh, currCommand, bumper);

		highLevelPlanner(avoidanceCommand, currCommand, ros::Time::now().toSec(), betweenSpin, lastSpin);

	


  		vel.angular.z = currCommand.angularSpeed;
  		vel.linear.x = currCommand.linearSpeed;

  		vel_pub.publish(vel);


  		//ROS_INFO("X:%f Y:%f Yaw:%f", currPosition.x, currPosition.y,currPosition.yaw);		
  		//ROS_INFO("LL:%f   L: %f   C:%f   R:%f  RR:%f   TN:%f",sectorRange[4],sectorRange[3], sectorRange[2],sectorRange[1],sectorRange[0],tempTurnNumber);
	
  		//ROS_INFO("left: %i   center: %i   right: %i   isbumped: %i", bumper.left,bumper.center,bumper.right,bumper.isBumped());

	} 


	printf("its done bruv \n");



	//automatically save the map

	time_t now = time(0);
	tm *ltm = localtime(&now);

	string year = to_string(1900 + ltm->tm_year);
    string month = to_string(1+ ltm->tm_mon);
	string day = to_string(ltm->tm_mday);
	string hour = to_string(ltm->tm_hour);
	string minute = to_string(ltm->tm_min);


	string directory_name = "~/ros_maps/" + year + "_" + month + "_" + day + "_" + hour + "_" + minute;
	string make_folder = "mkdir " + directory_name;
	system(make_folder.c_str());
	string save_map = "rosrun map_server map_saver -f " + directory_name + "/map";
	system(save_map.c_str());






	return 0;
}









//to do

// 360 turn on time more robust



// bumper callback
// just get bumper state

/*can get the bumper state in an object, also has a function to check if any bumper 
are currently bumped, should pass bumper into avoidance planner*/




//check for turning left then turning right if in corner




/*----------- change log --------------- */

/*changed turning for big turn to 120 degrees, to help turn out of corners
*/













//stuff here



/*float obstacleVisionCone(vector<float> scanArray, int laserSize, double obstacleThreshold){


	float sum = 0;
	for(int i = 0; i < laserSize; i++){

		if(isnan(scanArray[i])){
			printf("nan at index %i \n",i);
		}
		else{
			sum = sum + scanArray[i];
			printf("%f \n",sum);
			printf("%f \n",scanArray[i]);
		}
	}

	return sum/laserranges;


	int * obstacleArray;

}*/
