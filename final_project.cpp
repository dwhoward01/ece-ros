// This program steers a turtlesim turtle1 toward the xy locatins entered in the command line.
// Daniel Howard
#include <ros/ros.h>
#include <geometry_msgs/Twist.h> // For geometry_msgs::Twist
#include <turtlesim/Pose.h>
#include <iomanip> // for std: setprecision and std::fixed
#include <stdlib.h> // For rand() and RAND_MAX
#include <math.h>       /* sqrt */
#include <std_srvs/Empty.h> // for clearing the screen

turtlesim::Pose turtlePose;  // global variable for the turtle's pose.  
int poseInitialized = 0;
// control variables
float angErr,angGoal,distErr,homeDistErr,locDistErr;
// goal location
float goalx, goaly;
float displacement;
float cornerAngleErr;
float cornerAngGoal;

// A callback function. Executed each time a new pose message arrives.
void poseMessageReceived(const turtlesim::Pose& msg) {
    // TODO 2b:  copy the msg pose to your global variable
    turtlePose = msg;
    poseInitialized = 1;
    //ROS_INFO_STREAM("poseMessageReceived");
}

int main(int argc, char ** argv) {
    // Initialize the ROS system and become a node.
    ros::init(argc, argv, "inclass_steer_turtle_to_xy");
    ros::NodeHandle nh;
    
    // clear the screen
    ros::ServiceClient clearClient = nh.serviceClient<std_srvs::Empty>("/clear");
    std_srvs::Empty srv;
    clearClient.call(srv); 
    // Create a publisher object
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1000);
    // Create a subscriber object
    ros::Subscriber sub = nh.subscribe("turtle1/pose", 1000, &poseMessageReceived);
    // get the goal x and y locations
    if (argc == 3){
        goalx = atof(argv[1]);
        // TODO 2a: save the y location from the command input
        goaly = atof(argv[2]);
        ROS_INFO_STREAM(std::setprecision(2) << std::fixed << "goal position=(" << goalx << "," <<goaly << ")");
    }else{
        ROS_INFO_STREAM(std::setprecision(2) << std::fixed << "You need to supply an x and y output");
        return -1;
    }
    
    // Loop at 10 Hz until the node is shut down.
    ros::Rate rate(10);
    //message to be sent to turtle.  The other four fields, which are ignored by turtlesim, default to zero.
    geometry_msgs::Twist msg;	

    float homePoseX = 5.54; //variables needed to track positions and mower pattern
    float homePoseY = 5.54;
    float currentPoseX = homePoseX;
    float currentPoseY = homePoseY;
    float saveAngle = 0;
    float poseTheta = 0;
    float angRad = 1.57;
    cornerAngGoal = poseTheta + angRad;
    displacement = 0;
    int sideCount = 0;
    int cornerCount = 0;
    float length = .5;
    int objectFound = 0;
    int returnHome = 0; 
    int returnToLocation = 0;
    int resumeSearch = 0;

    while(ros::ok()) {
        ros::spinOnce(); // check callbacks

        if(!returnHome) // if we haven't found the goal object yet, go forward and track displacement
        {
        	msg.linear.x = 1.0;
    		msg.angular.z = 0;
    		pub.publish(msg);
    		displacement = sqrt(pow(currentPoseX-turtlePose.x,2)+pow(currentPoseY-turtlePose.y,2));
    		//ROS_INFO_STREAM("displacement:" << displacement);
        }

    	if (displacement >= length && !returnHome) // need to turn in place if traveled target length
    	{
    		msg.linear.x = 0;
    		msg.angular.z = 0.5;
    		pub.publish(msg);
    		cornerAngleErr = fabsf(turtlePose.theta - cornerAngGoal);
    		//ROS_INFO_STREAM("turning, cornerAngleErr" << ": " << cornerAngleErr);
    		if(cornerAngleErr < 0.1) // if we are done turning reset displacement, update currentPose, cornerAngGoal
    		{
    			displacement = 0;
    			currentPoseX = turtlePose.x;
    			currentPoseY = turtlePose.y;
    			cornerAngGoal += angRad;
    			cornerCount++; // increment count to keep track of corners
    			sideCount++;
    			//ROS_INFO_STREAM("finished turning");
    			if(sideCount == 2)
	    		{
	    			//ROS_INFO_STREAM("made two sides");
	    			sideCount = 0; // reset sideCount
	    			length += 0.2; // incrememnt side length
	    		}
    			if(cornerCount == 4)
	    		{
	    			cornerCount = 0;
	    			cornerAngGoal = angRad;
	    		}
    		}
    	}        
        if (objectFound == 0) // if we haven't found the object yet go ahead and get distance
        	distErr = sqrt(pow(goalx-turtlePose.x,2)+pow(goaly-turtlePose.y,2)); // calculate the distance to goal object
        else distErr = 100; // if we have found the object, set the distance error to something large so that we don't "find" it again
        if (distErr < 0.2) //made it to goal object
        {
        	ROS_INFO_STREAM("found the object");
        	objectFound++; //tracking objects found
        	returnHome = 1; //flag to return home set
        	msg.linear.x = 0; // stop moving 
        	saveAngle = turtlePose.theta; //save the orientation of turtle for later
        }
        
        if(returnHome) // return if at the goal.
        { 
        	ROS_INFO_STREAM("returning home");
            //compute control parameters to get home
	        homeDistErr = sqrt(pow(homePoseX-turtlePose.x,2)+pow(homePoseY-turtlePose.y,2)); //calculate the distance to home location
	        angGoal = atan2((homePoseY-turtlePose.y), (homePoseX-turtlePose.x)); // atan2 to calculate the (relative) angular error
	        if (angGoal < 0)
	        	angGoal+=6.28;
	        angErr = fabsf(turtlePose.theta - angGoal);
	        ROS_INFO_STREAM("homeDistErr: " << homeDistErr << " " << "angGoal: " << angGoal << " " << "angErr: " << angErr);
	        
	        // control law
	        if( fabsf(angErr) > 1.0){
	            msg.linear.x = 0; // if angular error is large, turn in place
	        }else{
	            msg.linear.x = fmin(homeDistErr,1.0);
	        }
	        msg.angular.z = 0.5*angErr;
	        pub.publish(msg); // Publish the message.
	        
	        if(homeDistErr < 0.2) //made it back home
	        {
	        	//returnHome = 0; // turn off returnHome flag
	        	returnToLocation = 1; // turn on returnToLocation flag
	        }
	    }   

	    if(returnToLocation) // return to where we found the object after returning it.
        { 
        	ROS_INFO_STREAM("returning to location");
            //compute control parameters to get back to the location
	        locDistErr = sqrt(pow(goalx-turtlePose.x,2)+pow(goaly-turtlePose.y,2)); //calculate the distance to home location
	        angGoal = atan2((goaly-turtlePose.y), (goalx-turtlePose.x)); // atan2 to calculate the (relative) angular error
	        angErr = fabsf(turtlePose.theta - angGoal);
	        if (angGoal < 0)
	        	angGoal+=6.28; //add 2pi to angErr if it comes out negative.
	        ROS_INFO_STREAM("locDistErr: " << locDistErr << " " << "angGoal: " << angGoal << " " << "angErr: " << angErr);
	        
	        // control law
	        if( fabsf(angErr) > 1.0){
	            msg.linear.x = 0.2; // if angular error is large, turn in place
	        }else{
	            msg.linear.x = fmin(locDistErr,1.0);
	        }
	        msg.angular.z = 0.5;
	        pub.publish(msg);
	        
	        if(locDistErr < 0.2)
	        {
	        	// now we are back where we found the first object
	        	msg.linear.x = 0; // stop moving
	        	msg.angular.z = 0; //stop turning
	        	pub.publish(msg);
	        	returnToLocation = 0; //turn off returnToLocation flag
	        	angGoal = saveAngle;
	        	ROS_INFO_STREAM("angGoal" << " " << saveAngle);
	        	angErr = fabsf(turtlePose.theta - angGoal);
	        	while(fabsf(angErr) > 1.0)
	        	{
	        		msg.linear.x = 0;
	        		msg.angular.z = 0.5;
	        		pub.publish(msg);
	        		angErr = fabsf(turtlePose.theta - angGoal);
	        	}
	        	returnHome = 0;
	        	
	        }
	    } 
        // Wait until it's time for another interaction
        rate.sleep();// 
    }
}