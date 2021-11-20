/**
* \file randTarget_server.cpp
* \brief This file defines the position service
* \author Juri Khanmeh
* \version 0.1
* \date 20/11/2021
*
* \details
*
* Services : <BR>
* ° /target
*
* Description :
*
* This node is a random room position server. 
* It responds with a random room position (x,y), which is used as a target to be reached.
* The random room  position is chosen from pre-definde in 'room' matrix.
*
*/
#include "ros/ros.h"
#include "exp_rob_a1/Rand_Target.h"
#include <math.h>

int room[6][2] = {{3,4},{-3,4},{4,0},{-4,0},{3,-4},{-3,-4}};

/**
* \brief random number generator
* \param &req the upper bound for number generator
* \return a random number
*
* This function generate a random integer number 
* between 0 and n
*/
int randNum(int n){
	return rand()%n;
}

/**
* \brief '/target' server callback
* \param &req null
* \param &res defines the position [x,y] of a room
* \return always true as this method cannot fail.
*
* This function creates a RandomPosition message. It fills up the response message
* with random value for x, y for a random room
*/
bool randTarget (exp_rob_a1::Rand_Target::Request &req, 
			exp_rob_a1::Rand_Target::Response &res){
   int i = randNum(6);
   res.pos_x = room[i][0];
   res.pos_y = room[i][1];
   return true;
}

/**
* \brief main function
* \param argc 
* \param argv
* \return always true.
*
* The main function initializes the node and creates a '/target' service server.
*/
int main(int argc, char **argv)
{
   ros::init(argc, argv, "randTarget_server");
   ros::NodeHandle n;

   ros::ServiceServer service= n.advertiseService("/target", randTarget);
   ros::spin();

   return 0;
}
