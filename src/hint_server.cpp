/**
* \file hint_server.cpp
* \brief This file defines the hint service
* \author Juri Khanmeh
* \version 0.1
* \date 20/11/2021
*
* \details
*
* Services : <BR>
* ° /hint
*
* Description :
*
* This node is a hint transmitter.
* It responds with a hint, which consists of an id, type and information.
* The hint list is pre-defined in the matrix 'Hints'.
*
*/
#include "ros/ros.h"
#include "exp_rob_a1/Hint.h"
#include <math.h>
#include "std_msgs/String.h"


struct hint{
	int id;
	const char* type;
	const char* info;
};

struct hint Hints[15] = {{1,"where","room"},{1,"who","Jim"},{1,"who","Pole"},
	{2,"where","bathroom"},{2,"where","livingroom"},{2,"who","Sam"},
	{3,"where","hall"},{3,"who","Scarlett"},{3,"what","candlestick"},
	{4,"where","ballroom"},{4,"who","Plum"},{4,"what","rope"},
	{0,"where","library"},{0,"who","Peacock"},{0,"what","spanner"},};


int sent[15] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}; ///< matix of flags in order not to send the same hint more than once

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
* \brief '/hint' server callback
* \param &req null
* \param &res hint
* \return always true as this method cannot fail.
*
* This function send a random hint from the hint list
* The hint consists of an id, type and info.
*/
bool hint_callback (exp_rob_a1::Hint::Request &req, 
			exp_rob_a1::Hint::Response &res){
   int i = randNum(15);
   while(sent[i]==1) {i = randNum(15);}
   res.id = Hints[i].id;
   res.type = Hints[i].type;
   res.info = Hints[i].info;
   sent[i] = 1;
   return true;
}

/**
* \brief main function
* \param argc 
* \param argv
* \return always true.
*
* The main function initializes the node and creates a '/hint' service server.
*/
int main(int argc, char **argv)
{
   ros::init(argc, argv, "hint_server");
   ros::NodeHandle n;

   ros::ServiceServer service= n.advertiseService("/hint", hint_callback);
   ros::spin();

   return 0;
}
