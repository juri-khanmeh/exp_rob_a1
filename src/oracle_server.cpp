/**
* \file oracle_server.cpp
* \brief This file defines the oracle service
* \author Juri Khanmeh
* \version 0.1
* \date 20/11/2021
*
* \details
*
* Services : <BR>
* ° /oracle
*
* Description :
*
* This node is an oracle server. 
* The client ask to check whether the given ID is the correct ID or not
*
*/
#include "ros/ros.h"
#include "exp_rob_a1/Check_id.h"
#include <math.h>

int ID_solution = 4; ///< the correct hypothesis ID

/**
* \brief '/oracle' server callback
* \param &req an ID
* \param &res True/False
* \return always true as this method cannot fail.
*
* This funtion verifies if the ID is correct or not
*/
bool oracle_callback (exp_rob_a1::Check_id::Request &req, 
				exp_rob_a1::Check_id::Response &res){
   if (req.id == ID_solution)
   	res.result = true;
   else
   	res.result = false;
   	
   return true;
}

/**
* \brief main function
* \param argc 
* \param argv
* \return always true.
*
* The main function initializes the node and creates a '/oracle' service server.
*/
int main(int argc, char **argv)
{
   ros::init(argc, argv, "oracle_server");
   ros::NodeHandle n;
   ros::ServiceServer service= n.advertiseService("/oracle", oracle_callback);
   ros::spin();

   return 0;
}
