/**
* \file EngConverter_server.cpp
* \brief This file defines the english phrase converter service
* \author Juri Khanmeh
* \version 0.1
* \date 20/11/2021
*
* \details
*
* Services : <BR>
* ° /eng_sentence
*
* Description :
*
* This node is a english converter server. 
* It responds with an english expression correspoding to a given hypothesis.
*
*/
#include "ros/ros.h"
#include "exp_rob_a1/Sentence.h"
#include "std_msgs/String.h"

/**
* \brief '/target' server callback
* \param &req hypothesis (3 hints)
* \param &res string
* \return an english expression of the hypothesis
*
* This function creates an englsih phrase to express the hypothesis
*/
bool sentence_callback (exp_rob_a1::Sentence::Request &req, 
			exp_rob_a1::Sentence::Response &res){			
   std_msgs::String who; std_msgs::String where; std_msgs::String what;
   
   if(req.type1=="who")
   	who.data = req.info1;
   else if(req.type1=="where")
   	where.data = req.info1;
   else if(req.type1=="what")
   	what.data = req.info1;
   
   if(req.type2=="who")
   	who.data = req.info2;
   else if(req.type2=="where")
   	where.data = req.info2;
   else if(req.type2=="what")
   	what.data = req.info2;
   	
   if(req.type3=="who")
   	who.data = req.info3;
   else if(req.type3=="where")
   	where.data = req.info3;
   else if(req.type3=="what")
   	what.data = req.info3;
   	
   res.phrase = who.data + " with the " + what.data + " in the " + where.data;
   	
   return true;
}

/**
* \brief main function
* \param argc 
* \param argv
* \return always true.
*
* The main function initializes the node and creates a '/eng_sentence' service server.
*/
int main(int argc, char **argv)
{
   ros::init(argc, argv, "converter_server");
   ros::NodeHandle n;
   ros::ServiceServer service= n.advertiseService("/eng_sentence", sentence_callback);
   ros::spin();

   return 0;
}
