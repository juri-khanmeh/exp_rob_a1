/**
* \file state_machine.cpp
* \brief state machine for exp_rob_a1 control
* \author Juri Khanmeh
* \version 0.1
* \date 20/11/2021
*
* \details
*
* Services : <BR>
* ° /go_point
* ° /target
* ° /hint
* ° /oracle
* ° /eng_sentence
*
* Description :
*
* This file defines the state machine of Cluedo game control. 
* The node receives a random room position through /target topic.
* Then the robot reaches that room.
* Once the robot reaches the room, the node receives a hint.
* It repeats the process until a consistent hypothesis is found.
* The game ends when the correct hypothesis is found.
*/

#include "ros/ros.h"
#include "exp_rob_a1/Hint.h"
#include "exp_rob_a1/Position.h"
#include "exp_rob_a1/Rand_Target.h"
#include "exp_rob_a1/Check_id.h"
#include "exp_rob_a1/Sentence.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Point.h"

struct hint{
	std_msgs::String type;
	std_msgs::String info;	
};

struct data_table{
	int counter; // counter of hints for same ID
	struct hint hints[3];
};

data_table* Data = new data_table[5]; // hint list for 5 IDs

exp_rob_a1::Rand_Target target; 
exp_rob_a1::Hint hint_s; 
exp_rob_a1::Position position_s;
exp_rob_a1::Check_id check;
exp_rob_a1::Sentence se;

geometry_msgs::Point T_vel; 

ros::ServiceClient client_room; 
ros::ServiceClient client_hint; 
ros::ServiceClient client_go;
ros::ServiceClient client_oracle;
ros::ServiceClient client_sentence;

/**
* \brief function for checking consistency
* \param int ID
* \return boolean
*
* This function checks the consistency of given hypothesis.
* It returns True if the hypothesis is consistent.
* Otherwise it returns False.
*/
bool check_consistence(int ID){
	bool who = false; bool what = false; bool where = false;
	for(int j=0; j<3; j++){
 		if(Data[ID].hints[j].type.data =="who"){
 			who = true; break;
 		}
 	}
 	if(who){
 		for(int j=0; j<3; j++){
 			if(Data[ID].hints[j].type.data =="what"){
 				what = true; break;
 			}
 		}
 		if(what){
 			for(int j=0; j<3; j++){
 				if(Data[ID].hints[j].type.data =="where"){
 					return true;
 				}	
 			}
 			return false;
 		}
 		else return false;
 	}
 	else return false; 
}

/**
* \brief main function
* \param argc 
* \param argv
* \return always true.
*
* The main function initializes the node, creates a '/go_point',
* '/target', '/hint', '/oracle', '/eng_sentence' service clients
* Then in the main while loop, it receives a random room position,
* go to that room and receives hints.
* Once the node recieves a complete hypothesis, it checks its consistency.
* If the correct hypothesis is found, the game is over.
*/
int main(int argc, char **argv)
{
   ros::init(argc, argv, "state_machine");
   ros::NodeHandle n;
   
   client_go = n.serviceClient<exp_rob_a1::Position>("/go_point");
   client_room = n.serviceClient<exp_rob_a1::Rand_Target>("/target");
   client_hint = n.serviceClient<exp_rob_a1::Hint>("/hint");
   client_oracle = n.serviceClient<exp_rob_a1::Check_id>("/oracle");
   client_sentence = n.serviceClient<exp_rob_a1::Sentence>("/eng_sentence");
     
   int start = true;
   
   while(start){
   	ros::spinOnce();
   	client_room.call(target);
   	position_s.request.pos_x = target.response.pos_x;
   	position_s.request.pos_y = target.response.pos_y;
   	ROS_INFO("Room position [%d, %d]", target.response.pos_x , target.response.pos_y); 
   	client_go.call(position_s);
   	ROS_INFO("Room is reached");
 	client_hint.call(hint_s);
 	int i = hint_s.response.id;
 	int k = Data[i].counter;
 	Data[i].hints[k].type.data = hint_s.response.type;
 	Data[i].hints[k].info.data = hint_s.response.info;
 	Data[i].counter = k + 1;
 	printf("Hint ID[%d] number[%d] ", i, k);
 	printf(Data[i].hints[k].type.data.c_str()); putchar(' ');
 	puts(Data[i].hints[k].info.data.c_str());
 	bool consistent;
 	if(k==2){
 		consistent = check_consistence(i); 
 		if(consistent){
 			printf("Hypothesis [%d] is Consistent",i);
 			std::cout << "\nLet's go to [0,0]";
 			position_s.request.pos_x =0; position_s.request.pos_y =0;
 			client_go.call(position_s);
 			std::cout << "\nHypothesis is: ";
 			se.request.type1 = Data[i].hints[0].type.data;
 			se.request.type2 = Data[i].hints[1].type.data;
 			se.request.type3 = Data[i].hints[2].type.data;
 			se.request.info1 = Data[i].hints[0].info.data;
 			se.request.info2 = Data[i].hints[1].info.data;
 			se.request.info3 = Data[i].hints[2].info.data;
 			client_sentence.call(se);
 			std::cout << "\n"<< se.response.phrase <<"\n";
 			check.request.id = i;
 			client_oracle.call(check);
 			if(check.response.result){
 				start = false;
 				printf("Hypothesis [%d] is CORRECT",i);
 				std::cout << "\nGame Over"<< "\n";
 			}
 			else {printf("Hypothesis [%d] is NOT correct",i);std::cout << "\n";}
 		}
 		else
 			{printf("Hypothesis [%d] is NOT consistent",i);std::cout << "\n";}
 	}
   }	
   	
   
   return 0;
}
