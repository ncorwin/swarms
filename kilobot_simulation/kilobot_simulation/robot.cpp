#include <iostream>
#include "robot.h"
#include <vector>

//vectors
std::vector<int> neighbor_id(10);
std::vector<int> neighbor_distance(10);
std::vector<int>::iterator it;


void robot::controller_move_straight()
{

	motor_command = 1;

	color[0] = 1;
	color[1] = 0;
	color[2] = 0;

	if (light < 700)
	  {
		color[0] = 0;
		color[1] = 1;
		color[2] = 0;
	  }
	
}
//test of following light
void robot::controller_light_follow()
{
    color[0] = 1;
	color[1] = 1;
	color[2] = 1;

	//light following
	if (light < 100)
	{
	    motor_command = 1;
	}
	else
	{
	    motor_command = 2;
	  //*
	    if (previous_light < light)
		{
		    if (motor_command == 2)
			{
			    motor_command = 3;
			}
		    else
			{
			    motor_command = 2;
			}
		}
		else
		{
		    motor_command = 2;
		}
		//*/
	}
	
	previous_light = light;

	if (light < 900 && light > 800)
	{
		color[0] = 1;
		color[1] = 1;
		color[2] = 0;
	}
	else if (light < 800 && light > 700)
	{
		color[0] = 0;
		color[1] = 1;
		color[2] = 1;
	}
	else if (light < 700 && light > 600)
	{
		color[0] = 1;
		color[1] = 0;
		color[2] = 1;
	}
	else if (light < 600 && light > 500)
	{
		color[0] = 0;
		color[1] = 1;
		color[2] = 0;
	}
	else if (light < 500 && light > 400)
	{
		color[0] = 0;
		color[1] = 0;
		color[2] = 1;
	}
	else if (light < 100)
	{
		color[0] = 1;
		color[1] = 0;
		color[2] = 0;
	}
	else
	{
		color[0] = 0;
		color[1] = 0;
		color[2] = 0;
	}

	//timer to controll how frequently i tx
	if ((timer % 10) == 0)
	{
		tx_request = 1;
		
	}
	timer++;
	
}

//Swarms Project Controller Brazil Nut Algorithim
void robot::controller_brazil_nut()
{
    //set artificial radius
    a = 25;
	b = 4;

	if (id < 25)
	{
	   	a_radius = a*b;
	    color[0] = 1;
		color[1] = 0;
		color[2] = 0;
	}
	/*
	else if (id < 75)
	{
	   	a_radius = a*b*2;
	    color[0] = 0;
		color[1] = 0;
		color[2] = 1;
	}
	//*/
	else
	{
	    a_radius = a;
	    color[0] = 0;
		color[1] = 1;
		color[2] = 0;
	}

	//light following
	if (light < 100)
	{
	    motor_command = 1;
	}
	else
	{
	    motor_command = 2;

	    if (previous_light < light)
		{
		    if (motor_command == 2)
			{
			    motor_command = 3;
			}
		    else
			{
			    motor_command = 2;
			}
		}
		else
		{
		  //motor_command = 2;
		}
	}
	
	previous_light = light;
	
	//neighbor avoidance
	data_out.id = id;//send out my id

					 //if i received a message
	if (incoming_message_flag == 1)
	{
		//clear the message rx flag
		incoming_message_flag = 0;

		//compare current distance to radius
		if (data_in.distance < a_radius)
		{
		    if (motor_command == 1)
			{
			    motor_command = 2;
			}
		    else if (motor_command == 2)
			{
			    motor_command = 3;
			}
			else 
			{
			    motor_command = 1;
			}
		}

	}

	//timer to controll how frequently i tx
	if ((timer % 10) == 0)
	{
		tx_request = 1;
		
	}
	timer++;
	
}

//Test function using vectors
void robot::controller_brazil_nut_test()
{
	data_out.id = id;//send out my id

					 //if i received a message
	if (incoming_message_flag == 1)
	{
		//clear the message rx flag
		incoming_message_flag = 0;
		
		test = 0;
		i = 0;

		//chech incoming id against list
		/*
		for (i = 0;i < neighbor_id.size();i++)
		{
		    if (neighbor_id.at(i) == data_in.id)
			{
			    index = i;
				test = 1;
				it = neighbor_id.begin() + i;
		    }
		}
		*/
		//if id not in list, append to list with current distance and 1 for motor control
		if (test == 0)
		{
		    neighbor_id.push_back (data_in.id);
			neighbor_distance.push_back (data_in.distance);
			index = neighbor_id.size();
			it = neighbor_id.end();
		}
		
		/*
		test = std::find(neighbor_id.begin(), neighbor_id.end(), data_in.id)

		if(test != neighbor_id.end()) 
		{
		    
		    for (i = 0;i < neighbor_id.size();i++)
			{
			    if (neighbor_id.at(i) == data_in.id)
				{
				    index = i;
					test = 1;
					it = neighbor_id.begin() + i;
				}
			}
		} 
		else 
		{
		    //if id not in list, append to list with current distance and 1 for motor control
		    neighbor_id.push_back (data_in.id);
			neighbor_distance.push_back (data_in.distance);
			index = neighbor_id.size();
			it = neighbor_id.end();
		}
		*/

		//compare current distance to radius
		if (data_in.distance < a_radius)
		{
		    //if d < r, check previous distance
		  if (data_in.distance > neighbor_distance.at(index))
			{
			    motor_command = 1;
				color[0] = 1;
				color[1] = 0;
				color[2] = 0;
			}
			else
			{
			    if (previous_command == 1)
			    {
				    color[0] = 0;
					color[1] = 1;
					color[2] = 0;
					motor_command = 2;
				}
				else
				{
				    color[0] = 0;
					color[1] = 0;
					color[2] = 1;
					motor_command = 3;
				}
			}
		    previous_command = motor_command;
		}
		neighbor_distance.insert ( it, data_in.distance);
	}

	//timer to controll how frequently i tx
	if ((timer % 10) == 0)
	{
		tx_request = 1;

	}
	timer++;
}

//example controller that has one robot orbit the other
void robot::controller_orbit()
{
	data_out.id = id;//send out my id

					 //if i received a message
	if (incoming_message_flag == 1)
	{
		//clear the message rx flag
		incoming_message_flag = 0;

		//if i have a higher id than my neighbors
		if ((data_in.id)< id)
		{
			//choose motor command based on distance and past distance value
			color[0] = 1;
			if (data_in.distance > 55)
			{

				if (previous_distance>data_in.distance)
				{
					motor_command = 1;
					color[0] = 1;
					color[1] = 0;
					color[2] = 0;
				}
				else
				{
					color[0] = 0;
					color[1] = 1;
					color[2] = 0;
					motor_command = 2;
				}
			}
			else
			{
				if (previous_distance<data_in.distance)
				{
					color[0] = 1;
					color[1] = 0;
					color[2] = 0;
					motor_command = 1;
				}
				else
				{
					color[0] = 0;
					color[1] = 0;
					color[2] = 1;
					motor_command = 3;
				}
			}

		}
		previous_distance = data_in.distance;
	}

	//timer to controll how frequently i tx
	if ((timer % 10) == 0)
	{
		tx_request = 1;


	}
	timer++;
}

//different controller to do hop count (gradient) display
void robot::controller_timestep_gradient()
{
	//only seed has hop 0
	if (hop != 0)
	{
		//getting a message
		if (incoming_message_flag == 1)
		{
			//clear flag
			incoming_message_flag = 0;

			//update hop if rx'ed message is a lower hop
			if ((data_in.message)< hop)
			{

				hop = data_in.message + 1;

			}
		}
	}

	//start sending out my new hop
	data_out.message = hop;




	//update color based on hop
	if ((hop % 6) == 0)
	{
		color[0] = 1;
		color[1] = 0;
		color[2] = 0;
	}
	else if ((hop % 6) == 1)
	{
		color[0] = 0;
		color[1] = 1;
		color[2] = 0;
	}
	else if ((hop % 6) == 2)
	{
		color[0] = 0;
		color[1] = 0;
		color[2] = 1;
	}
	else if ((hop % 6) == 3)
	{
		color[0] = 0;
		color[1] = 1;
		color[2] = 1;
	}
	else if ((hop % 6) == 4)
	{
		color[0] = 1;
		color[1] = 0;
		color[2] = 1;
	}
	else if ((hop % 6) == 5)
	{
		color[0] = 1;
		color[1] = 1;
		color[2] = 0;
	}




	//transmit every 100 timer ticks
	if ((timer % 100) == 0)
	{
		tx_request = 1;

	}

	timer++;

}

void robot::init(int x, int y, int t)
{


	//initalize robot variables
	pos[0] = x;
	pos[1] = y;
	pos[2] = t;
	motor_command = 0;
	timer = rand() / 100;
	incoming_message_flag = 0;
	tx_request = 0;
	id = rand();
	hop = 255;
	rand();
	motor_error = gaussrand()*motion_error_std;

	//set the robot at this position to be the seed of the gradient/hop count
	if ((x == 200) && (y == 200))
		hop = 0;

	if ((x == 150) && (y == 150))
	    motor_command = 0;
	    color[0] = 1;
		color[1] = 1;
		color[2] = 1;

}

double robot::gaussrand()
{
	static double V1, V2, S;
	static int phase = 0;
	double X;

	if (phase == 0) {
		do {
			double U1 = (double)rand() / RAND_MAX;
			double U2 = (double)rand() / RAND_MAX;

			V1 = 2 * U1 - 1;
			V2 = 2 * U2 - 1;
			S = V1 * V1 + V2 * V2;
		} while (S >= 1 || S == 0);

		X = V1 * sqrt(-2 * log(S) / S);
	}
	else
		X = V2 * sqrt(-2 * log(S) / S);

	phase = 1 - phase;

	return X;
}
