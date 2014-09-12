#include <iostream>
#include <vector>
#include <queue>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <math.h>
#include <stdint.h>
#include <assert.h>
#include <errno.h>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#include <wavefront/graphBuilder.h>
#include <wavefront/robot.h>
#include <wavefront/Vector2.h>
#include <wavefront/node.h>

#include <stack>
#include <queue>

class goal{
	public:
		Vector2 pose;
		nav_msgs::Odometry poseOdom;

		int newPose;
		int onNav;

		ros::Subscriber goalPose;

		void poseCallback(nav_msgs::OdometryConstPtr& msg){
			this->poseOdom = *msg;
			pose.x = poseOdom.pose.pose.position.x;
			pose.y = poseOdom.pose.pose.position.y;

			newPose = 1;
		}

		goal(ros::NodeHandle& nh)
		{
			goalPose = nh.subscribe("/robot_1/base_pose_ground_truth", 1, &goal::poseCallback, this);
			newPose = 0;
			onNav = 0;
		}

		void poseCallback(const nav_msgs::Odometry::ConstPtr& msg)
		{
			pose.x = msg->pose.pose.position.x;
			pose.y = msg->pose.pose.position.y;
		}
};

class wavefrontCost  //used as a cell in the priority queue used in dijstra's algorithm
{
    public:
        node* vNode;       // node
        double cost; 
        node* s;           // neighbor of the robot that initialized that cell

        void clear(){
        	vNode = NULL;
        	cost = 0;
        	s = NULL;
        }

        wavefrontCost(){
        	clear();
        }
};

class compareCost    //função usada pela Priotity Queue como critério de ordenação
{
    public:
        bool operator() (wavefrontCost& c1, wavefrontCost& c2)
        {
            if (c1.cost > c2.cost)
            {
                return true;
            }
            else return false;
        }
};

class wavefront{
	std::stack<node*> stackNodes;

	graph* grafo;
	robot* robo;
	goal* alvo;
	public:
		void calculateTrajectory()
		{
			node *n, *neighbor, *nodeRobot;
			std::priority_queue<wavefrontCost,  std::vector<wavefrontCost>, compareCost> PQ;

			wavefrontCost w;

			grafo->clearGraph();


			node* nodeGoal = grafo->getNodeByPose(alvo->pose);
			nodeRobot = grafo->getNodeByPose(robo->pose);
			w.vNode = nodeGoal;
			w.cost = 0;
			w.s = NULL;

			PQ.push(w);

			while(!PQ.empty() || n == nodeGoal)
			{
				w = PQ.top();    // gets the element on the queue with the lowest weight
				PQ.pop();        // and removes it
				n = w.vNode;

				if(w.cost > n->powerDist) continue;

				for(int i = 0; i < 8; i = i+2)
				{
					neighbor = n->neighbor[i];

					if(neighbor!=NULL)
					{
						w.cost = w.cost + 1;
						w.vNode = n;
						n->powerDist = w.cost;
						n->s = n;
					}
				}

				node* p = nodeGoal->neighbor[0];
				for(int i = 0; i < 8; i = i+2)
				{
					if(p->powerDist < nodeGoal->neighbor[i]->powerDist)
						p = nodeGoal->neighbor[i];
				}

				stackNodes.push(p);

				while(p!=nodeRobot)
				{
					n = p;
					for(int i = 0; i < 8; i = i+2)
					{
						if(p->powerDist < n->neighbor[i]->powerDist && n != n->neighbor[i])
							p = n->neighbor[i];
					}
					stackNodes.push(p);
				}
				alvo->newPose = 0;
				alvo->onNav = 1;

			}
		}

		void navigate() 
		{
			node* subgoal = stackNodes.top();

			double dx = subgoal->pose.x - robo->pose.x;
			double dy = subgoal->pose.y - robo->pose.y;

			if( (dx < grafo->getSquareSize()*grafo->getSizeMetersPixel()/4) && (dy < grafo->getSquareSize()*grafo->getSizeMetersPixel()/4) )
			{
				stackNodes.pop();
				if(stackNodes.empty())
				{
					ROS_INFO("GOAL REACHED.");
					alvo->newPose = 0;
					robo->setSpeed(0, 0);

				}
				else
				{
					subgoal = stackNodes.top();
					dx = subgoal->pose.x - robo->pose.x;
					dy = subgoal->pose.y - robo->pose.y;
					robo->setSpeedHolo(dx, dy);
					alvo->onNav = 0;
				}
			}
			else
			{
				robo->setSpeedHolo(dx, dy);
			}
			robo->publishSpeed();
		}

		wavefront(graph* g, robot* r, goal* gl)
		{
			this->grafo = g;
			this->robo = r;
			this->alvo = gl;
		}
};

 

int main( int argc, char** argv )
{
    ros::init(argc, argv, "PID");
    ros::NodeHandle n; 
    
	double discretization, resolution;
    int threshold = 250;
    std::string mapFile, outFile;

    rgb color(255, 0, 0);

    if(ros::param::get("/graph/discretization", discretization));
    else{
        ROS_INFO("Error getting parameter square size parameter");
        exit(1);
    }
    if(ros::param::get("/graph/resolution", resolution));
    else{
        ROS_INFO("Error getting parameter: resolution");
        exit(1);
    }
    if(ros::param::get("/graph/map",mapFile));
    else{
        ROS_INFO("Error getting parameter: map file.");
        exit(1);
    }
     if(ros::param::get("/graph/outmap",outFile));
    else{
        ROS_INFO("Error getting parameter: out file.");
        exit(1);
    }


    graph Grafo(discretization, (char*) mapFile.c_str(), resolution, (char*) outFile.c_str());
	Grafo.BuildGraph(threshold); 


	robot rbx(0, 0, color, "robot_");
	goal gx;

	wavefront wvf(&Grafo, &rbx, &gx);


	ros::Rate loop_rate(20);

	while(ros::ok()){

		if(gx.newPose == 1)
			wvf.calculateTrajectory();

		if (gx.onNav)
			wvf.navigate();
		else
		{
			rbx.setSpeed(0, 0);
			rbx.publishSpeed();
		}
		loop_rate.sleep();
		ros::spinOnce();
	}

	return 0;

}