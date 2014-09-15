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

		void poseCallback(const nav_msgs::Odometry::ConstPtr& msg){
			this->poseOdom = *msg;
			if( abs(pose.x - poseOdom.pose.pose.position.x)>0.5 || abs(pose.y - poseOdom.pose.pose.position.y)>0.5)
			{
				newPose = 1;
				ROS_INFO("GOAL changed its position! Asking to recalculate the trajectory\n");
				pose.x = poseOdom.pose.pose.position.x;
				pose.y = poseOdom.pose.pose.position.y;
			}	
			
		}

		goal(ros::NodeHandle& nh)
		{
			goalPose = nh.subscribe("/robot_1/base_pose_ground_truth", 1, &goal::poseCallback, this);
			newPose = 0;
			onNav = 0;
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
			nodeGoal->powerDist = 0;
			w.s = NULL;			

			PQ.push(w);

			printf("oi\n");

			while(!PQ.empty() || n == nodeRobot)
			{
				w = PQ.top();    // gets the element on the queue with the lowest weight
				printf("pop, weight: %f\n", w.cost);
				PQ.pop();        // and removes it
				n = w.vNode;
				//printf("popped\n");

				if(w.cost > n->powerDist) continue;

				for(int i = 0; i < 8; i = i+2)
				{
					neighbor = n->neighbor[i];
					//printf("opa\n");
					if(neighbor!=NULL && (n->powerDist + 1 < neighbor->powerDist))
					{
						//printf("eita\n");
						wavefrontCost o;
						o.cost = n->powerDist + 1;
						o.vNode = neighbor;
						o.s = n;
						o.vNode->powerDist = o.cost;
						o.vNode->s = n;
						PQ.push(o);
						printf("cost %f \n", o.cost);
					}
				}

			}

			for(int i = grafo->vertices.y; i >= 0 ; i--)
			{
				for(int j = 0; j < grafo->vertices.x; j++)
				{
					if(grafo->getNodeByIndex(j, i) != NULL)	
						printf("%1.1f ", grafo->getNodeByIndex(j, i)->powerDist);
					else
						printf(" X  ");
				}
				printf("\n");
			}
			alvo->newPose = 0;
			alvo->onNav = 1;
		}

		void navigate() 
		{
			node* nodeRobot = grafo->getNodeByPose(robo->pose);
			node* nodeGoal = grafo->getNodeByPose(alvo->pose);

			//printf("nav\n");
			node* subgoal;
			double lowCost = HUGE_VAL;
			for(int i=0; i<8; i = i + 2)
			{
				//printf("1\n");
				if(nodeRobot->neighbor[i]!=NULL)
				{
					//printf("2\n");
					if(nodeRobot->neighbor[i]->powerDist < lowCost)
					{
						//printf("3\n");
						subgoal = nodeRobot->neighbor[i];
						lowCost = subgoal->powerDist;
						//printf("4\n");
					}
				}
			}

			double dx = subgoal->pose.x - robo->pose.x;
			double dy = subgoal->pose.y - robo->pose.y;

			if( (dx < grafo->getSquareSize()*grafo->getSizeMetersPixel()/4) && (dy < grafo->getSquareSize()*grafo->getSizeMetersPixel()/4) )
			{
				if(subgoal==nodeGoal)
				{
					ROS_INFO("GOAL REACHED.");
					alvo->newPose = 0;
					robo->setSpeed(0, 0);
					alvo->onNav = 0;

				}
			}
			else
			{
				robo->setSpeedHolo(dx, dy);
			}
			robo->publishSpeed();
			//printf("bye nav\n");
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
    ros::init(argc, argv, "wavefront");
    ros::NodeHandle n; 
    
	double discretization, resolution;
    int threshold = 250;
    std::string mapFile, outFile;

    rgb color(255, 0, 0);

    if(ros::param::get("/wavefront/discretization", discretization));
    else{
        ROS_INFO("Error getting parameter square size parameter");
        exit(1);
    }
    if(ros::param::get("/wavefront/resolution", resolution));
    else{
        ROS_INFO("Error getting parameter: resolution");
        exit(1);
    }
    if(ros::param::get("/wavefront/map",mapFile));
    else{
        ROS_INFO("Error getting parameter: map file.");
        exit(1);
    }
     if(ros::param::get("/wavefront/outmap",outFile));
    else{
        ROS_INFO("Error getting parameter: out file.");
        exit(1);
    }

    printf("disc: %f\n, resolution: %f\n", discretization, resolution);

    graph Grafo(discretization, (char*) mapFile.c_str(), resolution, (char*) outFile.c_str());
	Grafo.BuildGraph(threshold); 


	robot rbx(0, 0, color, "robot_", n);
	goal gx(n);

	wavefront wvf(&Grafo, &rbx, &gx);


	ros::Rate loop_rate(20);


	while(ros::ok()){
		if(gx.newPose == 1)
		{
			wvf.calculateTrajectory();
		} 		
		if (gx.onNav)
		{
			wvf.navigate();
		}
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