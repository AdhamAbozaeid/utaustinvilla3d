/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

#include "naobehavior.h"
#include "common.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>

int playerClosestToBall = -1;
int playerClosestToBall2 = -1;
int playerClosestToBall3 = -1;
int playerClosestToBall4 = -1;
double closestDistanceToBall = 1000000;
double closestDistanceToBall2 = 1000000;
double closestDistanceToBall3 = 1000000;
double closestDistanceToBall4 = 1000000;


int playerClosestToMe = -1;
int playerClosestToMe2 = -1;
int playerClosestToMe3 = -1;
int playerClosestToMe4 = -1;
double closestDistanceToMe = 1000000;
double closestDistanceToMe2 = 1000000;
double closestDistanceToMe3 = 1000000;
double closestDistanceToMe4 = 1000000;


using namespace std;

void NaoBehavior::find_closest_player_to_ball() {
    for (int i = WO_TEAMMATE1; i < WO_TEAMMATE1 + NUM_AGENTS; ++i) {
        VecPosition temp;
        int playerNum = i - WO_TEAMMATE1 + 1;
        if (worldModel->getUNum() == playerNum) {
            // This is us
            temp = worldModel->getMyPosition();
        }
        else {
            WorldObject* teammate = worldModel->getWorldObject(i);
            if (teammate->validPosition) {
                temp = teammate->pos;
            }
            else {
                continue;
            }
        }
        temp.setZ(0);

        //find 3 nearest player
        double distanceToBall = temp.getDistanceTo(ball);
        if (distanceToBall < closestDistanceToBall4) {
            if (distanceToBall < closestDistanceToBall3) {
                if (distanceToBall < closestDistanceToBall2) {
                    if (distanceToBall < closestDistanceToBall) {
                        playerClosestToBall = playerNum;
                        closestDistanceToBall = distanceToBall;
                    } else {
                        playerClosestToBall2 = playerNum;
                        closestDistanceToBall2 = distanceToBall;
                    }
                } else {
                    playerClosestToBall3 = playerNum;
                    closestDistanceToBall3 = distanceToBall;
                }
            } else {
                playerClosestToBall4 = playerNum;
                closestDistanceToBall4 = distanceToBall;
            }

        }
    }

    closestPlayer[0] = playerClosestToBall;
    closestPlayer[1] = closestDistanceToBall;
    closestPlayer[2] = playerClosestToBall2;
    closestPlayer[3] = closestDistanceToBall2;
    closestPlayer[4] = playerClosestToBall3;
    closestPlayer[5] = closestDistanceToBall3;
    closestPlayer[6] = playerClosestToBall4;
    closestPlayer[7] = closestDistanceToBall4;

}

//find the closest oppenent to me to AVOID him
void NaoBehavior::find_closest_opponent_to_me(){

    for (int i = WO_OPPONENT1; i < WO_OPPONENT1 + NUM_AGENTS; ++i) {
        VecPosition temp,me;
        int playerNum = i - WO_OPPONENT1 + 1;
	temp = worldModel->getOpponent(playerNum);
        temp.setZ(0);
	me = worldModel->getMyPosition();
        //find 3 nearest player
        double distanceToMe = temp.getDistanceTo(me);
        if (distanceToMe < closestDistanceToMe4) {
            if (distanceToMe < closestDistanceToMe3) {
                if (distanceToMe < closestDistanceToMe2) {
                    if (distanceToMe < closestDistanceToMe) {
                        playerClosestToMe = playerNum;
                        closestDistanceToMe = distanceToMe;
                    } else {
                        playerClosestToMe2 = playerNum;
                        closestDistanceToMe2 = distanceToMe;
                    }
                } else {
                    playerClosestToMe3 = playerNum;
                    closestDistanceToMe3 = distanceToMe;
                }
            } else {
                playerClosestToMe4 = playerNum;
                closestDistanceToMe4 = distanceToMe;
            }

        }
    }

}


SkillType NaoBehavior::offense(){
VecPosition target=VecPosition(HALF_FIELD_X, 0, 0);

//step 1, shift from defense formation to offense formation

//step 2, dribble the ball forward until unable to
//So if the distance is smaller than 2.0 we think it's dangerous
	if(closestDistanceToMe >= 2.0){
		return kickBall(KICK_DRIBBLE, target);
	}

//step 3, pass the ball to teammate or shoot to goal

	if(closestDistanceToMe < 2.0 && worldModel->getMyPosition().getX() < 10){
		assignRoles();
		//pass to forward left or right
		return pass_ball(ROLE_FRONT_RIGHT);
	}else if(1){
		//kick to goal
		return kickBall(KICK_IK, target);
	}

}

SkillType NaoBehavior::pass_ball(int pass_to) {

    if (worldModel->getUNum() != playerClosestToBall) {
        return SKILL_STAND;
    }
    ofstream problem_file;
    problem_file.open("../planner/soccer_problem.pddl");
    string line;
    line = "(define (problem navigation_soccer_world)\n";
    problem_file << line;
    line = "(:domain NAVIGATION_SOCCER)\n";
    problem_file << line;
    line = "(:objects a1 a2 a3 a4 a5 a6 a7 a8 a9 a10 a11 - agent)\n";
    problem_file << line;
    line = "(:init (at a1)\n";
    problem_file << line;
    for (int agent = WO_TEAMMATE1; agent < WO_TEAMMATE1 + NUM_AGENTS; agent++) {
        for (int teammate = WO_TEAMMATE1; teammate < WO_TEAMMATE1 + NUM_AGENTS; teammate++) {
            double closest_opp_distance = 10000;
            for (int opponent = WO_OPPONENT1; opponent < WO_OPPONENT1 + NUM_AGENTS; opponent++) {
                double distance = worldModel->getTeammate(teammate).getDistanceTo(worldModel->getOpponent(opponent));
                //cout<<distance<<endl;
                if (distance != 0 && distance < closest_opp_distance) {
                    closest_opp_distance = distance;
                }
            }
            double distance_to_me = worldModel->getTeammate(agent).getDistanceTo(teammate);

            // let's say the ball travels 5 times faster than agent
            // so if the teammate is within 5 time the distance to me than opponent
            // we are safe to make the pass
            if (distance_to_me / closest_opp_distance < 5 && distance_to_me < 10) {
                stringstream tmp_teammate, tmp_agent;
                tmp_teammate << teammate - WO_TEAMMATE1 + 1;
                tmp_agent << agent - WO_TEAMMATE1 + 1;
                line = "(connected a" + tmp_teammate.str() + " a" + tmp_agent.str() + ")\n";
                problem_file << line;
                line = "(connected a" + tmp_agent.str() + " a" + tmp_teammate.str() + ")\n";
                problem_file << line;

            }
        }
    }

    line = ")\n";
    problem_file << line;

    // we need to determine who we want to pass the ball to
    stringstream tmp_pass_to;
    tmp_pass_to << pass_to;// - WO_TEAMMATE1 + 1;
    line = "(:goal (at a"+tmp_pass_to.str()+"))";
    problem_file << line;
    line = ")";
    problem_file << line;
    problem_file.close();

    //run the path planning code
    int sys_return = system("python ../planner/fast-downward.py ../planner/domain.pddl ../planner/soccer_problem.pddl --search 'astar(lmcut())'> soccer_solution.soln &");
    //open the solution file and start decoding
    ifstream solution_file;
    solution_file.open("soccer_solution.soln");
    int path_found_flag = 0;
    int pass_ball_to = -1;
    if (solution_file) {
        while (path_found_flag == 0 && getline(solution_file, line)) {
            if (line.find("move_agent") != std::string::npos) {
                pass_ball_to = atoi(line.substr(15, 1).c_str());
                path_found_flag = 1;
            }
        }
        cout << "Pass ball to:" << pass_ball_to << endl;
    }
    //pass the ball
    //return kickBall(KICK_IK, worldModel->getTeammate(pass_ball_to + WO_TEAMMATE1 - 1));
    return kickBall(KICK_IK, worldModel->getTeammate(pass_ball_to));
}

