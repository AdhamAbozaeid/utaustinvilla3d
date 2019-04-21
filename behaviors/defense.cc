/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

#include "naobehavior.h"
#include "common.h"

/* If the ball is predicted to reach our goal at this distance from the post, 
still try to block it*/
#define GOAL_MARGIN 0.5

/* Checks if the ball is currently moving */
bool isBallMoving(const WorldModel *worldModel, double &slope, double &intercept) {
    static VecPosition lastBall = worldModel->getBallGroundTruth();
    static double lastTime = worldModel->getTime();

    double thisTime = worldModel->getTime();
    VecPosition thisBall = worldModel->getBall();

    thisBall.setZ(0);
    lastBall.setZ(0);
    
    if (thisBall.getDistanceTo(lastBall) > 0.2) {
        // the ball moved!
        //cout << "last: " << lastBall << " now: " << thisBall << " Dist: " << thisBall.getDistanceTo(lastBall)<<endl;
        slope = (lastBall.getY() - thisBall.getY()) / (lastBall.getX() - thisBall.getX());
        intercept = thisBall.getY() - (slope * thisBall.getX());
        //cout<<slope<< " "<<intercept<<endl;
        lastBall = thisBall;
        lastTime = thisTime;
        return true;
    }

//    if (thisTime - lastTime < 0.5) {
//        // not sure yet if the ball has settled
//        return true;
//    } else {
//        return false;
//    }
    return false;
}

SkillType NaoBehavior::goalieAction() {
    double slope = 0, intercept = 0, yAtGoal;
    if (isBallMoving(worldModel, slope, intercept)) {
        // Is ball moving towards goal?
        yAtGoal = (slope * - HALF_FIELD_X) + intercept;
        if ((yAtGoal < (HALF_GOAL_Y + GOAL_MARGIN)) && (yAtGoal > -(HALF_GOAL_Y + GOAL_MARGIN))) {
            VecPosition target = worldModel->getMyPosition();
            if (yAtGoal > HALF_GOAL_Y)
                yAtGoal = HALF_GOAL_Y;
            else if (yAtGoal < -HALF_GOAL_Y)
                yAtGoal = -HALF_GOAL_Y;
            //cout<<slope <<" " << intercept << " " <<yAtGoal<<endl;
            target.setY(yAtGoal);
            //cout<<"Goalie: "<<target<<endl;
            return goToTarget(target);
        }
    }
    return SKILL_STAND;
}

SkillType NaoBehavior::defense() {

    if (worldModel->getUNum() == ROLE_GOALIE) {
        return goalieAction();
    } else {
        VecPosition target = getPosInFormation();
        if (me.getDistanceTo(target) < .25) {
            // Close enough to desired position and orientation so just stand
            return SKILL_STAND;
        } else {
            // Move toward target location
            return goToTarget(target);
        }
    }
}
