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

// Given 2 points on a line, get the slope and intercept

void getLineParam(VecPosition point1, VecPosition point2, double &slope, double &intercept) {
    slope = (point1.getY() - point2.getY()) / (point1.getX() - point2.getX());
    intercept = point1.getY() - (slope * point1.getX());
}

/* Checks if the ball is currently moving */
bool isBallMoving(const WorldModel *worldModel, double &slope, double &intercept) {
    static VecPosition lastBall = worldModel->getBallGroundTruth();
    static double lastTime = worldModel->getTime();
    static double lastSlope = 0, lastIntercept = 0;

    double thisTime = worldModel->getTime();
    VecPosition thisBall = worldModel->getBall();

    thisBall.setZ(0);
    lastBall.setZ(0);

    if (thisBall.getDistanceTo(lastBall) > 0.1) {
        // the ball moved!
        //cout << "last: " << lastBall << " now: " << thisBall << " Dist: " << thisBall.getDistanceTo(lastBall)<<endl;
        getLineParam(thisBall, lastBall, slope, intercept);
        //cout<<slope<< " "<<intercept<<endl;
        lastBall = thisBall;
        lastTime = thisTime;
        lastSlope = slope;
        lastIntercept = intercept;
        return true;
    }

    if (thisTime - lastTime < 0.5) {
        // not sure yet if the ball has settled
        slope = lastSlope;
        intercept = lastIntercept;
        return true;
    } else {
        return false;
    }
    return false;
}

SkillType NaoBehavior::goalieAction() {
    double slope = 0, intercept = 0, yAtGoal;
    if (isBallMoving(worldModel, slope, intercept)) {
        // Is ball moving towards goal?
        yAtGoal = (slope * - HALF_FIELD_X) + intercept;
        if ((yAtGoal < (HALF_GOAL_Y + GOAL_MARGIN)) && (yAtGoal > -(HALF_GOAL_Y + GOAL_MARGIN))) {
            VecPosition target = VecPosition(-HALF_FIELD_X + 0.25, 0, 0);
            if (yAtGoal > HALF_GOAL_Y)
                yAtGoal = HALF_GOAL_Y;
            else if (yAtGoal < -HALF_GOAL_Y)
                yAtGoal = -HALF_GOAL_Y;
            //cout<<slope <<" " << intercept << " " <<yAtGoal<<endl;
            target.setY(yAtGoal);

#ifdef ENABLE_DRAWINGS
            worldModel->getRVSender()->drawLine("BALL_GOAL", worldModel->getBall().getX(), worldModel->getBall().getY(), -HALF_FIELD_X, yAtGoal, RVSender::MAGENTA);
#endif
            //cout<<"Goalie: "<<target<<endl;
            //double angle;
            //double distance;
            //getTargetDistanceAndAngle(target, distance, angle);
            //return goToTargetRelative(worldModel->g2l(target), angle, 2);
            return goToTarget(target);
        }
    }
    return SKILL_STAND;
}

SkillType NaoBehavior::backAction() {
    VecPosition Ball = worldModel->getBall();
    VecPosition goalCenter = VecPosition(-HALF_FIELD_X, 0, 0);
    VecPosition target;
    double slope, intercept, y, distBallToGoal, distAgentToGoal;
    float angle;

    getLineParam(Ball, goalCenter, slope, intercept);
    angle = atan(slope);
    distBallToGoal = abs(Ball.getDistanceTo(goalCenter));


    if (worldModel->getUNum() == ROLE_BACK_RIGHT) {
        /*Stand on line few degrees above line from ball to center of goal
         Back Left will stand at an offset below the line*/
        angle -= 0.07 * (signbit(angle) ? -1 : 1);

        if (distBallToGoal > 2)
            distAgentToGoal = 2;
        else
            distAgentToGoal = distBallToGoal - 0.5;
        target.setX(-HALF_FIELD_X + (distAgentToGoal * cos(angle)));
        y = distAgentToGoal * sin(angle);
        //cout<<"BR angle: " << angle<<" x: "<<-HALF_FIELD_X + (2*sin(angle))<<" Y: "<<y<<endl;
        target.setY(y);

#ifdef ENABLE_DRAWINGS
        worldModel->getRVSender()->drawLine("BALL_GOAL", Ball.getX(), Ball.getY(), goalCenter.getX(), goalCenter.getY(), RVSender::GREEN);
#endif
    } else {
        angle += 0.07 * (signbit(angle) ? -1 : 1);

        if (distBallToGoal > 3)
            distAgentToGoal = 3;
        else
            distAgentToGoal = distBallToGoal - 0.1;
        target.setX(-HALF_FIELD_X + (distAgentToGoal * cos(angle)));
        y = distAgentToGoal * sin(angle);

        //cout<<"BL angle: " << angle<<" x: "<<-HALF_FIELD_X + (3*sin(angle))<<" Y: "<<y<<endl;
        target.setY(y);
    }

    target = collisionAvoidance(true /*teammate*/, false/*opponent*/, true/*ball*/, 1/*proximity thresh*/, .5/*collision thresh*/, target, true/*keepDistance*/);

    // Use goto relative to face the ball
    return goToTargetRelative(worldModel->g2l(target), angle);
}

SkillType NaoBehavior::defense() {

    switch (worldModel->getUNum()) {
        case ROLE_GOALIE:
            return goalieAction();
            break;
        case ROLE_BACK_LEFT:
        case ROLE_BACK_RIGHT:
            return backAction();
            break;
        default:
            VecPosition target = getPosInFormation();
            if (me.getDistanceTo(target) < .25) {
                // Close enough to desired position and orientation so just stand
                return SKILL_STAND;
            } else {
                // Move toward target location
                return goToTarget(target);
            }
            break;
    }
}
