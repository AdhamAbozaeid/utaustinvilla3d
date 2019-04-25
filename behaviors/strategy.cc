#include "naobehavior.h"
#include "../rvdraw/rvdraw.h"
#include "common.h"
#include "defense.cc"
#include "offense.cc"

extern int agentBodyType;

VecPosition NaoBehavior::getPosInFormation() {
    VecPosition target;
    switch (worldModel->getUNum()) {
        case ROLE_GOALIE:
            target.setX(-HALF_FIELD_X + 0.25);
            target.setY(0);
            break;
        case ROLE_ON_BALL:
            target.setX(worldModel->getBall().getX());
            target.setY(worldModel->getBall().getY());
            break;
        case ROLE_FRONT_RIGHT:
            target.setX(worldModel->getBall().getX() - 0.5);
            target.setY(worldModel->getBall().getY() - 2);
            break;
        case ROLE_FRONT_LEFT:
            target.setX(worldModel->getBall().getX() - 0.5);
            target.setY(worldModel->getBall().getY() + 2);
            break;
        case ROLE_FORWARD_CENTER:
            target.setX(HALF_FIELD_X / 2);
            target.setY(0);
            break;
        case ROLE_SUPPORTER:
            target.setX(worldModel->getBall().getX() - 2);
            target.setY(worldModel->getBall().getY());
            break;
        case ROLE_WING_RIGHT:
            target.setX(worldModel->getBall().getX() - 3);
            target.setY(worldModel->getBall().getY() - 2);
            break;
        case ROLE_WING_LEFT:
            target.setX(worldModel->getBall().getX() - 3);
            target.setY(worldModel->getBall().getY() + 2);
            break;
        case ROLE_MIDDLE:
            target.setX(-HALF_FIELD_X / 2);
            target.setY(0);
            break;
        case ROLE_BACK_RIGHT:
        {
            VecPosition Ball = worldModel->getBall();
            VecPosition goalCenter = VecPosition(-HALF_FIELD_X, 0, 0);
            double slope, intercept, y, distBallToGoal, distAgentToGoal;
            float angle;

            distBallToGoal = Ball.getDistanceTo(goalCenter);
            getLineParam(Ball, goalCenter, slope, intercept);
            angle = atan(slope);
            /*Stand on line few degrees above line from ball to center of goal
             Back Left will stand at an offset below the line*/
            angle -= 0.07 * (signbit(angle) ? -1 : 1);

            if (distBallToGoal > 2)
                distAgentToGoal = 2;
            else
                distAgentToGoal = distBallToGoal - 0.01;
            target.setX(-HALF_FIELD_X + (distAgentToGoal * cos(angle)));
            y = distAgentToGoal * sin(angle);
            //cout<<"BR angle: " << angle<<" x: "<<-HALF_FIELD_X + (2*sin(angle))<<" Y: "<<y<<endl;
            target.setY(y);
            break;
        }
        case ROLE_BACK_LEFT:
        {
            VecPosition Ball = worldModel->getBall();
            VecPosition goalCenter = VecPosition(-HALF_FIELD_X, 0, 0);
            double slope, intercept, y, distBallToGoal, distAgentToGoal;
            float angle;

            distBallToGoal = Ball.getDistanceTo(goalCenter);
            getLineParam(Ball, goalCenter, slope, intercept);
            angle = atan(slope);
            angle += 0.07 * (signbit(angle) ? -1 : 1);

            if (distBallToGoal > 3)
                distAgentToGoal = 3;
            else
                distAgentToGoal = distBallToGoal - 0.01;
            target.setX(-HALF_FIELD_X + (distAgentToGoal * cos(angle)));
            y = distAgentToGoal * sin(angle);

            //cout<<"BL angle: " << angle<<" x: "<<-HALF_FIELD_X + (3*sin(angle))<<" Y: "<<y<<endl;
            target.setY(y);
            break;
        }
    }
    /* Verify position*/
    if (target.getX() > HALF_FIELD_X)
        target.setX(HALF_FIELD_X);
    else if (target.getX() < -HALF_FIELD_X)
        target.setX(-HALF_FIELD_X);
    if (target.getY() > HALF_FIELD_Y)
        target.setY(HALF_FIELD_Y);
    else if (target.getY() < -HALF_FIELD_Y)
        target.setY(-HALF_FIELD_Y);

    // Adjust target to not be too close to teammates or the ball
    if (worldModel->getUNum() != ROLE_ON_BALL)
        target = collisionAvoidance(true /*teammate*/, false/*opponent*/, true/*ball*/, 1/*proximity thresh*/, .5/*collision thresh*/, target, true/*keepDistance*/);

    return target;
}

/*
 * Real game beaming.
 * Filling params x y angle
 */
void NaoBehavior::beam(double& beamX, double& beamY, double& beamAngle) {
    //beamX = -HALF_FIELD_X + worldModel->getUNum();
    //beamY = 0;if case ROLE_ON_BALL:
    if (worldModel->getUNum() == ROLE_ON_BALL) {
        beamX = -1;
        beamY = 0;
    } else if (worldModel->getUNum() == ROLE_FORWARD_CENTER) {
        beamX = -2;
        beamY = 1;
    } else {
        VecPosition target = getPosInFormation();
        beamX = target.getX();
        beamY = target.getY();
    }
    beamAngle = 0;
    memset(markingAgents, -1, sizeof(markingAgents));
}

SkillType NaoBehavior::selectSkill() {
    // My position and angle
    //cout << worldModel->getUNum() << ": " << worldModel->getMyPosition() << ",\t" << worldModel->getMyAngDeg() << "\n";

    // Position of the ball
    //cout << worldModel->getBall() << "\n";

    // Example usage of the roboviz drawing system and RVSender in rvdraw.cc.
    // Agents draw the position of where they think the ball is
    // Also see example in naobahevior.cc for drawing agent position and
    // orientation.
    /*
    worldModel->getRVSender()->clear(); // erases drawings from previous cycle
    worldModel->getRVSender()->drawPoint("ball", ball.getX(), ball.getY(), 10.0f, RVSender::MAGENTA);
     */
#ifdef ENABLE_DRAWINGS
    worldModel->getRVSender()->clear();
    worldModel->getRVSender()->clearStaticDrawings();
#endif

    // ### Demo Behaviors ###

    // Walk in different directions
    //return goToTargetRelative(VecPosition(1,0,0), 0); // Forward
    //return goToTargetRelative(VecPosition(-1,0,0), 0); // Backward
    //return goToTargetRelative(VecPosition(0,1,0), 0); // Left
    //return goToTargetRelative(VecPosition(0,-1,0), 0); // Right
    //return goToTargetRelative(VecPosition(1,1,0), 0); // Diagonal
    //return goToTargetRelative(VecPosition(0,1,0), 90); // Turn counter-clockwise
    //return goToTargetRelative(VecPdosition(0,-1,0), -90); // Turn clockwise
    //return goToTargetRelative(VecPosition(1,0,0), 15); // Circle

    // Walk to the ball
    //return goToTarget(ball);

    // Turn in place to face ball
    /*double distance, angle;
    getTargetDistanceAndAngle(ball, distance, angle);
    if (abs(angle) > 10) {
      return goToTargetRelative(VecPosition(), angle);
    } else {
      return SKILL_STAND;
    }*/

    // Walk to ball while always facing forward
    //return goToTargetRelative(worldModel->g2l(ball), -worldModel->getMyAngDeg());

    // Dribble ball toward opponent's goal
    //return kickBall(KICK_DRIBBLE, VecPosition(HALF_FIELD_X, 0, 0));

    // Kick ball toward opponent's goal
    //return kickBall(KICK_FORWARD, VecPosition(HALF_FIELD_X, 0, 0)); // Basic kick
    //return kickBall(KICK_IK, VecPosition(HALF_FIELD_X, 0, 0)); // IK kick

    // Just stand in place
    //return SKILL_STAND;
    //return goto
    // Demo behavior where players form a rotating circle and kick the ball
    // back and forth
    //return demoKickingCircle();

    //return defense();
    return offense();
}

/*
 * Demo behavior where players form a rotating circle and kick the ball
 * back and forth
 */
SkillType NaoBehavior::demoKickingCircle() {
    // Parameters for circle
    VecPosition center = VecPosition(-HALF_FIELD_X / 2.0, 0, 0);
    double circleRadius = 5.0;
    double rotateRate = 2.5;

    // Find closest player to ball
    int playerClosestToBall = -1;
    double closestDistanceToBall = 10000;
    for (int i = WO_TEAMMATE1; i < WO_TEAMMATE1 + NUM_AGENTS; ++i) {
        VecPosition temp;
        int playerNum = i - WO_TEAMMATE1 + 1;
        if (worldModel->getUNum() == playerNum) {
            // This is us
            temp = worldModel->getMyPosition();
        } else {
            WorldObject* teammate = worldModel->getWorldObject(i);
            if (teammate->validPosition) {
                temp = teammate->pos;
            } else {
                continue;
            }
        }
        temp.setZ(0);

        double distanceToBall = temp.getDistanceTo(ball);
        if (distanceToBall < closestDistanceToBall) {
            playerClosestToBall = playerNum;
            closestDistanceToBall = distanceToBall;
        }
    }

    if (playerClosestToBall == worldModel->getUNum()) {
        // Have closest player kick the ball toward the center
        return kickBall(KICK_FORWARD, center);
    } else {
        // Move to circle position around center and face the center
        VecPosition localCenter = worldModel->g2l(center);
        SIM::AngDeg localCenterAngle = atan2Deg(localCenter.getY(), localCenter.getX());

        // Our desired target position on the circle
        // Compute target based on uniform number, rotate rate, and time
        VecPosition target = center + VecPosition(circleRadius, 0, 0).rotateAboutZ(360.0 / (NUM_AGENTS - 1)*(worldModel->getUNum()-(worldModel->getUNum() > playerClosestToBall ? 1 : 0)) + worldModel->getTime() * rotateRate);

        // Adjust target to not be too close to teammates or the ball
        target = collisionAvoidance(true /*teammate*/, false/*opponent*/, true/*ball*/, 1/*proximity thresh*/, .5/*collision thresh*/, target, true/*keepDistance*/);

        if (me.getDistanceTo(target) < .25 && abs(localCenterAngle) <= 10) {
            // Close enough to desired position and orientation so just stand
            return SKILL_STAND;
        } else if (me.getDistanceTo(target) < .5) {
            // Close to desired position so start turning to face center
            return goToTargetRelative(worldModel->g2l(target), localCenterAngle);
        } else {
            // Move toward target location
            return goToTarget(target);
        }
    }
}


