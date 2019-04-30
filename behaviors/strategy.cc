#include "naobehavior.h"
#include "../rvdraw/rvdraw.h"
#include "common.h"
#include "defense.cc"
#include "offense.cc"
#include "scram.cc"

extern int agentBodyType;

void NaoBehavior::assignRoles() {
    Test t;
    VecPosition agent, target;
    VecPosition Ball = worldModel->getBall();
    int noMarkedOpp=0;
    //cout << "Assign Roles"<<endl;
    /* Goalie doesn't get assigned a goal, and has to be the first role*/
    for (int i = WO_TEAMMATE1; i < WO_TEAMMATE1 + NUM_AGENTS; i++) {
        if (i == ROLE_GOALIE || i == closestPlayer[0])
            continue;
        if (i == worldModel->getUNum())
            agent = worldModel->getMyPosition();
        else
            agent = worldModel->getTeammate(i);
        //t.starts.push_back(std::make_pair(rand()%(n*n), rand()%(n*n)));
        //cout<<i<<" "<<agent<<endl;
        t.starts.push_back(std::make_pair(agent.getX(), agent.getY()));
    }
    //cout<<"targets"<<endl;
    for (int i = 0; i < NUM_OPPONENT; i++) {
        if (markedOpponents[i]) {
            target = worldModel->getOpponent(WO_OPPONENT1 + i);
            t.targets.push_back(std::make_pair(target.getX(), target.getY()));
            noMarkedOpp++;
        }
    }

    for (int i = ROLE_ON_BALL+1; i < ROLE_ON_BALL + NUM_AGENTS-1-noMarkedOpp; i++) {
        target = getPosInFormation(i, worldModel->getBall());
        //cout<<i<<" "<<target<<endl;
        t.targets.push_back(std::make_pair(target.getX(), target.getY()));
    }

    std::vector<Edge> answer = SOLVER(t);
    //cout<<"assignment"<<endl;
    for (int i = 0; i < NUM_AGENTS-2; i++) {
        //cout<<answer[i].second.first+ROLE_ON_BALL << " > " << answer[i].second.second + ROLE_ON_BALL<< "= "<<answer[i].first<<endl;
        // The first noMarkedOpp locations in t.targets array are covering
        if (answer[i].second.second < noMarkedOpp)
            roles[answer[i].second.first] = ROLE_COVERING;
        else
            roles[answer[i].second.first] = answer[i].second.second + ROLE_ON_BALL+1;
        roles_positions[answer[i].second.first] = VecPosition(t.targets[answer[i].second.second].first, t.targets[answer[i].second.second].second);
    }
    // Insert the agent closest to the ball at his location in the roles array
    for(int i = NUM_AGENTS-3 ; i >= (closestPlayer[0]-WO_TEAMMATE2); i--) {
        roles[i+1] = roles[i];
        roles_positions[i+1] = roles_positions[i];
    }
    
    roles[(int)closestPlayer[0]-WO_TEAMMATE2] = ROLE_ON_BALL;
    roles_positions[(int)closestPlayer[0]-WO_TEAMMATE2] = Ball;
    /*
    for(int i=0; i<NUM_AGENTS-1; i++)
        cout<<"agent "<<i<<" role: "<< roles[i]<<endl;
    */
}

VecPosition NaoBehavior::getPosInFormation(int role, VecPosition ball) {
    VecPosition target;
    switch (role) {
        case ROLE_GOALIE:
            target.setX(-HALF_FIELD_X + 0.25);
            target.setY(0);
            break;
        case ROLE_ON_BALL:
            target.setX(ball.getX()-0.1);
            target.setY(ball.getY());
            break;
        case ROLE_FRONT_RIGHT:
            target.setX(ball.getX() - 0.5);
            target.setY(ball.getY() - 2);
            break;
        case ROLE_FRONT_LEFT:
            target.setX(ball.getX() - 0.5);
            target.setY(ball.getY() + 2);
            break;
        case ROLE_FORWARD_CENTER:
            target.setX(HALF_FIELD_X / 2);
            target.setY(0);
            break;
        case ROLE_SUPPORTER:
            target.setX(ball.getX() - 2);
            target.setY(ball.getY());
            break;
        case ROLE_WING_RIGHT:
            target.setX(ball.getX() - 3);
            target.setY(ball.getY() - 2);
            break;
        case ROLE_WING_LEFT:
            target.setX(ball.getX() - 3);
            target.setY(ball.getY() + 2);
            break;
        case ROLE_MIDDLE:
            target.setX(-HALF_FIELD_X / 2);
            target.setY(0);
            break;
        case ROLE_BACK_RIGHT:
        {
            VecPosition goalCenter = VecPosition(-HALF_FIELD_X, 0, 0);
            double slope, intercept, y, distBallToGoal, distAgentToGoal;
            float angle;

            distBallToGoal = ball.getDistanceTo(goalCenter);
            getLineParam(ball, goalCenter, slope, intercept);
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
            VecPosition goalCenter = VecPosition(-HALF_FIELD_X, 0, 0);
            double slope, intercept, y, distBallToGoal, distAgentToGoal;
            float angle;

            distBallToGoal = ball.getDistanceTo(goalCenter);
            getLineParam(ball, goalCenter, slope, intercept);
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
    if (role != ROLE_ON_BALL)
        target = collisionAvoidance(true /*teammate*/, false/*opponent*/, true/*ball*/, 1/*proximity thresh*/, .5/*collision thresh*/, target, true/*keepDistance*/);
    else
        target = collisionAvoidance(true /*teammate*/, false/*opponent*/, false/*ball*/, 1/*proximity thresh*/, .5/*collision thresh*/, target, true/*keepDistance*/);
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
        VecPosition target = getPosInFormation(worldModel->getUNum(), worldModel->getBall());
        beamX = target.getX();
        beamY = target.getY();
    }
    beamAngle = 0;
    memset(markingAgents, -1, sizeof (markingAgents));
    roles[0] = ROLE_GOALIE;
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

    find_closest_player_to_ball();
    //if(closestPlayer[1] < 0.25)
     //   return offense();
    return defense();
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