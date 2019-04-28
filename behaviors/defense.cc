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
#define MARKING_GOAL_DIST_THRESH 15
#define MARKING_MIN_BALL_THRESH 2
#define MARKING_MAX_BALL_THRESH 30

char markedOpponents[NUM_AGENTS];
char markingAgents[NUM_AGENTS];

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

bool NaoBehavior::selectMarkedOpp() {
    VecPosition goalCenter = VecPosition(-HALF_FIELD_X, 0, 0);
    VecPosition opp;
    double shortestDistToBall = DBL_MAX, distToBall;
    int closestOppToBall = -1, noMarkedAgents = 0;

    memset(markedOpponents, 0, sizeof (markedOpponents));
    /* Opponent is close enough to take a shot on goal
        Opponent is not the closest opponent to the ball
        Opponent is not too close to the ball
        Opponent is not too far behind the ball*/
    for (int i = 0; i < NUM_OPPONENT; i++) {
        opp = worldModel->getOpponent(WO_OPPONENT1 + i);
        distToBall = opp.getDistanceTo(worldModel->getBall());
        if (distToBall < shortestDistToBall) {
            shortestDistToBall = distToBall;
            closestOppToBall = i;
        }

        if ((opp.getDistanceTo(goalCenter) < MARKING_GOAL_DIST_THRESH) &&
                (distToBall > MARKING_MIN_BALL_THRESH) && (distToBall < MARKING_MAX_BALL_THRESH)) {
            markedOpponents[i] = 1;
            noMarkedAgents++;
        }
    }

    /* No need to mark closest opponent to Ball. Will be automatically marked by OnBall Agent*/
    if (markedOpponents[closestOppToBall]) {
        markedOpponents[closestOppToBall] = 0;
        noMarkedAgents--;
    }
    for (int i = 0; i < NUM_OPPONENT; i++) {
        opp = worldModel->getOpponent(WO_OPPONENT1 + i);
        if (markedOpponents[i]) {
            //cout << WO worldModel->getUNum() << " opp " << i << " marked" << endl;
            //worldModel->getRVSender()->drawCircle(opp.getX(), opp.getY(), 0.25, RVSender::YELLOW);
        }
    }

    //cout << worldModel->getUNum() - WO_TEAMMATE1 << " no marked agents: " << noMarkedAgents << endl;
    return noMarkedAgents > 0;
}

void NaoBehavior::selectMarkingAgents() {
    double currentAgentDist, closestAgentDist;
    int closestAgentId;
    VecPosition opp, agent;
    memset(markingAgents, -1, sizeof (markingAgents));
    for (int j = (ROLE_ON_BALL - WO_TEAMMATE1) + 1; j < NUM_AGENTS; j++) {
        if (worldModel->getUNum() == (WO_TEAMMATE1 + j))
            agent = worldModel->getMyPosition();
        else
            agent = worldModel->getTeammate(WO_TEAMMATE1 + j);

        //cout << worldModel->getUNum() - WO_TEAMMATE1 << " agent " << j << agent << endl;
    }
    for (int i = 0; i < NUM_OPPONENT; i++) {
        if (markedOpponents[i] == 1) {
            closestAgentDist = INT_MAX;
            closestAgentId = -1;
            opp = worldModel->getOpponent(WO_OPPONENT1 + i);

            //cout << worldModel->getUNum()-WO_TEAMMATE1 << " opp "<<i<<opp<<endl;
            for (int j = (ROLE_ON_BALL - WO_TEAMMATE1) + 1; j < NUM_AGENTS; j++) {
                /* If the agent is not marking another opponent*/
                if (markingAgents[j] == -1) {
                    if (worldModel->getUNum() == (WO_TEAMMATE1 + j))
                        agent = worldModel->getMyPosition();
                    else
                        agent = worldModel->getTeammate(WO_TEAMMATE1 + j);

                    currentAgentDist = agent.getDistanceTo(opp);
                    //cout << worldModel->getUNum()-WO_TEAMMATE1 << " agent "<< j << " distance "<<currentAgentDist<<" closest "<<closestAgentDist<< endl;
                    if (currentAgentDist < closestAgentDist) {
                        closestAgentDist = currentAgentDist;
                        closestAgentId = j;
                    }
                } else {
                    //cout << worldModel->getUNum()-WO_TEAMMATE1 << " agent "<< j << " already marking"<<endl;
                }
            }
            if (closestAgentId >= 0) {
                markingAgents[closestAgentId] = i;
                //cout << worldModel->getUNum()-WO_TEAMMATE1 << " agent "<<closestAgentId<<" marking opp: "<<i<<endl;
            }
        }
    }
    //cout << worldModel->getUNum()-WO_TEAMMATE1 << " *********************** "<<endl<<endl;
}

SkillType NaoBehavior::backAction() {
    VecPosition Ball = worldModel->getBall();
    VecPosition goalCenter = VecPosition(-HALF_FIELD_X, 0, 0);
    VecPosition target;
    double slope, intercept, y, distBallToGoal, distAgentToGoal;
    float angle;

    getLineParam(Ball, goalCenter, slope, intercept);
    angle = atan(slope);
    distBallToGoal = Ball.getDistanceTo(goalCenter);


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
    int role;
    VecPosition target;

    if (worldModel->getUNum() == ROLE_GOALIE)
        return goalieAction();

    assignRoles();
    role = roles[worldModel->getUNum() - ROLE_ON_BALL];
    if (role == ROLE_BACK_LEFT || role == ROLE_BACK_RIGHT)
        return backAction();

    target = getPosInFormation(role, worldModel->getBall());
    target.setZ(me.getZ());
    
    if (me.getDistanceTo(target) < .25) {
        // Close enough to desired position and orientation so just stand
        return SKILL_STAND;
    } else {
        // Move toward target location
        return goToTarget(target);
    }

    int markingOppIdx;
#if 0
    if ((worldModel->getUNum() != ROLE_GOALIE) && (worldModel->getUNum() != ROLE_ON_BALL)) {
        if (selectMarkedOpp()) {
            selectMarkingAgents();

            markingOppIdx = markingAgents[worldModel->getUNum() - WO_TEAMMATE1];
            //cout << worldModel->getUNum()-WO_TEAMMATE1 << " assigned " << markingOppIdx << endl;
            if (markingOppIdx != -1) {
                VecPosition target = worldModel->getOpponent(WO_OPPONENT1 + markingOppIdx);
#ifdef ENABLE_DRAWINGS
                worldModel->getRVSender()->drawLine(worldModel->getMyPosition().getX(), worldModel->getMyPosition().getY(),
                        target.getX(), target.getY(), RVSender::YELLOW);
                worldModel->getRVSender()->drawCircle("player " + markingOppIdx, target.getX(), target.getY(), 0.25, RVSender::YELLOW);
#endif

                double angle;
                double distance;
                target.setX(target.getX() - 0.1);
                getTargetDistanceAndAngle(target, distance, angle);
                //cout << worldModel->getUNum()-WO_TEAMMATE1 << " agent " << worldModel->getUNum() - WO_TEAMMATE1 << " marking opp " << markingOppIdx << endl;
                return goToTargetRelative(worldModel->g2l(target), angle, 2);
                //return goToTarget(target);
                //return SKILL_STAND();
            }

        }
    }
#endif

#if 0
    switch (worldModel->getUNum()) {
        case ROLE_GOALIE:
            return goalieAction();
            break;
        case ROLE_BACK_LEFT:
        case ROLE_BACK_RIGHT:
            return backAction();
            break;
        default:
            VecPosition target = getPosInFormation(worldModel->getUNum(), worldModel->getBall());
            if (me.getDistanceTo(target) < .25) {
                // Close enough to desired position and orientation so just stand
                return SKILL_STAND;
            } else {
                // Move toward target location
                return goToTarget(target);
            }
            break;
    }
#endif
}

