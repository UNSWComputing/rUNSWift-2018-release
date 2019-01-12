/**
 * Walk2014Generator.cpp
 * BH 18th Jan 2014
 * The period of each foot-step is set by T. T generates the forcing function by alternatively lifting each foot
 * Control:
 * The change of support foot is driven by the ZMP switching sign, but must be > T/2. If > 3*T we switch to try revive
 * The front-back sway is controlled by ankle tilts proportional to the GyroY
 * The user specifies forward(m), left(m), turn(radians) to activate the walk.
 * If these values are all zero the robot stands with the motors turned off.
 * The CoM is moved forward when walking to position it more over the center of the foot
 */

#include "motion/generator/Walk2014Generator.hpp"
#include <cmath>
#include <cstdio>
#include "utils/angles.hpp"
#include "utils/body.hpp"
#include "utils/Logger.hpp"
#include "utils/basic_maths.hpp"
#include "utils/speech.hpp"
#include <algorithm>

//#define WRITE_LEG_JOINT_TEMP_TO_FILE 1;

#define KICK_STEP_HEIGHT 0.065  // how far to lift kicking foot
#define TURN_THRESHOLD DEG2RAD(25) // min angle for triggering turn step
#define TURN_PERIOD 0.75 // time to perform turn step
#define TURN_STEP_HEIGHT 0.024 // how far to lift turning foot
#define TURN_LEAN DEG2RAD(20.25) // sideways lean while turning
#define TURN_SCALE 1.15 // how much to scale up turn
#define EPSILON 0.01       //10 mm
#define TURN_EPSILON 0.05  //2.8 degrees

//mm value of safety barrier
#define FOOT_AVOID_RADIUS 75
#define TURN_CLIPPING 0.10 //radians
#define NINETY 1.51
#define FORTY_FIVE 1.51 / 2
#define TT_DEGREE FORTY_FIVE / 2

using boost::program_options::variables_map;
using namespace std;
using namespace Joints;
using namespace Sensors;

const float MM_PER_M = 1000.0;             // number of millimeters in one meter
const float CROUCH_STAND_PERIOD = 0.5;              // time in seconds to crouch
const float COM_OFFSET = 0.01; // center of mass offset in x direction in meters
const float FORWARD_CHANGE = 0.0025; // was 0.08. max change of 100mm/sec at each leg change to ratchet up/down
const float LEFT_CHANGE = 0.003;
const float TURN_CHANGE = 0.2;
const float STAND_HIP_HEIGHT = 0.248; // for tall power saving stand, matches INITIAL action command
const float KNEE_PITCH_RANGE = DEG2RAD(60); // the knee pitch range from standing to crouching
const float BASE_WALK_PERIOD = .25;    //.23  - .25            // seconds to walk one step, ie 1/2 walk cycle
const float WALK_HIP_HEIGHT = .23; // Walk hip height - seems to work from .2 to .235
#ifdef SIMULATION       // simulated robot falls over at higher walk speeds (TODO try to debug this problem)
const float MAX_FORWARD = .25;
#else
const float MAX_FORWARD = .3;                              // meters
#endif
const float MAX_LEFT = .2;                                 // meters
const float MAX_TURN = .58;                                // radians
const float BASE_LEG_LIFT = 0.012;                         // meters

float ellipsoidClampWalk(float &forward, float &left, float &turn);
float evaluateWalkVolume(float x, float y, float z);

Walk2014Generator::Walk2014Generator(Blackboard *bb) :
        t(0.0f), z(0.0f), PI(3.1415927), blackboard(bb) {
    initialise();
    llog(INFO) << "Walk2014Generator constructed" << std::endl;

}

Walk2014Generator::~Walk2014Generator() {
    llog(INFO) << "Walk2014Generator destroyed" << std::endl;
}

void Walk2014Generator::initialise() {
    llog(INFO) << "Walk2014 initializing" << endl;
    dt = 0.01;                                           // 100 Hz motion thread
    t = 0.0;                                   // initialise timers (in seconds)
    timer = 0.0;                            // timer to crouch to walking height
    globalTime = 0;                          // use for diagnostic purposes only
    T = BASE_WALK_PERIOD; // seconds - the period of one step in a two step walk cycle
    stopping = false;                         // legacy code for stopping robot?
    stopped = true;                            // legacy code for stopped robot?
    leftL = leftR = lastLeft = left = 0.0; // Side-step for left, right foot, and (last) left command in meters
    turnRL = turnRL0 = lastTurn = turn = 0.0;       // Initial turn variables for feet
    forwardL = forwardR = 0.0; // forward step per for left and right foot in meters
    forwardR0 = forwardL0 = 0; // initial last positions for left and right feet keep constant during next walk step
    forward = lastForward = 0.0;           // Current and previous forward value
    shoulderPitchL = shoulderPitchR = 0;              // arm swing while walking
    shoulderRollL = shoulderRollR = 0;                   // arm roll during kick
    hiph = hiph0 = STAND_HIP_HEIGHT; // make robot stand initially based on Stand command
    foothL = foothR = 0;         // robots feet are both on the ground initially
    thigh = Limbs::ThighLength / MM_PER_M;             // thigh length in meters
    tibia = Limbs::TibiaLength / MM_PER_M;             // tibia length in meters
    ankle = Limbs::FootHeight / MM_PER_M;        // height of ankle above ground
    nextFootSwitchT = 0.0; // next time-point to switch support foot (in seconds)
    stiffness = kneeStiffness = ankleStiffness = 0.9; // initial motor stiffness
    currentVolume = 0;                      // initial volume
    walk2014Option = NONE;                           // initial walk 2014 option
    walkState = NOT_WALKING;                               // initial walkState
    supportFoothasChanged = false;       // triggers support foot change actions
    comOffset = 0; // Center of Mass offset in sagittal plane used to spread weight along feet in x-dir
    prevTurn = prevForwardL = prevForwardR = 0;            // odometry
    prevLeftL = prevLeftR = 0;                             // odometry
    exactStepsRequested = false; // turns off ratcheting as requested by WalkEnginePreProcessor for kick

    // Balance control
    filteredGyroX = 0;
    filteredGyroY = 0;
    filteredAngleY = 0;
    sagittalBalanceAdjustment = 0;
    coronalBalanceAdjustment = 0;
    sagittalHipBalanceAdjustment = 0;

    // Gyro PD controller
    KpGyro = 0.07;                // Proportional gain
    KdGyro = 0.0003934;           // Derivative gain
    preErrorGyro = 0;             // Previous tick gyro error

    // Angle PID controller
    KpAngle = 0.2;                // Proportional gain
    KiAngle = 0.05;               // Integral gain
    KdAngle = 0.008;              // Derivative gain
    angleError = 0;               // Current angle error
    angleErrorSum = 0;            // Error sum
    preErrorAngle = 0;            // Previous tick angle error

    // Kick specific
    kickT = 0;
    rock = 0;
    kneePitchL = kneePitchR = lastKneePitch = 0;
    anklePitchL = anklePitchR = 0;
    lastKickForward = 0;
    lastSide = 0;
    lastKickTime = T;
    dynamicSide = 0.0f;
    turnAngle = 0;
    lastKickTurn = 0;
    motionOdometry.reset();
    kneePitchEnd = 0;
    anklePitchStart = 0;
    anklePitchEnd = 0;
    swingDelayFactor = 0;
    holdAnkle = false;
    hipBalance = false;

    lElbowYawCounter = 0;
    rElbowYawCounter = 0;

    // Toe lift
    leftToeLift = false;        // true if left toe to be lifted during left phase
    rightToeLift = false;       // true if right toe
    toeLiftAngle = 0;

    // Kick parameter constants (@ijnek: Commenting this out because kick_fast is actually uninitialised at this point, and kick_fast doesnt work.)
    // if (kick_fast) {
    //     shiftPeriod = 1.8;
    //     shiftEndPeriod = 1.7;
    //     backPhase = 0.32;
    //     kickPhase = 0.1;
    //     throughPhase = 0.15;
    //     endPhase = 0.0;
    //     shoulderRollAmpDivisor = 1.5;
    //     kickLean = 23.0;
    // } else {
    //     shiftPeriod = 2.6;
    //     shiftEndPeriod = 2.5;
    //     backPhase = 0.75;
    //     kickPhase = 0.2;
    //     throughPhase = 0.3;
    //     endPhase = 0.0;
    //     shoulderRollAmpDivisor = 2.5;
    //     kickLean = 20.9;
    // }

    shiftPeriod = 2.6;
    shiftEndPeriod = 2.5;
    backPhase = 0.75;
    kickPhase = 0.2;
    throughPhase = 0.3; 
    endPhase = 0.0;
    shoulderRollAmpDivisor = 2.5;
    kickLean = 20.9;

    // Each robot has a slightly different lean angle that works. This is dealt through individual lean offsets.
    kickLean += kickLeanOffset;
}

bool intersects(FootInfo &f, Point &p) {
	const int toeRadius = 45;

    float closestX = std::max(p.x(), std::min(f.robotBounds.a.x(), f.robotBounds.b.x()));
    float closestY = std::max(p.y(), std::min(f.robotBounds.a.y(), f.robotBounds.b.y()));

    float distanceX = p.x() - closestX;
    float distanceY = p.y() - closestY;

    float distanceSquared = (distanceX * distanceX) + (distanceY * distanceY);

    return distanceSquared < (toeRadius * toeRadius);
}

void Walk2014Generator::avoidFeet(float &forward, float &left, float &turn, BodyModel& bodyModel) {
    //Given a set of clamped walk parameters check if any of the feet would result in crash
    const int footOffset = 50;
	const int toeOffset = 40;
    vector<FootInfo> boxes = readFrom(vision, feet_boxes);

    bool clipped = false;
    if (forward != 0 || left != 0 || turn != 0) {
        Point toeFall;

        if (bodyModel.isLeftPhase) {
            toeFall = Point(forward + toeOffset, left + footOffset);
        } else {
            toeFall = Point(forward + toeOffset, left - footOffset);
        }
        while (true) {
            bool inBox = false;
            for (vector<FootInfo>::iterator it = boxes.begin(); it != boxes.end(); ++it) {
                if (intersects(*it, toeFall)) {
                    inBox = true;
                    break;
                }
            }

            if (inBox) {
                forward *= 0.7;
                left *= 0.7;
                clipped = true;
                if (bodyModel.isLeftPhase) {
                    toeFall = Point(forward + toeOffset, left + footOffset);
                } else {
                    toeFall = Point(forward + toeOffset, left - footOffset);
                }
                if (left <= 5 && forward <= 5) {
                    break;
                }
            } else {
                break;
            }
            //need to chop forward and left components from the walk to be outside of the box
        }
    }
}

float ellipsoidClampWalk(float &forward, float &left, float &turn, float speed) {
    const float MIN_SPEED = 0.0f;
    const float MAX_SPEED = 1.0f;
    speed = crop(speed, MIN_SPEED, MAX_SPEED);

    // limit max to 66-100% depending on speed
    float M_FORWARD = MAX_FORWARD * 0.66 + MAX_FORWARD * 0.34 * speed;
    float M_LEFT = MAX_LEFT * 0.66 + MAX_LEFT * 0.34 * speed;
    float M_TURN = MAX_TURN * 0.66 + MAX_TURN * 0.34 * speed;

    float clampedForward = crop(forward, -M_FORWARD, M_FORWARD);
    float clampedLeft = crop(left, -M_LEFT, M_LEFT);
    float clampedTurn = crop(turn, -M_TURN, M_TURN);

    // Values in range [-1..1]
    float forwardAmount = clampedForward / M_FORWARD;
    float leftAmount = clampedLeft / M_LEFT;
    float turnAmount = clampedTurn / M_TURN;

    float x = fabs(forwardAmount);
    float y = fabs(leftAmount);
    float z = fabs(turnAmount);

    // see if the point we are given is already inside the allowed walk params volume
    if (evaluateWalkVolume(x, y, z) > 1.0) {
        float scale = 0.5;
        float high = 1.0;
        float low = 0.0;

        // This is basically a binary search to find the point on the surface.
        for (unsigned i = 0; i < 10; i++) {
            // give priority to turn. keep it the same
            x = fabs(forwardAmount) * scale;
            y = fabs(leftAmount) * scale;

            if (evaluateWalkVolume(x, y, z) > 1.0) {
                float newScale = (scale + low) / 2.0;
                high = scale;
                scale = newScale;
            } else {
                float newScale = (scale + high) / 2.0;
                low = scale;
                scale = newScale;
            }
        }

        forwardAmount *= scale;
        leftAmount *= scale;
    }

    forward = M_FORWARD * forwardAmount;
    left = M_LEFT * leftAmount;
    turn = clampedTurn;
    float volume = evaluateWalkVolume(x, y, z);
    return volume;
}

// x = forward, y = left, z = turn
float evaluateWalkVolume(float x, float y, float z) {
    // this affects the relationship between forward and left.
    float e = 1.05;

    // lower value allows turn to be higher with a high forward/left, higher values dont allow a high turn
    float n = 1;

    float r = 2.0 / e;
    float t = 2.0 / n;

    return pow(pow(x, r) + pow(y, r), (t / r)) + pow(z, t);
}

// Stiffness required by the knee to keep stability
// Equation was found by finding the lowest stiffness that was stable at a continous requested speed
// Relate each speed to the corresponding volume and curve fit
float Walk2014Generator::calculateKneeStiffness(float volume) {
    float stiffness = -0.42 * pow(volume, 2) + volume + 0.395;
    stiffness = ceilf(MIN(1,MAX(0.4,stiffness)) * 100) / 100;    // Min and max (2 dp) required stiffness
    return stiffness;
}
// Stiffness required by the ankle to keep stability
// Equation was found by setting the lowest stiffness that was stable at a continous requested speed
// Relate each speed to the corresponding volume and curve fit
float Walk2014Generator::calculateAnkleStiffness(float volume) {
    float stiffness = -0.32 * pow(volume, 2) + 0.76 * volume + 0.56;
    stiffness = ceilf(MIN(1,MAX(0.55,stiffness)) * 100) / 100;    // Min and max (2 dp) required stiffness
    return stiffness;
}

JointValues Walk2014Generator::makeJoints(ActionCommand::All* request, Odometry* odometry, const SensorValues &sensors, BodyModel &bodyModel, float ballX, float ballY) {
    // 0. The very first time walk is called, the previous stand height could have been anything, so make sure we interpolate from that    
    if (walk2014Option == NONE) {
        // Calculate the current hip height by checking how bent the knee is
        hiph = sensors.joints.angles[LKneePitch] / KNEE_PITCH_RANGE * (WALK_HIP_HEIGHT - STAND_HIP_HEIGHT) + STAND_HIP_HEIGHT;
    }

    // 1. Read in new walk values (forward, left, turn, power) only at the start of a walk step cycle, ie when t = 0
    if (t == 0 && walk2014Option != STEP) {
        active = request->body;
        forward = (float) active.forward / MM_PER_M;    // in meters
        left = (float) active.left / MM_PER_M;       // in meters
        turn = active.turn;                       // in radians
        power = active.power;                     // controls stiffness when standing and kicking when walking*
        bend = active.bend;                       // knee-bend parameter
        speed = active.speed;                     // controls speed of walk, distinguishes Jab kick
        foot = active.foot;                       // kicking foot
        isFast = active.isFast;
        useShuffle = active.useShuffle;
        if (stopping) {                                   // not used at present
        } else {
            stopped = false;                                  // (re)activate
        }

        if (forward == 0 and left == 0 and turn == 0 and power == 0)
            bend = 0;

        // limit the speed of walk when overheating, can be commented out for serious games
        // for (int i = 0; i < Joints::NUMBER_OF_JOINTS; ++i) {
        //     float temp = sensors.joints.temperatures[i];
        //     if (temp > 70)
        //         speed = min(0.5f, speed);
        //     if (temp > 75)
        //         speed = 0;
        // }
    }

#ifdef WRITE_LEG_JOINT_TEMP_TO_FILE
    // Record the temperatures of the leg joints ever 5 seconds in the file temp.txt
    static int counter = 0;
    if(counter == 500){
        FILE *file = fopen("/home/nao/data/temp.txt", "a");        
        for (int i = 8; i < 19; ++i) {                          //Only want leg joints
            float temp = sensors.joints.temperatures[i];
            fprintf(file, "%lf ", temp);
            //std::cout << " " << i << " - " << temp;
        }
        fprintf(file, "\n");
        //std::cout << "\n";
        fclose(file);
        counter = 0;
    }
    counter++;
#endif //WRITE_LEG_JOINT_TEMP_TO_FILE

    forward = (float) active.forward / MM_PER_M;    // in meters
    left = (float) active.left / MM_PER_M;       // in meters
    turn = active.turn;                       // in radians

    // Scale back values to try to ensure stability.
    if (!exactStepsRequested) {
        currentVolume = ellipsoidClampWalk(forward, left, turn, speed);
    }
    float f = forward * MM_PER_M;
    float l = left * MM_PER_M;
    // avoidFeet(f, l, turn, bodyModel);
    forward = f / MM_PER_M;
    left = l / MM_PER_M;

    // Modify T when sidestepping
    T = BASE_WALK_PERIOD + 0.1 * abs(left) / MAX_LEFT;

    // ratchet forward by FORWARD_CHANGE
    if (!exactStepsRequested) {
        if (abs(forward - lastForward) > FORWARD_CHANGE) {
            forward = lastForward + (forward - lastForward) / abs(forward - lastForward) * FORWARD_CHANGE;
        }
        if (abs(left - lastLeft) > LEFT_CHANGE) {
            left = lastLeft + (left - lastLeft) / abs(left - lastLeft) * LEFT_CHANGE;
        }
        if (abs(turn - lastTurn) > TURN_CHANGE) {
            turn = lastTurn + (turn - lastTurn) / abs(turn - lastTurn) * TURN_CHANGE;
        }
    }
    lastForward = forward; // back up old value in m/s
    lastLeft = left; // used to detect when the left walk parameter changes sign
    lastTurn = turn;

    // 1.6 Walk Calibration
    // The definition of forward, left and turn is the actual distance/angle traveled in one second
    // One walk-cycle consists of two Phases, a left phase (left swing foot) and a right phase (right swing foot)

    if (!exactStepsRequested) {
        forward *= 2 * T; // theoretical calibration. 2 - because there are two steps per walk cycle
        left *= 2 * T;
        turn *= 2 * T;
        // linear calibration to achieve actual performance ie turn in action command achieves turn/sec in radians on the real robot
        forward *= 1.0;
        left *= 0.82;
        turn *= 1.43;
    }
    turn *= -1;   // reverses sign

    // 2. Update timer
    t += dt;
    globalTime += dt;
    lastKickTime += dt;

    // 3. Determine Walk2014 Option
    if (request->body.actionType != ActionCommand::Body::KICK && kickT > 0) {
        if (canAbortKick()) {
            // Finish transition out if in the middle of a kick by skipping to the end phase
            kickT = backPhase + kickPhase + throughPhase;
        }
    } else if (active.actionType == ActionCommand::Body::KICK) {
        // This makes sure that the action type gets set back to walk just after a kick is finished.
        // If we don't leave enough time for this to happen, motion moves back into a kick before behaviour
        // can change its mind.
        if (lastKickTime < 4 * T) {
            request->body.actionType = ActionCommand::Body::WALK;
        } else if (abs(hiph - WALK_HIP_HEIGHT) < .0001) { // make sure we retain the designated walking height
            if (walk2014Option == WALK || walk2014Option == STEP) {
                // make sure walk is in neutral stance before kicking, L is symmetrical to R
                if (fabs(forwardL) < EPSILON && fabs(leftL) < EPSILON && fabs(turnRL) < TURN_EPSILON && t == dt) {
                    // Assuming already at t = 0 from active getting set to kick
                    // Any new settings the first time walk2014Option==KICK go here

                    // Calculate if turn step is required before kicking
                    if (walk2014Option == WALK && fabs(active.kickDirection) >= TURN_THRESHOLD) {
                        // Halve the kick direction since HypL and HypR are connected
                        turnAngle = MIN(DEG2RAD(90), fabs(active.kickDirection)) / 2; // Safe max value for Hyp is ~45
                        if ((active.foot == ActionCommand::Body::LEFT && !bodyModel.isLeftPhase) || (active.foot != ActionCommand::Body::LEFT && bodyModel.isLeftPhase)) {
                            walk2014Option = STEP;
                        }
                        // If no turn step is required, kick
                    } else if (fabs(active.kickDirection) < TURN_THRESHOLD) {
                        prepKick(active.foot == ActionCommand::Body::LEFT, bodyModel);
                    }
                }
            } else if (walk2014Option != KICK) {
                // Calculate if turn is required before kicking
                if (fabs(active.kickDirection) >= TURN_THRESHOLD) {
                    turnAngle = MIN(DEG2RAD(90), fabs(active.kickDirection)) / 2; // Safe max value for Hyp is ~45
                    walk2014Option = STEP;
                } else {
                    prepKick(active.foot == ActionCommand::Body::LEFT, bodyModel);
                }
            }
        } else {                                      // hiph not= walkHipHeight
            if (walk2014Option != CROUCH) { // robot starts crouching to walking height
                hiph0 = hiph;
                timer = 0;
            }
            walk2014Option = CROUCH;                       // continue crouching
        }

    } // end Kick test
    else if (walk2014Option == WALK and walkState != NOT_WALKING) { // we are in the process of walking
        if (forward == 0 and left == 0 and turn == 0) { // request to stop walking
            walkState = STOPPING;
        }
    } else if (bend == 0) { // if we are not walking and wish to stand and power off
        if (abs(hiph - STAND_HIP_HEIGHT) < .0001) { // and robot has reached stand height
            walk2014Option = STAND;                       // (keep) standing
        } else {                                  // if hiph not= standHipHeight
            if (walk2014Option != STANDUP) {
                hiph0 = hiph;
                timer = 0;
            }
            walk2014Option = STANDUP;                     // stand up first
        }
    } else if (forward == 0 and left == 0 and turn == 0 and bend == 1) { // not walking, but ready to go again, ie don't stand up
        if (abs(hiph - WALK_HIP_HEIGHT) < .0001) {
            walk2014Option = READY;
        } else {
            if (walk2014Option != CROUCH) { // robot starts crouching to walking height
                hiph0 = hiph;
                timer = 0;
            }
            walk2014Option = CROUCH;                       // continue crouching
        }
    } else {                             // if some walk parameters are non-zero
        if (abs(hiph - WALK_HIP_HEIGHT) < .0001) { // and we are at the designated walking height
            if (walk2014Option != WALK) {
                // Any new settings the first time walk2014Option==WALK go here (just for testing the walk)
                walkState = STARTING;
                nextFootSwitchT = T;
            }
            walk2014Option = WALK;                          // (keep) walking
        } else {                                      // hiph not= walkHipHeight
            if (walk2014Option != CROUCH) { // robot starts crouching to walking height
                hiph0 = hiph;
                timer = 0;
            }
            walk2014Option = CROUCH;                       // continue crouching
        }
    }

    // 4. Execute Walk2014 Option
    if (walk2014Option == STAND) { // Place CoM over ankle and turn set power to motors
        hiph = STAND_HIP_HEIGHT;
        forward = left = turn = 0;
        t = nextFootSwitchT = 0;
        stiffness = power;
        if (stiffness < 0.2)
            stiffness = 0.2;
        comOffset = 0;
    } else if (walk2014Option == STANDUP) {
        hiph = hiph0 + (STAND_HIP_HEIGHT - hiph0) * parabolicStep(timer, CROUCH_STAND_PERIOD, 0);
        forward = left = turn = 0;
        comOffset -= 0.02 * comOffset; // reduce offset to zero to allow stiffness to be turned down
        stiffness = 1;
        t = nextFootSwitchT = 0;
        timer += dt;
    } else if (walk2014Option == CROUCH) {
        forward = left = turn = 0;
        stiffness = 1;
        hiph = hiph0 + (WALK_HIP_HEIGHT - hiph0) * parabolicStep(timer, CROUCH_STAND_PERIOD, 0);
        comOffset = COM_OFFSET * parabolicStep(timer, CROUCH_STAND_PERIOD, 0); // move comOffset to 0.01 meters when walking
        t = nextFootSwitchT = 0;
        timer += dt;                                        // inc. option timer
    } else if (walk2014Option == WALK) {
        if(!exactStepsRequested){
            kneeStiffness = calculateKneeStiffness(currentVolume);
            ankleStiffness = calculateAnkleStiffness(currentVolume);
        }
        stiffness = 1;
    } else if (walk2014Option == KICK) {
        stiffness = 1;
    } else if (walk2014Option == STEP) {
        stiffness = 1;
        nextFootSwitchT = TURN_PERIOD;
    }
    if (walk2014Option == READY) {
        forward = left = turn = 0;
        stiffness = power;
        if (stiffness < 0.4)
            stiffness = 0.4;  // need enough stiffness to keep crouching posture
        t = nextFootSwitchT = 0;
    }

    // 5. Determine walk variables throughout the walk step phase
    if (walk2014Option == WALK and nextFootSwitchT > 0) {
        // 5.1 Calculate the height to lift each swing foot
        float maxFootHeight = BASE_LEG_LIFT + abs(forward) * 0.01 + abs(left) * 0.02;
        if (useShuffle){
            maxFootHeight *= 0.7;  // lower step height when shuffling (ie. close to obstacles)
        }
        float varfootHeight = maxFootHeight * parabolicReturnMod(t / nextFootSwitchT); // 0.012 lift of swing foot from ground
        // 5.2 When walking in an arc, the outside foot needs to travel further than the inside one - void
        // 5.3L Calculate intra-walkphase forward, left and turn at time-step dt, for left swing foot
        if (bodyModel.isLeftPhase) {             // if the support foot is right
            if (weightHasShifted) {
                // 5.3.1L forward (the / by 4 is because the CoM moves as well and forwardL is wrt the CoM
                forwardR = forwardR0 + ((forward) / 4 - forwardR0) * linearStep(t, nextFootSwitchT);
                forwardL = forwardL0 + parabolicStep(t, nextFootSwitchT, 0) * (-(forward) / 4 - forwardL0); // swing-foot follow-through
                // 5.3.2L Jab kick with left foot
                if (foot == ActionCommand::Body::LEFT and speed == 0 and isFast) {
                    forwardL -= abs(power) * MAX_FORWARD * 2 * T/4 * parabolicReturn(t/nextFootSwitchT);
                }
                // 5.3.3L Determine how much to lean from side to side - removed
                // 5.3.4L left
                if (left > 0) {
                    leftR = leftAngle() / 2; // share left between both feet, hence /2
                    // if changing direction of left command, cancel swingAngle out of next left step
                    if (lastLeft * left < 0)
                        leftR -= swingAngle * (1 - parabolicStep(t, nextFootSwitchT, 0.1));
                    leftL = -leftR;
                } else {
                    leftL = swingAngle * (1 - parabolicStep(t, nextFootSwitchT, 0.0));
                    leftR = -leftL;
                }
                // 5.3.5L turn
                if (turn < 0)
                    turnRL = turnRL0 + (-.67 * turn - turnRL0) * parabolicStep(t, nextFootSwitchT, 0.0);
                else
                    turnRL = turnRL0 + (-.33 * turn - turnRL0) * parabolicStep(t, nextFootSwitchT, 0.0); //turn back to restore previous turn angle
            }
            // 5.3.6L determine how high to lift the swing foot off the ground
            foothL = varfootHeight;                      // lift left swing foot
            foothR = 0;                             // do not lift support foot;
        }
        // 5.3R Calculate intra-walkphase forward, left and turn at time-step dt, for right swing foot
        if (not bodyModel.isLeftPhase) {          // if the support foot is left
            if (weightHasShifted) {
                // 5.3.1R forward
                forwardL = forwardL0 + ((forward) / 4 - forwardL0) * linearStep(t, nextFootSwitchT);
                forwardR = forwardR0 + parabolicStep(t, nextFootSwitchT, 0) * (-(forward) / 4 - forwardR0); // swing-foot follow-through
                // 5.3.2R Jab-Kick with right foot
                if (foot == ActionCommand::Body::RIGHT and speed == 0 and isFast) {
                    forwardR -= abs(power) * MAX_FORWARD * 2 * T/4 * parabolicReturn(t/nextFootSwitchT);
                }
                // 5.3.3R lean - not used
                // 5.3.4R left
                if (left < 0) {
                    leftL = leftAngle() / 2; // divide by 2 to share left between both feet
                    // if changing direction of left command, cancel swingAngle out of next left step
                    if (lastLeft * left < 0)
                        leftL -= swingAngle * (1 - parabolicStep(t, nextFootSwitchT, 0.1));
                    leftR = -leftL;
                } else {
                    leftR = swingAngle * (1 - parabolicStep(t, nextFootSwitchT, 0.0));
                    leftL = -leftR;
                }
                // 5.3.5R turn
                if (turn < 0)
                    turnRL = turnRL0 + (.33 * turn - turnRL0) * parabolicStep(t, nextFootSwitchT, 0.0);
                else
                    turnRL = turnRL0 + (.67 * turn - turnRL0) * parabolicStep(t, nextFootSwitchT, 0.0);
                // 5.3.6R Foot height
            }
            foothR = varfootHeight;
            foothL = 0;
        }
        // 5.4 Special conditions when priming the walk
        if (walkState == STARTING) {
            turnRL = 0; // don't turn on start of rocking - may relax this in future
            foothL /= 3.5; // reduce max lift due to short duration - may need to adjust this later
            foothR /= 3.5;                                  // "
            leftR = leftL = 0; // don't try to step left on starting and stopping
            forwardL = forwardR = 0;           // don't move forward or backward
            lastForward = 0.0;
            lastLeft = 0.0;
            lastTurn = 0.0;
        }
        // 5.5 "natural" arm swing while walking/kicking to counterbalance foot swing
        shoulderPitchR = -forwardR * 6; //10;                     // forwardR is in meters, 10 is an arbitrary scale factor to match previous walk
        shoulderPitchL = -forwardL * 6;                        //10;
    } else if (walk2014Option == KICK) { // Kicking

        // HARDCODED TO LEFT FOOT FOR COMPETITION - SHOULD TAKE THIS OUT @ijnek

        ballX = MIN(300, MAX(160, ballX));
        ballY = MIN(200, MAX(ballY, 30));
        float kickLeanMod = kickLean + ((ballY-30)/(200-30))*1.5;
        makeForwardKickJoints(kickLeanMod, KICK_STEP_HEIGHT, foothL, forwardL, leftL, kneePitchL, shoulderRollL, anklePitchL, ballX, ballY, request);
        // if (active.foot == ActionCommand::Body::LEFT) {
        //     ballX = MIN(300, MAX(160, ballX));
        //     ballY = MIN(200, MAX(ballY, 30));
        //     makeForwardKickJoints(kickLean, KICK_STEP_HEIGHT, foothL, forwardL, leftL, kneePitchL, shoulderRollL, anklePitchL, ballX, ballY, request);
        //     leftR = leftL * 0.1; // Balance slightly more over support foot if need be
        // } else { // with added adjustments for right side
        //     ballX = MIN(300, MAX(160, ballX));
        //     ballY = -MIN(200, MAX(-ballY, 30));
        //     makeForwardKickJoints(-kickLean + 1.5, KICK_STEP_HEIGHT + 0.005, foothR, forwardR, leftR, kneePitchR, shoulderRollR, anklePitchR, ballX, ballY, request);
        //     leftR = -leftR; // switch signs for right side
        //     leftL = leftR * 0.12;
        // }
    } else if (walk2014Option == STEP) { // Turn step
        float footHeight = TURN_STEP_HEIGHT * parabolicReturn(t / nextFootSwitchT);
        if (active.foot != ActionCommand::Body::LEFT) { // if the support foot is right while turning
            foothL = footHeight;                         // lift left swing foot
            foothR = 0;                             // do not lift support foot;
            rock = TURN_LEAN * parabolicReturn(t / nextFootSwitchT);
        } else {                    // if the support foot is left while turning
            foothR = footHeight;                        // lift right swing foot
            foothL = 0;                             // do not lift support foot;
            rock = -TURN_LEAN * parabolicReturn(t / nextFootSwitchT);
        }
        // Interpolate turn step over turnT with dead period to avoid dragging against the ground
        turnRL = turnAngle * TURN_SCALE * parabolicStep(t, nextFootSwitchT, 0.15);
        // Once the turn step has pretty much finished, readjust the kick direction
        if (t >= nextFootSwitchT - dt) {
            if (active.kickDirection >= TURN_THRESHOLD) {
                active.kickDirection -= (turnAngle * 2);
            } else if (active.kickDirection <= -TURN_THRESHOLD) {
                active.kickDirection += (turnAngle * 2);
            }
        }
        lastKickTurn = turnRL;
    } else { // When walk option is not WALK or KICK
        foothL = foothR = 0;
    }

    // 6. Changing Support Foot. Note bodyModel.isLeftPhase means left foot is swing foot.
    //    t>0.75*T tries to avoid bounce, especially when side-stepping
    //    lastZMPL*ZMPL<0.0 indicates that support foot has changed
    //    t>3*T tires to get out of "stuck" situations
    if ((t > 0.75 * T and bodyModel.ZMPL * bodyModel.lastZMPL < 0) or t > 3 * T)
        supportFoothasChanged = true;
    if (supportFoothasChanged) {
        weightHasShifted = (bodyModel.isLeftPhase != (bodyModel.ZMPL < 0));
        bodyModel.setIsLeftPhase(bodyModel.ZMPL < 0); // set isLeft phase in body model for kinematics etc
    }
    if (supportFoothasChanged and walk2014Option == WALK) {
        supportFoothasChanged = false;                      //reset
        // 6.1 Recover previous "left" swing angle
        if (bodyModel.isLeftPhase)
            swingAngle = leftL;
        else
            swingAngle = leftR;
        // 6.2 Decide on timing of next walk step phase
        if (walkState == NOT_WALKING) {                       // Start the walk
            nextFootSwitchT = T;
            walkState = STARTING;
        } else if (walkState == STARTING) {
            nextFootSwitchT = T;
            walkState = WALKING;
        } else if (walkState == WALKING) {
            nextFootSwitchT = T;
            walkState = WALKING; // continue walking until interrupted by a command to stop (or kick)
        } else if (walkState == STOPPING) {
            nextFootSwitchT = T;
            walkState = NOT_WALKING;
        } else
            llog(FATAL) << "Should never get here: walkState error" << endl;
        // 6.3 reset step phase time
        t = 0;                                         // reset step phase timer
        // 6.4 backup values
        turnRL0 = turnRL;               // store turn value for use in next step
        forwardR0 = forwardR;                             // sagittal right foot
        forwardL0 = forwardL;                              // sagittal left foot
        // 6.5 Other stuff on support foot change - none at the moment
    } // end of changing support foot

    // 7. Sagittal Balance

    // PD Controller tuned using the Ziegler-Nichols method
    filteredGyroY = 0.8 * filteredGyroY + 0.2 * sensors.sensors[InertialSensor_GyroscopeY];
    // Calculate total output
    //    Control output     =    Kp   x     error     +    Kd   x (          deltaError          / dt)
    sagittalBalanceAdjustment =  KpGyro * filteredGyroY + (KdGyro * (filteredGyroY - preErrorGyro) / dt);

    // Save current error to be next previous error
    preErrorGyro = filteredGyroY;

    // PID Controller hand tuned by inspection
    filteredAngleY = 0.8 * filteredAngleY + 0.2 * sensors.sensors[InertialSensor_AngleY];

    // Calculate error only outside angle thresholds
    if (filteredAngleY > 0.06) {
        angleError = 0.06 - filteredAngleY;
    }
    else if (filteredAngleY < -0.09) {
        angleError = -0.09 - filteredAngleY;
    }
    // Otherwise clear the error and error sum
    else {
        angleErrorSum = 0;
        angleError = 0;
    }

    // Add the error to the error sum
    angleErrorSum += angleError * dt;

    // Calculate total output (negated to adjust hip angle correctally)
    //     Control output       =   Kp    x     error  +    Ki   x    errorSum   + (  Kd   *  (          deltaError          / dt)
    sagittalHipBalanceAdjustment = -(KpAngle * angleError + KiAngle * angleErrorSum + (KdAngle * (angleError - preErrorAngle) / dt));

    //Save error to previous error
    preErrorAngle = angleError;

    if (walk2014Option == READY) {
        sagittalBalanceAdjustment = 0;        // to stop swaying while not walking
    }

    // 7.5 Coronal Balance
    filteredGyroX = 0.8 * filteredGyroX + 0.2 * sensors.sensors[InertialSensor_GyroscopeX];
    if (walk2014Option == KICK) {
        coronalBalanceAdjustment = filteredGyroX / 5; // adjust ankle roll in proportion to filtered gryoX aggressively during kick
    } else {
        coronalBalanceAdjustment = filteredGyroX / 10; // adjust ankle roll in proportion to filtered gryoX
    }

    // 7.6 Toe lift conditions
    // Toe lift when leaning too far forward which could result in toe catching
    if (walk2014Option == WALK and nextFootSwitchT > 0) {   // Only during walk and when a period has been set
        if (sensors.sensors[InertialSensor_AngleY] > 0.2 and forward > 0 and t < 0.6 * T) { // Toe lift only when angle is > 0.2 and less than 60% of phase completed and walking forwards
            toeLiftAngle = sensors.sensors[InertialSensor_AngleY] * 0.7;    // Lift toe by 70% of current forward angle
            if (toeLiftAngle > 0.2)
                toeLiftAngle = 0.2;         // Clamps the amount of toe lift to a maximum of 0.2 rad
            if (bodyModel.isLeftPhase)   // Left phase, lift left toe
                leftToeLift = 1;
            else if (!bodyModel.isLeftPhase)  // Right phase, lift right toe
                rightToeLift = 1;
        }
        if (leftToeLift and (t >= 0.7 * T or !bodyModel.isLeftPhase or t == 0))  // Disable left toe lift after 70% of expected phase time or if no longer left phase
            leftToeLift = 0;
        else if (rightToeLift and (t >= 0.7 * T or bodyModel.isLeftPhase or t == 0))  // Disable right toe lift after 70% of expected phase time or if no longer right phase
            rightToeLift = 0;
    }

    // 8. Odometry update for localisation
    *odometry = *odometry + motionOdometry.updateOdometry(sensors, updateOdometry(bodyModel.isLeftPhase));

    // 9. Work out joint angles from walk variables above
    // 9.1 Left foot closed form inverse kinematics
    float leghL = hiph - foothL - ankle; // vertical height between ankle and hip in meters
    float legX0L = leghL / cos(leftL); // leg extension (eliminating knee) when forwardL = 0
    float legXL = sqrt(legX0L * legX0L + (forwardL + comOffset) * (forwardL + comOffset)); //leg extension at forwardL
    float beta1L = acos((thigh * thigh + legXL * legXL - tibia * tibia) / (2.0f * thigh * legXL)); // acute angle at hip in thigh-tibia triangle
    float beta2L = acos((tibia * tibia + legXL * legXL - thigh * thigh) / (2.0f * tibia * legXL)); // acute angle at ankle in thigh-tibia triangle
    float tempL = legX0L / legXL;
    if (tempL > 1.0f)
        tempL = 1.0f; // sin ratio to calculate leg extension pitch. If > 1 due to numerical error round down.
    float deltaL = asin(tempL);                           // leg extension angle
    float dirL = 1.0f;
    if ((forwardL + comOffset) > 0.0f)
        dirL = -1.0f; // signum of position of foot
    float HpL = beta1L + dirL * (M_PI / 2.0f - deltaL); // Hip pitch is sum of leg-extension + hip acute angle above
    float ApL = beta2L + dirL * (deltaL - M_PI / 2.0f); // Ankle pitch is a similar calculation for the ankle joint
    float KpL = HpL + ApL; // to keep torso upright with both feet on the ground, the knee pitch is always the sum of the hip pitch and the ankle pitch.

    // 9.2 right foot closed form inverse kinematics (comments as above but for right leg)
    float leghR = hiph - foothR - ankle;
    float legX0R = leghR / cos(leftR);
    float legXR = sqrt(legX0R * legX0R + (forwardR + comOffset) * (forwardR + comOffset));
    float dirR = 1.0f;
    if ((forwardR + comOffset) > 0.0f)
        dirR = -1.0f;
    float beta1R = acos((thigh * thigh + legXR * legXR - tibia * tibia) / (2.0f * thigh * legXR));
    float beta2R = acos((tibia * tibia + legXR * legXR - thigh * thigh) / (2.0f * tibia * legXR));
    float tempR = legX0R / legXR;
    if (tempR > 1.0f)
        tempR = 1.0f;
    float deltaR = asin(tempR);
    float HpR = beta1R + dirR * (M_PI / 2.0f - deltaR);
    float ApR = beta2R + dirR * (deltaR - M_PI / 2.0f);
    float KpR = HpR + ApR;

    // 9.3 Sert hip and ankle values
    float HrL = -leftL;
    float HrR = -leftR;
    float ArL = -HrL;
    float ArR = -HrR;
    if (walk2014Option == KICK || walk2014Option == STEP) {
        HrL += rock;
        HrR += rock;
        ArL -= rock;
        ArR -= rock;
    }

    // 9.4 Adjust HpL, HrL, ApL, ArL LEFT based on Hyp turn to keep ankle in situ
    // Turning
    XYZ_Coord tL = mf2b(z, -HpL, HrL, KpL, -ApL, ArL, z, z, z);
    XYZ_Coord sL;
    float Hyp = -turnRL;
    for (int i = 0; i < 3; i++) {
        sL = mf2b(Hyp, -HpL, HrL, KpL, -ApL, ArL, z, z, z);
        XYZ_Coord e((tL.x - sL.x), (tL.y - sL.y), (tL.z - sL.z));
        Hpr hpr = hipAngles(Hyp, -HpL, HrL, KpL, -ApL, ArL, z, z, z, e);
        HpL -= hpr.Hp;
        HrL += hpr.Hr;
    }
    // ApL and ArL to make sure LEFT foot is parallel to ground
    XYZ_Coord up = mf2b(Hyp, -HpL, HrL, KpL, -ApL, ArL, 1.0f, 0.0f, 0.0f);
    XYZ_Coord ur = mf2b(Hyp, -HpL, HrL, KpL, -ApL, ArL, 0.0f, 1.0f, 0.0f);
    ApL = ApL + asin(sL.z - up.z);
    ArL = ArL + asin(sL.z - ur.z);

    // 9.5 Adjust HpR, HrR, ApR, ArR (RIGHT) based on Hyp turn to keep ankle in situ
    // Map to LEFT - we reuse the left foot IK because of symmetry right foot
    float Hr = -HrR;
    float Ar = -ArR;
    // Target foot origin in body coords
    XYZ_Coord t = mf2b(z, -HpR, Hr, KpR, -ApR, Ar, z, z, z);
    XYZ_Coord s;
    Hyp = -turnRL;
    for (int i = 0; i < 3; i++) {
        s = mf2b(Hyp, -HpR, Hr, KpR, -ApR, Ar, z, z, z);
        XYZ_Coord e((t.x - s.x), (t.y - s.y), (t.z - s.z));
        Hpr hpr = hipAngles(Hyp, -HpR, Hr, KpR, -ApR, Ar, z, z, z, e);
        HpR -= hpr.Hp;
        Hr += hpr.Hr;
    }
    // 9.6 Ap and Ar to make sure foot is parallel to ground
    XYZ_Coord u1 = mf2b(Hyp, -HpR, Hr, KpR, -ApR, Ar, 1.0f, 0.0f, 0.0f);
    XYZ_Coord u2 = mf2b(Hyp, -HpR, Hr, KpR, -ApR, Ar, 0.0f, 1.0f, 0.0f);
    ApR = ApR + asin(s.z - u1.z);
    Ar = Ar + asin(s.z - u2.z);
    // map back from left foot to right foot
    HrR = -Hr;
    ArR = -Ar;

    // 10. Set joint values and stiffness
    JointValues j = sensors.joints;
    for (uint8_t i = 0; i < Joints::NUMBER_OF_JOINTS; ++i)
        j.stiffnesses[i] = stiffness;

    // Overwrite knee and ankle pitch stiffness to reduce overheating
    if (walk2014Option == WALK && !exactStepsRequested) {
        j.stiffnesses[Joints::LKneePitch] = kneeStiffness;
        j.stiffnesses[Joints::RKneePitch] = kneeStiffness;
        j.stiffnesses[Joints::LAnklePitch] = ankleStiffness;
        j.stiffnesses[Joints::RAnklePitch] = ankleStiffness;
    }

    // 10.1 Arms
    j.angles[LShoulderPitch] = DEG2RAD(90) + shoulderPitchL;
    j.angles[LShoulderRoll] = DEG2RAD(7) + shoulderRollL;
    j.angles[LElbowRoll] = DEG2RAD(0); //DEG2RAD(-30)+shoulderPitchL;  //swing bent arms
    j.angles[LWristYaw] = DEG2RAD(0);
    j.angles[RShoulderPitch] = DEG2RAD(90) + shoulderPitchR;
    j.angles[RShoulderRoll] = DEG2RAD(-7) - shoulderRollR;
    j.angles[RElbowRoll] = DEG2RAD(0); //DEG2RAD(30)-shoulderPitchR; //swing bent arms
    j.angles[RWristYaw] = DEG2RAD(0);

    float armStiffness = 0.1f;
    int stiffnessCounter = 90;

    if (active.leftArmLimp) {
        j.stiffnesses[Joints::LShoulderPitch] = -1.0f;
        j.stiffnesses[Joints::LShoulderRoll] = -1.0f;
        j.angles[LElbowYaw] = DEG2RAD(90); //DEG2RAD(-90); //swing bent arms

        if (lElbowYawCounter < stiffnessCounter) {
            j.stiffnesses[Joints::LElbowYaw] = armStiffness;
            lElbowYawCounter++;
        }
        else {
            j.stiffnesses[Joints::LElbowYaw] = -1.0f;
        }
    }
    else {
        j.stiffnesses[Joints::LShoulderPitch] = armStiffness;
        j.stiffnesses[Joints::LShoulderRoll] = armStiffness;
        j.angles[LElbowYaw] = DEG2RAD(-90); //DEG2RAD(-90); //swing bent arms

        if (lElbowYawCounter > -stiffnessCounter) {
            j.stiffnesses[Joints::LElbowYaw] = armStiffness;
            lElbowYawCounter--;

        }
        else {
            j.stiffnesses[Joints::LElbowYaw] = -1.0f;
        }
    }

    if (active.rightArmLimp) {
        j.stiffnesses[Joints::RShoulderPitch] = -1.0f;
        j.stiffnesses[Joints::RShoulderRoll] = -1.0f;
        j.angles[RElbowYaw] = DEG2RAD(-90); //DEG2RAD(90);  //swing bent arms

        if (rElbowYawCounter < stiffnessCounter) {
            j.stiffnesses[Joints::RElbowYaw] = armStiffness;
            rElbowYawCounter++;
        }
        else {
            j.stiffnesses[Joints::RElbowYaw] = -1.0f;
        }
    }
    else {
        j.stiffnesses[Joints::RShoulderPitch] = armStiffness;
        j.stiffnesses[Joints::RShoulderRoll] = armStiffness;
        j.angles[RElbowYaw] = DEG2RAD(90); //DEG2RAD(90);  //swing bent arms

        if (rElbowYawCounter > -stiffnessCounter) {
            j.stiffnesses[Joints::RElbowYaw] = armStiffness;
            rElbowYawCounter--;
        }
        else {
            j.stiffnesses[Joints::RElbowYaw] = -1.0f;
        }
    }

    // Make anything shoulder down limp for greater stability when walking into another robot
    j.stiffnesses[Joints::LElbowRoll] = -1.0f;
    j.stiffnesses[Joints::LWristYaw] = -1.0f;
    j.stiffnesses[Joints::RElbowRoll] = -1.0f;
    j.stiffnesses[Joints::RWristYaw] = -1.0f;

    // 10.2 Turn
    j.angles[Joints::LHipYawPitch] = -turnRL;

    // 10.3 Sagittal Joints
    j.angles[Joints::LHipPitch] = -HpL;
    j.angles[Joints::RHipPitch] = -HpR;
    j.angles[Joints::LKneePitch] = KpL;
    j.angles[Joints::RKneePitch] = KpR;

    // Toe lift
    if (leftToeLift)    // Left toe needs lift
        ApL += toeLiftAngle;    // Ankle is already calculated to be parallel to ground, dynamic addition to lift toe
    if (rightToeLift)   // Right toe needs lift
        ApR += toeLiftAngle;    // Ankle is already calculated to be parallel to ground, dynamic addition to lift toe

    // Only activate balance control if foot is on the ground
    j.angles[Joints::LAnklePitch] = -ApL;
    j.angles[Joints::RAnklePitch] = -ApR;
    if (walk2014Option == WALK and nextFootSwitchT > 0) {
        if (bodyModel.isLeftPhase) {
            if (forward == 0 && turn == 0 && left != 0) {
                // If we are only walking sideways then balance using the hip pitch over ankle pitch
                j.angles[Joints::RAnkleRoll] += coronalBalanceAdjustment;
                j.angles[Joints::RHipPitch] += sagittalBalanceAdjustment;
            }
            else {
                j.angles[Joints::RAnklePitch] += sagittalBalanceAdjustment;
                j.angles[Joints::RAnkleRoll] += coronalBalanceAdjustment;
                j.angles[Joints::RHipPitch] += sagittalHipBalanceAdjustment;
            }

        } else {
            if (forward == 0 && turn == 0 && left != 0) {
                // If we are only walking sideways then balance using the hip pitch over ankle pitch
                j.angles[Joints::LAnkleRoll] += coronalBalanceAdjustment;
                j.angles[Joints::LHipPitch] += sagittalBalanceAdjustment;
            }
            else {
                j.angles[Joints::LAnklePitch] += sagittalBalanceAdjustment;
                j.angles[Joints::LAnkleRoll] += coronalBalanceAdjustment;
                j.angles[Joints::LHipPitch] += sagittalHipBalanceAdjustment;
            }
        }
    } else if (walk2014Option == KICK) {
        if (bodyModel.isLeftPhase)
            j.angles[Joints::RAnklePitch] += sagittalBalanceAdjustment;
        else
            j.angles[Joints::LAnklePitch] += sagittalBalanceAdjustment;
    } else {
        j.angles[Joints::RAnklePitch] += sagittalBalanceAdjustment;
        j.angles[Joints::LAnklePitch] += sagittalBalanceAdjustment;
    }

    // 10.4 Coronal Joints
    j.angles[Joints::LHipRoll] = HrL;
    j.angles[Joints::RHipRoll] = HrR;
    j.angles[Joints::LAnkleRoll] = ArL;
    j.angles[Joints::RAnkleRoll] = ArR;

    // Add in joint adjustments for kicks
    if (walk2014Option == KICK) {
        addKickJoints(j);
        // Add in some coronal balancing
        if (bodyModel.isLeftPhase) {
            j.angles[Joints::RAnkleRoll] += coronalBalanceAdjustment;
            if(hipBalance == true) {
                j.angles[Joints::RHipRoll] += coronalBalanceAdjustment / 2;
            }
        }
        else {
            j.angles[Joints::LAnkleRoll] += coronalBalanceAdjustment;
            if(hipBalance == true) {
                j.angles[Joints::LHipRoll] += coronalBalanceAdjustment / 2;
            }
        }
    }

    return j;
}

void Walk2014Generator::prepKick(bool isLeft, BodyModel &bodyModel) {
    t = 0;
    walk2014Option = KICK;
    turnAngle = 0;
    kickT = 0;
    holdAnkle = false;
    hipBalance = false;
    if (isLeft) {
        lastKickForward = forwardL;
        lastKneePitch = kneePitchL;
        lastSide = leftL;
        bodyModel.setIsLeftPhase(true);
        forwardR = 0;
        foothR = 0;
    } else {
        lastKickForward = forwardR;
        lastKneePitch = kneePitchR;
        lastSide = -leftR;
        bodyModel.setIsLeftPhase(false);
        forwardL = 0;
        foothL = 0;
    }
}

// nicer for the motors
bool Walk2014Generator::canAbortKick() {
    float totalShift = shiftPeriod / 4;
    float halfShift = totalShift / 2;
    float pauseTime = (backPhase - totalShift) / 2;
    return kickT - pauseTime < halfShift || (kickT >= totalShift + pauseTime && kickT < backPhase);
}

void Walk2014Generator::makeForwardKickJoints(float kickingLean, float kickStepH, float &footh, float &forwardDist, float &side, float &kneePitch,
        float &shoulderRoll, float &anklePitch, float&ballX, float &ballY, ActionCommand::All* request) {

    kickT += dt;
    float totalPhase = backPhase + kickPhase + throughPhase + endPhase;

    // Used to enable hip stabalisation after lean during kick
    hipBalance = false;

    // Update side position of ball as late as possible.
    if (request->body.misalignedKick) {
        dynamicSide = 0; // This will try to kick the robot with the outside of the foot.
    } else if (kickT < backPhase * 0.8) {
        dynamicSide = ballY - 50; // offset to kick point on foot
    }
    // Max safe/useful value for lifting leg (without going past joint limits/balancing)
    float sideAmp = -MAX(0, MIN(90, dynamicSide)) / 200.0;
    float kickAmp = -0.07; // how far forward the kick should reach
    float kickPower = pow(power, 1.7);
    float kickBackAmp = -kickAmp * kickPower;

    float ballXAdjust = ballX / 3.5;

    // Calculate the knee extension to hit balls far away from the robot
    if (kickT < backPhase) {
        kneePitchEnd = -MAX(35, MIN(75, ballXAdjust));
        anklePitchEnd = MAX(0, MIN(50,-kneePitchEnd * 1.3));
        if (fabs(ballY) > 40 && ballX > 190){
            anklePitchEnd += MAX(0, MIN(15, dynamicSide));
        }
        if (ballX < 200) {
            anklePitchStart = (ballX - 180) / 2;
            anklePitchStart = MAX(5, anklePitchStart);
            swingDelayFactor = 0.1;
            holdAnkle = true;
        } else if (ballX < 215) {
            anklePitchStart = 0.33 * ballX - 56;
            swingDelayFactor = 0.2;
        } else {
            anklePitchStart = 15;
            swingDelayFactor = 0.3;
        }
    }

    float shoulderRollAmp = -sideAmp / shoulderRollAmpDivisor; // how much arm should lift to make room for raised kicking leg
    float kneePitchAmp = DEG2RAD(35);
    float kneePitchAmpEnd = DEG2RAD(kneePitchEnd);
    float anklePitchAmp = DEG2RAD(anklePitchEnd);
    float anklePitchRetracted = DEG2RAD(anklePitchStart);
    float anklePitchAmpFinal = DEG2RAD(35);

    if (lastKickTurn > 0) {
        shiftPeriod = 3.5;
    }

    // Shift weight over and swing foot back
    if (kickT < backPhase) {
        float totalShift = shiftPeriod / 4;
        float halfShift = totalShift / 2;
        float pauseTime = (backPhase - totalShift) / 2;
        // pause slightly to make sure we at a steady state before moving our weight
        if (t >= pauseTime) {
            float t2 = t - pauseTime;
            // shift weight sideways
            if (t2 < totalShift) {
                rock = DEG2RAD(kickingLean) * parabolicStep(t2, totalShift, 0);
                // If the robot had turned, bring the feet back together after some time for foot to lift off the ground
                if (lastKickTurn > 0) {
                    if (t2 >= halfShift) {
                        turnRL = lastKickTurn * (1 - parabolicStep(t2 - halfShift, halfShift, 0.0));
                    } else {
                        turnRL = lastKickTurn;
                    }
                    // Add in some extra lean while shifting weight along with turn
                    rock += DEG2RAD(kickingLean) / 3.5 * parabolicReturn(t2 / totalShift);
                }
            } else {
                turnRL = 0;
                lastKickTurn = 0;
            }

            // only start lifting the kicking at 1/3
            if (t2 >= totalShift / 3) {
                float t3 = t2 - totalShift / 3;
                float endT = backPhase - totalShift / 3 - pauseTime;
                footh = kickStepH * parabolicStep(t3, endT, 0);
            }

            // Once we're halfway through shifting our weight, start moving the foot back.
            if (kickT - pauseTime >= halfShift) {
                float kickT2 = kickT - halfShift - pauseTime;
                float endT = backPhase - halfShift - pauseTime;
                forwardDist = interpolateSmooth(0, kickBackAmp, kickT2, endT);
                side = interpolateSmooth(0, sideAmp, kickT2, endT);
                shoulderRoll = interpolateSmooth(0, shoulderRollAmp, kickT2, endT);
                kneePitch = interpolateSmooth(0, kneePitchAmp * 0.9, kickT2, endT);
                anklePitch = interpolateSmooth(0, anklePitchRetracted, kickT2, endT);
            }
        }
        // Swing foot forward.
    } else if (kickT < (backPhase + kickPhase)) {
        // Use hip balance now
        hipBalance = true;
        if (kickT >= backPhase + swingDelayFactor * kickPhase) {
            forwardDist = interpolateSmooth(kickBackAmp, kickAmp, kickT - backPhase - swingDelayFactor*kickPhase, kickPhase);
            anklePitch = interpolateSmooth(anklePitchRetracted, anklePitchAmp, kickT - backPhase - swingDelayFactor*kickPhase, kickPhase);            
        }
        if (holdAnkle == true) {
            // Overwrite if we want to hold ankle
            anklePitch = anklePitchRetracted;
        }
        side = sideAmp;
        shoulderRoll = shoulderRollAmp;
        kneePitch = squareSmooth(kneePitchAmp * 0.9, kneePitchAmpEnd, kickT - backPhase, kickPhase);
        // Hold...
    } else if (kickT < (backPhase + kickPhase + throughPhase)) {
        // Keep hip balance
        hipBalance = true;        
        forwardDist = kickAmp;
        side = sideAmp;
        shoulderRoll = shoulderRollAmp;
        kneePitch = kneePitchAmpEnd;
        if (holdAnkle == true) {
            anklePitch = interpolateSmooth(anklePitchRetracted, anklePitchAmpFinal,kickT - backPhase - kickPhase, throughPhase);            
        } else {
            anklePitch = interpolateSmooth(anklePitchAmp, anklePitchAmpFinal,kickT - backPhase - kickPhase, throughPhase);
        }
    //     // Return foot. @ijnek: this step was taken out in an attempt to stabilise the kick
    // } else if (kickT < (backPhase + kickPhase + throughPhase + endPhase)) {
    //     forwardDist = interpolateSmooth(lastKickForward, 0, kickT - backPhase - kickPhase - throughPhase, endPhase);
    //     side = interpolateSmooth(lastSide, 0, kickT - backPhase - kickPhase - throughPhase, endPhase);
    //     shoulderRoll = interpolateSmooth(lastShoulderRollAmp, 0, kickT - backPhase - kickPhase - throughPhase, endPhase);
    //     kneePitch = interpolateSmooth(lastKneePitch, 0, kickT - backPhase - kickPhase - throughPhase, endPhase);
    //     anklePitch = interpolateSmooth(lastAnklePitch, 0, kickT - backPhase - kickPhase - throughPhase, endPhase);
    //
        // Shift weight back to both feet.
    } else if (kickT < (totalPhase + shiftEndPeriod / 4)) {
        forwardDist = lastKickForward - lastKickForward * parabolicStep(kickT-totalPhase, shiftEndPeriod/8, 0);
        side = lastSide - lastSide * parabolicStep(kickT-totalPhase, shiftEndPeriod/8, 0);
        shoulderRoll = lastShoulderRollAmp - shoulderRollAmp * parabolicStep(kickT-totalPhase, shiftEndPeriod/8, 0);
        kneePitch = lastKneePitch - lastKneePitch * parabolicStep(kickT-totalPhase, shiftEndPeriod/8, 0);
        anklePitch = lastAnklePitch - lastAnklePitch * parabolicStep(kickT-totalPhase, shiftEndPeriod/6, 0);
        rock = lastRock - lastRock * parabolicStep(kickT-totalPhase, shiftEndPeriod/4, 0);
        footh = lastFooth - lastFooth * parabolicStep(kickT-totalPhase, shiftEndPeriod/6, 0);
    } else {
        kickT = 0;
        rock = 0;
        footh = 0;
        walk2014Option = WALK;
        walkState = NOT_WALKING;
        request->body.actionType = ActionCommand::Body::WALK;
        lastKickTime = 0;
    }

    // aborting or ending a kick from these values
    if (kickT < (backPhase + kickPhase + throughPhase)) {
        lastKickForward = forwardDist;
        lastKneePitch = kneePitch;
        lastSide = side;
        lastFooth = footh;
        lastAnklePitch = anklePitch;
        lastRock = rock;
        lastShoulderRollAmp = shoulderRollAmp;
    }
}

void Walk2014Generator::addKickJoints(JointValues &j) {
    j.angles[Joints::LKneePitch] += kneePitchL;
    j.angles[Joints::LAnklePitch] += anklePitchL + shoulderRollL;
    j.angles[Joints::LShoulderPitch] -= kneePitchL;

    j.angles[Joints::RKneePitch] += kneePitchR;
    j.angles[Joints::RAnklePitch] += anklePitchR + shoulderRollR;
    j.angles[Joints::RShoulderPitch] -= kneePitchR;
}

float Walk2014Generator::leftAngle() {
    float left_at_t = left * parabolicStep(t, nextFootSwitchT, 0.0);
    float height = hiph - ankle;
    return atan(left_at_t / height);
}

float Walk2014Generator::linearStep(float time, float period) {
    if (time <= 0)
        return 0;
    if (time >= period)
        return 1;
    return time / period;
}

float Walk2014Generator::parabolicReturn(float f) { //normalised [0,1] up and down
    double x = 0;
    double y = 0;
    if (f < 0.25f) {
        y = 8 * f * f;
    }
    if (f >= 0.25f && f < 0.5f) {
        x = 0.5f - f;
        y = 8 * x * x;
        y = 1.0f - y;
    }
    if (f >= 0.5f && f < 0.75f) {
        x = f - 0.5f;
        y = 8 * x * x;
        y = 1.0f - y;
    }
    if (f >= 0.75f && f <= 1.0f) {
        x = 1.0f - f;
        y = 8 * x * x;
    }
    return y;
}

float Walk2014Generator::parabolicReturnMod(float f) { //normalised [0,1] up and down
    double x = 0;
    double y = 0;
    if (f < 0.25f) {
        // y: 0 -> 0.75
        y = 8 * f * f * 1.50;
    }
    if (f >= 0.25f && f < 0.5f) {
        // y: 0.75 -> 1.00
        x = 0.5f - f;
        y = 8 * x * x;
        y = y / 2;
        y = 1.0f - y;
    }
    if (f >= 0.5f && f < 0.75f) {
        // y: 1.00 -> 0.75
        x = f - 0.5f;
        y = 8 * x * x;
        y = y / 2;
        y = 1.0f - y;
    }
    if (f >= 0.75f && f <= 1.0f) {
        // y: 0.75 -> 0
        x = 1.0f - f;
        y = 8 * x * x * 1.50;
    }
    return y;
}

float Walk2014Generator::parabolicStep(float time, float period, float deadTimeFraction) { //normalised [0,1] step up
    float deadTime = period * deadTimeFraction / 2;
    if (time < deadTime + dt / 2)
        return 0;
    if (time > period - deadTime - dt / 2)
        return 1;
    float timeFraction = (time - deadTime) / (period - 2 * deadTime);
    if (time < period / 2)
        return 2.0 * timeFraction * timeFraction;
    return 4 * timeFraction - 2 * timeFraction * timeFraction - 1;
}

Odometry Walk2014Generator::updateOdometry(bool isLeftSwingFoot) {
    // Work out incremental forward, left, and turn values for next time step
    float height = hiph - ankle;
    float turnOdo = -(turnRL - prevTurn);
    float leftOdo = (height * tan(leftR) - prevLeftR);
    float forwardOdo = (forwardR - prevForwardR);
    if (!isLeftSwingFoot) {
        turnOdo *= -1;
        leftOdo = (height * tan(leftL) - prevLeftL);
        forwardOdo = (forwardL - prevForwardL);
    }
    forwardOdo *= MM_PER_M;
    leftOdo *= MM_PER_M;
    //Calibrate odometry to match the actual speed
    forwardOdo *= 1;
    leftOdo *= 1.23;
    turnOdo *= -.80;

    // backup odometry values
    prevTurn = turnRL;
    prevLeftL = height * tan(leftL);
    prevLeftR = height * tan(leftR);
    prevForwardL = forwardL;
    prevForwardR = forwardR;
    return Odometry(forwardOdo, leftOdo, turnOdo);
}

bool Walk2014Generator::isActive() {
    return !stopped;
}

void Walk2014Generator::readOptions(const boost::program_options::variables_map &config) {
    kick_fast = (config["motion.kick_speed"].as<std::string>() == "FAST");
    if (kick_fast){
        kickLeanOffset = config["motion.fast_kick_lean_offset"].as<float>();
    } else {
        kickLeanOffset = config["motion.kick_lean_offset"].as<float>();
    }
}

void Walk2014Generator::reset() {
    initialise();
    llog(INFO) << "Walk2014 reset" << endl;
}

void Walk2014Generator::stop() {
    // this does nothing
    stopping = true;
    llog(INFO) << "Walk2014 stop" << endl;
}

float Walk2014Generator::interpolateSmooth(float start, float end, float tCurrent, float tEnd) {
    return start + (end - start) * (1 + cos(M_PI * tCurrent / tEnd - M_PI)) / 2;
}

float Walk2014Generator::squareSmooth(float start, float end, float tCurrent, float tEnd) {
    return ((end - start)/pow(tEnd,2)) * (pow(tCurrent,2)) + start;
}

XYZ_Coord Walk2014Generator::mf2b(float Hyp, float Hp, float Hr, float Kp, float Ap, float Ar, float xf, float yf, float zf) {
    // MFOOT2BODY Transform coords from foot to body.
    // This code originates from 2010 using symbolic equations in Matlab to perform the coordinate transforms - see team report (BH)
    // In future this approach to IK for the Nao should be reworked in closed form, significantly reducing the size of the code the
    // the computational complexity (BH)
    XYZ_Coord result;
    float pi = M_PI;
    float tibia = this->tibia * 1000;
    float thigh = this->thigh * 1000;
    float k = sqrt(2.0);
    float c1 = cos(Ap);
    float c2 = cos(Hr + pi / 4.0);
    float c3 = cos(Hyp - pi / 2.0);
    float c4 = cos(Hp);
    float c5 = cos(Kp);
    float c6 = cos(Ar - pi / 2.0);
    float s1 = sin(Kp);
    float s2 = sin(Hp);
    float s3 = sin(Hyp - 1.0 / 2.0 * pi);
    float s4 = sin(Hr + 1.0 / 4.0 * pi);
    float s5 = sin(Ap);
    float s6 = sin(Ar - 1.0 / 2.0 * pi);
    result.x = thigh * (s2 * s3 - c2 * c3 * c4) + tibia * (s1 * (c4 * s3 + c2 * c3 * s2) + c5 * (s2 * s3 - c2 * c3 * c4))
            - yf
                    * (c6 * (c1 * (s1 * (c4 * s3 + c2 * c3 * s2) + c5 * (s2 * s3 - c2 * c3 * c4)) - s5 * (s1 * (s2 * s3 - c2 * c3 * c4) - c5 * (c4 * s3 + c2 * c3 * s2)))
                            + c3 * s4 * s6)
            + zf
                    * (s6 * (c1 * (s1 * (c4 * s3 + c2 * c3 * s2) + c5 * (s2 * s3 - c2 * c3 * c4)) - s5 * (s1 * (s2 * s3 - c2 * c3 * c4) - c5 * (c4 * s3 + c2 * c3 * s2)))
                            - c3 * c6 * s4)
            + xf * (c1 * (s1 * (s2 * s3 - c2 * c3 * c4) - c5 * (c4 * s3 + c2 * c3 * s2)) + s5 * (s1 * (c4 * s3 + c2 * c3 * s2) + c5 * (s2 * s3 - c2 * c3 * c4)));
    result.y = xf
            * (c1 * (c5 * (s2 * ((k * s4) / 2.0f + (c2 * k * s3) / 2.0f) - (c3 * c4 * k) / 2.0f) + s1 * (c4 * ((k * s4) / 2.0f + (c2 * k * s3) / 2.0f) + (c3 * k * s2) / 2.0f))
                    + s5
                            * (c5 * (c4 * ((k * s4) / 2.0f + (c2 * k * s3) / 2.0f) + (c3 * k * s2) / 2.0f)
                                    - s1 * (s2 * ((k * s4) / 2.0f + (c2 * k * s3) / 2.0f) - (c3 * c4 * k) / 2.0f)))
            + tibia * (c5 * (c4 * ((k * s4) / 2.0f + (c2 * k * s3) / 2.0f) + (c3 * k * s2) / 2.0f) - s1 * (s2 * ((k * s4) / 2.0f + (c2 * k * s3) / 2.0f) - (c3 * c4 * k) / 2.0f))
            + thigh * (c4 * ((k * s4) / 2.0f + (c2 * k * s3) / 2.0f) + (c3 * k * s2) / 2.0f)
            - yf
                    * (s6 * ((c2 * k) / 2.0f - (k * s3 * s4) / 2.0f)
                            + c6
                                    * (c1
                                            * (c5 * (c4 * ((k * s4) / 2.0f + (c2 * k * s3) / 2.0f) + (c3 * k * s2) / 2.0f)
                                                    - s1 * (s2 * ((k * s4) / 2.0f + (c2 * k * s3) / 2.0f) - (c3 * c4 * k) / 2.0f))
                                            - s5
                                                    * (c5 * (s2 * ((k * s4) / 2.0f + (c2 * k * s3) / 2.0f) - (c3 * c4 * k) / 2.0f)
                                                            + s1 * (c4 * ((k * s4) / 2.0f + (c2 * k * s3) / 2.0f) + (c3 * k * s2) / 2.0f))))
            - zf
                    * (c6 * ((c2 * k) / 2.0f - (k * s3 * s4) / 2.0f)
                            - s6
                                    * (c1
                                            * (c5 * (c4 * ((k * s4) / 2.0f + (c2 * k * s3) / 2.0f) + (c3 * k * s2) / 2.0f)
                                                    - s1 * (s2 * ((k * s4) / 2.0f + (c2 * k * s3) / 2.0f) - (c3 * c4 * k) / 2.0f))
                                            - s5
                                                    * (c5 * (s2 * ((k * s4) / 2.0f + (c2 * k * s3) / 2.0f) - (c3 * c4 * k) / 2.0f)
                                                            + s1 * (c4 * ((k * s4) / 2.0f + (c2 * k * s3) / 2.0f) + (c3 * k * s2) / 2.0f))));
    result.z = yf
            * (s6 * ((c2 * k) / 2.0f + (k * s3 * s4) / 2.0f)
                    + c6
                            * (c1
                                    * (c5 * (c4 * ((k * s4) / 2.0f - (c2 * k * s3) / 2.0f) - (c3 * k * s2) / 2.0f)
                                            - s1 * (s2 * ((k * s4) / 2.0f - (c2 * k * s3) / 2.0f) + (c3 * c4 * k) / 2.0f))
                                    - s5
                                            * (c5 * (s2 * ((k * s4) / 2.0f - (c2 * k * s3) / 2.0f) + (c3 * c4 * k) / 2.0f)
                                                    + s1 * (c4 * ((k * s4) / 2.0f - (c2 * k * s3) / 2.0f) - (c3 * k * s2) / 2.0f))))
            - tibia * (c5 * (c4 * ((k * s4) / 2.0f - (c2 * k * s3) / 2.0f) - (c3 * k * s2) / 2.0f) - s1 * (s2 * ((k * s4) / 2.0f - (c2 * k * s3) / 2.0f) + (c3 * c4 * k) / 2.0f))
            - thigh * (c4 * ((k * s4) / 2.0f - (c2 * k * s3) / 2.0f) - (c3 * k * s2) / 2.0f)
            - xf
                    * (c1
                            * (c5 * (s2 * ((k * s4) / 2.0f - (c2 * k * s3) / 2.0f) + (c3 * c4 * k) / 2.0f)
                                    + s1 * (c4 * ((k * s4) / 2.0f - (c2 * k * s3) / 2.0f) - (c3 * k * s2) / 2.0f))
                            + s5
                                    * (c5 * (c4 * ((k * s4) / 2.0f - (c2 * k * s3) / 2.0f) - (c3 * k * s2) / 2.0f)
                                            - s1 * (s2 * ((k * s4) / 2.0f - (c2 * k * s3) / 2.0f) + (c3 * c4 * k) / 2.0f)))
            + zf
                    * (c6 * ((c2 * k) / 2.0f + (k * s3 * s4) / 2.0f)
                            - s6
                                    * (c1
                                            * (c5 * (c4 * ((k * s4) / 2.0f - (c2 * k * s3) / 2.0f) - (c3 * k * s2) / 2.0f)
                                                    - s1 * (s2 * ((k * s4) / 2.0f - (c2 * k * s3) / 2.0f) + (c3 * c4 * k) / 2.0f))
                                            - s5
                                                    * (c5 * (s2 * ((k * s4) / 2.0f - (c2 * k * s3) / 2.0f) + (c3 * c4 * k) / 2.0f)
                                                            + s1 * (c4 * ((k * s4) / 2.0f - (c2 * k * s3) / 2.0f) - (c3 * k * s2) / 2.0f))));
    return result;
}

Walk2014Generator::Hpr Walk2014Generator::hipAngles(float Hyp, float Hp, float Hr, float Kp, float Ap, float Ar, float xf, float yf, float zf, XYZ_Coord e) {
    // Code from 2010 to perform interative Inverse Kinematics.
    // Symbolic equations generated in Matlab - see 2010 team report for details and reference
    Hpr result;
    float pi = M_PI;
    float tibia = this->tibia * 1000;
    float thigh = this->thigh * 1000;
    float k = sqrt(2.0);
    float c1 = cos(Ap);
    float c2 = cos(Hr + pi / 4.0);
    float c3 = cos(Hyp - pi / 2.0);
    float c4 = cos(Hp);
    float c5 = cos(Kp);
    float c6 = cos(Ar - pi / 2.0);
    float s1 = sin(Kp);
    float s2 = sin(Hp);
    float s3 = sin(Hyp - 1.0 / 2.0 * pi);
    float s4 = sin(Hr + 1.0 / 4.0 * pi);
    float s5 = sin(Ap);
    float s6 = sin(Ar - 1.0 / 2.0 * pi);
    float j11 = thigh * (c4 * s3 + c2 * c3 * s2) - tibia * (s1 * (s2 * s3 - c2 * c3 * c4) - c5 * (c4 * s3 + c2 * c3 * s2))
            + xf * (c1 * (s1 * (c4 * s3 + c2 * c3 * s2) + c5 * (s2 * s3 - c2 * c3 * c4)) - s5 * (s1 * (s2 * s3 - c2 * c3 * c4) - c5 * (c4 * s3 + c2 * c3 * s2)))
            + c6 * yf * (c1 * (s1 * (s2 * s3 - c2 * c3 * c4) - c5 * (c4 * s3 + c2 * c3 * s2)) + s5 * (s1 * (c4 * s3 + c2 * c3 * s2) + c5 * (s2 * s3 - c2 * c3 * c4)))
            - s6 * zf * (c1 * (s1 * (s2 * s3 - c2 * c3 * c4) - c5 * (c4 * s3 + c2 * c3 * s2)) + s5 * (s1 * (c4 * s3 + c2 * c3 * s2) + c5 * (s2 * s3 - c2 * c3 * c4)));
    float j12 = yf * (c6 * (c1 * (c3 * s1 * s2 * s4 - c3 * c4 * c5 * s4) + s5 * (c3 * c4 * s1 * s4 + c3 * c5 * s2 * s4)) - c2 * c3 * s6)
            - tibia * (c3 * s1 * s2 * s4 - c3 * c4 * c5 * s4)
            - zf * (s6 * (c1 * (c3 * s1 * s2 * s4 - c3 * c4 * c5 * s4) + s5 * (c3 * c4 * s1 * s4 + c3 * c5 * s2 * s4)) + c2 * c3 * c6)
            + xf * (c1 * (c3 * c4 * s1 * s4 + c3 * c5 * s2 * s4) - s5 * (c3 * s1 * s2 * s4 - c3 * c4 * c5 * s4)) + c3 * c4 * s4 * thigh;
    float j21 = xf
            * (c1 * (c5 * (c4 * ((k * s4) / 2.0f + (c2 * k * s3) / 2.0f) + (c3 * k * s2) / 2.0f) - s1 * (s2 * ((k * s4) / 2.0f + (c2 * k * s3) / 2.0f) - (c3 * c4 * k) / 2.0f))
                    - s5
                            * (c5 * (s2 * ((k * s4) / 2.0f + (c2 * k * s3) / 2.0f) - (c3 * c4 * k) / 2.0f)
                                    + s1 * (c4 * ((k * s4) / 2.0f + (c2 * k * s3) / 2.0f) + (c3 * k * s2) / 2.0f)))
            - tibia * (c5 * (s2 * ((k * s4) / 2.0f + (c2 * k * s3) / 2.0f) - (c3 * c4 * k) / 2.0f) + s1 * (c4 * ((k * s4) / 2.0f + (c2 * k * s3) / 2.0f) + (c3 * k * s2) / 2.0f))
            - thigh * (s2 * ((k * s4) / 2.0f + (c2 * k * s3) / 2.0f) - (c3 * c4 * k) / 2.0f)
            + c6 * yf
                    * (c1
                            * (c5 * (s2 * ((k * s4) / 2.0f + (c2 * k * s3) / 2.0f) - (c3 * c4 * k) / 2.0f)
                                    + s1 * (c4 * ((k * s4) / 2.0f + (c2 * k * s3) / 2.0f) + (c3 * k * s2) / 2.0f))
                            + s5
                                    * (c5 * (c4 * ((k * s4) / 2.0f + (c2 * k * s3) / 2.0f) + (c3 * k * s2) / 2.0f)
                                            - s1 * (s2 * ((k * s4) / 2.0f + (c2 * k * s3) / 2.0f) - (c3 * c4 * k) / 2.0f)))
            - s6 * zf
                    * (c1
                            * (c5 * (s2 * ((k * s4) / 2.0f + (c2 * k * s3) / 2.0f) - (c3 * c4 * k) / 2.0f)
                                    + s1 * (c4 * ((k * s4) / 2.0f + (c2 * k * s3) / 2.0f) + (c3 * k * s2) / 2.0f))
                            + s5
                                    * (c5 * (c4 * ((k * s4) / 2.0f + (c2 * k * s3) / 2.0f) + (c3 * k * s2) / 2.0f)
                                            - s1 * (s2 * ((k * s4) / 2.0f + (c2 * k * s3) / 2.0f) - (c3 * c4 * k) / 2.0f)));
    float j22 = tibia * (c4 * c5 * ((c2 * k) / 2.0f - (k * s3 * s4) / 2.0f) - s1 * s2 * ((c2 * k) / 2.0f - (k * s3 * s4) / 2.0f))
            + xf
                    * (c1 * (c4 * s1 * ((c2 * k) / 2.0f - (k * s3 * s4) / 2.0f) + c5 * s2 * ((c2 * k) / 2.0f - (k * s3 * s4) / 2.0f))
                            + s5 * (c4 * c5 * ((c2 * k) / 2.0f - (k * s3 * s4) / 2.0f) - s1 * s2 * ((c2 * k) / 2.0f - (k * s3 * s4) / 2.0f)))
            + yf
                    * (s6 * ((k * s4) / 2.0f + (c2 * k * s3) / 2.0f)
                            - c6
                                    * (c1 * (c4 * c5 * ((c2 * k) / 2.0f - (k * s3 * s4) / 2.0f) - s1 * s2 * ((c2 * k) / 2.0f - (k * s3 * s4) / 2.0f))
                                            - s5 * (c4 * s1 * ((c2 * k) / 2.0f - (k * s3 * s4) / 2.0f) + c5 * s2 * ((c2 * k) / 2.0f - (k * s3 * s4) / 2.0f))))
            + zf
                    * (c6 * ((k * s4) / 2.0f + (c2 * k * s3) / 2.0f)
                            + s6
                                    * (c1 * (c4 * c5 * ((c2 * k) / 2.0f - (k * s3 * s4) / 2.0f) - s1 * s2 * ((c2 * k) / 2.0f - (k * s3 * s4) / 2.0f))
                                            - s5 * (c4 * s1 * ((c2 * k) / 2.0f - (k * s3 * s4) / 2.0f) + c5 * s2 * ((c2 * k) / 2.0f - (k * s3 * s4) / 2.0f))))
            + c4 * thigh * ((c2 * k) / 2.0f - (k * s3 * s4) / 2.0f);
    float j31 = tibia * (c5 * (s2 * ((k * s4) / 2.0f - (c2 * k * s3) / 2.0f) + (c3 * c4 * k) / 2.0f) + s1 * (c4 * ((k * s4) / 2.0f - (c2 * k * s3) / 2.0f) - (c3 * k * s2) / 2.0f))
            - xf
                    * (c1
                            * (c5 * (c4 * ((k * s4) / 2.0f - (c2 * k * s3) / 2.0f) - (c3 * k * s2) / 2.0f)
                                    - s1 * (s2 * ((k * s4) / 2.0f - (c2 * k * s3) / 2.0f) + (c3 * c4 * k) / 2.0f))
                            - s5
                                    * (c5 * (s2 * ((k * s4) / 2.0f - (c2 * k * s3) / 2.0f) + (c3 * c4 * k) / 2.0f)
                                            + s1 * (c4 * ((k * s4) / 2.0f - (c2 * k * s3) / 2.0f) - (c3 * k * s2) / 2.0f)))
            + thigh * (s2 * ((k * s4) / 2.0f - (c2 * k * s3) / 2.0f) + (c3 * c4 * k) / 2.0f)
            - c6 * yf
                    * (c1
                            * (c5 * (s2 * ((k * s4) / 2.0f - (c2 * k * s3) / 2.0f) + (c3 * c4 * k) / 2.0f)
                                    + s1 * (c4 * ((k * s4) / 2.0f - (c2 * k * s3) / 2.0f) - (c3 * k * s2) / 2.0f))
                            + s5
                                    * (c5 * (c4 * ((k * s4) / 2.0f - (c2 * k * s3) / 2.0f) - (c3 * k * s2) / 2.0f)
                                            - s1 * (s2 * ((k * s4) / 2.0f - (c2 * k * s3) / 2.0f) + (c3 * c4 * k) / 2.0f)))
            + s6 * zf
                    * (c1
                            * (c5 * (s2 * ((k * s4) / 2.0f - (c2 * k * s3) / 2.0f) + (c3 * c4 * k) / 2.0f)
                                    + s1 * (c4 * ((k * s4) / 2.0f - (c2 * k * s3) / 2.0f) - (c3 * k * s2) / 2.0f))
                            + s5
                                    * (c5 * (c4 * ((k * s4) / 2.0f - (c2 * k * s3) / 2.0f) - (c3 * k * s2) / 2.0f)
                                            - s1 * (s2 * ((k * s4) / 2.0f - (c2 * k * s3) / 2.0f) + (c3 * c4 * k) / 2.0f)));
    float j32 = -tibia * (c4 * c5 * ((c2 * k) / 2.0f + (k * s3 * s4) / 2.0f) - s1 * s2 * ((c2 * k) / 2.0f + (k * s3 * s4) / 2.0f))
            - xf
                    * (c1 * (c4 * s1 * ((c2 * k) / 2.0f + (k * s3 * s4) / 2.0f) + c5 * s2 * ((c2 * k) / 2.0f + (k * s3 * s4) / 2.0f))
                            + s5 * (c4 * c5 * ((c2 * k) / 2.0f + (k * s3 * s4) / 2.0f) - s1 * s2 * ((c2 * k) / 2.0f + (k * s3 * s4) / 2.0f)))
            - yf
                    * (s6 * ((k * s4) / 2.0f - (c2 * k * s3) / 2.0f)
                            - c6
                                    * (c1 * (c4 * c5 * ((c2 * k) / 2.0f + (k * s3 * s4) / 2.0f) - s1 * s2 * ((c2 * k) / 2.0f + (k * s3 * s4) / 2.0f))
                                            - s5 * (c4 * s1 * ((c2 * k) / 2.0f + (k * s3 * s4) / 2.0f) + c5 * s2 * ((c2 * k) / 2.0f + (k * s3 * s4) / 2.0f))))
            - zf
                    * (c6 * ((k * s4) / 2.0f - (c2 * k * s3) / 2.0f)
                            + s6
                                    * (c1 * (c4 * c5 * ((c2 * k) / 2.0f + (k * s3 * s4) / 2.0f) - s1 * s2 * ((c2 * k) / 2.0f + (k * s3 * s4) / 2.0f))
                                            - s5 * (c4 * s1 * ((c2 * k) / 2.0f + (k * s3 * s4) / 2.0f) + c5 * s2 * ((c2 * k) / 2.0f + (k * s3 * s4) / 2.0f))))
            - c4 * thigh * ((c2 * k) / 2.0f + (k * s3 * s4) / 2.0f);
    float xbe = e.x;
    float ybe = e.y;
    float zbe = e.z;
    float lambda = 0.4f;
    float la2 = lambda * lambda;
    float la4 = la2 * la2;
    float j322 = j32 * j32;
    float j222 = j22 * j22;
    float j122 = j12 * j12;
    float j212 = j21 * j21;
    float j112 = j11 * j11;
    float j312 = j31 * j31;
    float sigma = 1.0f
            / (la4 + j112 * j222 + j122 * j212 + j112 * j322 + j122 * j312 + j212 * j322 + j222 * j312 + j112 * la2 + j122 * la2 + j212 * la2 + j222 * la2 + j312 * la2 + j322 * la2
                    - 2.0f * j11 * j12 * j21 * j22 - 2.0f * j11 * j12 * j31 * j32 - 2.0f * j21 * j22 * j31 * j32);
    result.Hp = sigma * xbe * (j11 * j222 + j11 * j322 + j11 * la2 - j12 * j21 * j22 - j12 * j31 * j32)
            + sigma * ybe * (j122 * j21 + j21 * j322 + j21 * la2 - j11 * j12 * j22 - j22 * j31 * j32)
            + sigma * zbe * (j122 * j31 + j222 * j31 + j31 * la2 - j11 * j12 * j32 - j21 * j22 * j32);
    result.Hr = sigma * xbe * (j12 * j212 + j12 * j312 + j12 * la2 - j11 * j21 * j22 - j11 * j31 * j32)
            + sigma * ybe * (j112 * j22 + j22 * j312 + j22 * la2 - j11 * j12 * j21 - j21 * j31 * j32)
            + sigma * zbe * (j112 * j32 + j212 * j32 + j32 * la2 - j11 * j12 * j31 - j21 * j22 * j31);
    return result;
}
