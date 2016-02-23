#include <iostream>
#include <cmath>
#include "NaoMovement.h"

NaoMovement::NaoMovement(const string ip, const int port, bool local): posture(ip, port), motion(ip, port) {
    naoPositionOnLane = CENTER; // For default, the Nao is in the center of the lane.
    this->ip = ip;
    this->port = port;
    this->local = local;
}

// Establish the position in Crouch and then in StandInit.
void NaoMovement::initialPositionIndividualRace() {
    posture.goToPosture("Crouch", 0.5);
    posture.goToPosture("StandInit", 1.0);

    if (!local)
        cout << "Stand" << endl;
}

void NaoMovement::initialPositionRelayRace() {
    posture.goToPosture("Crouch", 0.5);
    motion.angleInterpolation("HeadYaw", -2.0f, 1.0f, true);

    if (!local)
        cout << "Stand" << endl;
}

// Given an angle in degrees, move the NAO in straight mode.
void NaoMovement::moveInIndividualRace(double angleInDegrees) {
    if (angleInDegrees == 90) {
        naoPositionOnLane = CENTER;
    } else if (angleInDegrees < 90) {
        naoPositionOnLane = RIGHT;       // Nao is on the right side.
    } else if (angleInDegrees > 90) {
        naoPositionOnLane = LEFT;        // Nao is on the left side.
    }

    motion.move(linearVelocity(angleInDegrees), 0, angularVelocity(angleInDegrees),walkParameters());

    if (!local){
        cout << "VelLin: " << linearVelocity(angleInDegrees) << endl;
        cout << "VelAng: " << angularVelocity(angleInDegrees) << endl;
        cout << "Theta: " << angleInDegrees << endl;
        cout << "--------------------------------" << endl;
    }
}

// Takes the NAO to a position just before the goal.
void NaoMovement::naoOnGoal() {
    if (!local)
        cout << "Nao on goal!" << endl;

    motion.moveTo(0.6, 0, 0, walkParameters());
}

// Establish the position in Crouch and set Stiffnesses to body.
void NaoMovement::stop() {
    motion.stopMove();
    posture.goToPosture("Crouch", 0.5);
    motion.setStiffnesses("Body", 0);

    if (!local)
        cout << "Stop" << endl;
}

// Helper methods.

// v = vmax * e^(-k*abs(theta - 90))
double NaoMovement::linearVelocity(double theta){
    double vMax = 0.85;
    const double k1 = 1.0 / 40;     // k1 linear velocity for the right side.
    const double k2 = 1.0 / 20;     // k2 linear velocity for the left side. // 1/20

    if ((naoPositionOnLane == RIGHT && theta >= 70 && theta <= 90) || (naoPositionOnLane == LEFT && theta >= 90 && theta <= 110))
        vMax = 0.65;

    return vMax * exp(-(theta > 90 ? k2 : k1) * abs(theta - 90));
}

// w = wmax * ( 1 - e^(-k*abs(theta - 90)))*N if (theta > 90) (N = -1) else (N = 1)
double NaoMovement::angularVelocity(double theta){
    double wMax = 0.25;
    const double k1 = 1.0 / 10;     // k1 right to left correction
    const double k2 = 1.0 / 10;     // k2 left to right correction

    if ((naoPositionOnLane == RIGHT && theta >= 70 && theta <= 90) || (naoPositionOnLane == LEFT && theta >= 90 && theta <= 110))
        wMax = 0.45;

    return pow(-1, theta > 90) * (wMax * (1 - exp(-(theta > 90 ? k2 : k1) * abs(theta - 90))));
}

// Enhance the walking parameters to increase the speed.
AL::ALValue NaoMovement::walkParameters() {
   return  AL::ALValue::array(AL::ALValue::array("MaxStepX",0.08),AL::ALValue::array("MaxStepY",0.14),
                              AL::ALValue::array("MaxStepTheta",0.4),AL::ALValue::array("MaxStepFrequency",0.5), //Frec 0.5
                              AL::ALValue::array("StepHeight",0.04),AL::ALValue::array("TorsoWx",0.0),
                              AL::ALValue::array("TorsoWy",0));
}
