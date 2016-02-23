#include <alvision/alimage.h>
#include <alproxies/almotionproxy.h>
#include <alproxies/alrobotpostureproxy.h>
#include <alproxies/alvideodeviceproxy.h>
#include <alvision/alvisiondefinitions.h>

using namespace std;
using namespace AL;

class NaoMovement {
public:
    enum NaoPositionOnLane {LEFT, CENTER, RIGHT};

    NaoMovement(const string ip, const int port, bool local);
    void initialPositionIndividualRace();
    void initialPositionRelayRace();
    void moveInIndividualRace(double angleInDegrees);
    void naoOnGoal();
    void stop();

private:
    AL::ALRobotPostureProxy posture;     // Posture Proxy
    AL::ALMotionProxy motion;            // Motion Proxy
    NaoPositionOnLane naoPositionOnLane; // Variable that determines where is the Nao on the lane.

    bool local;             // Flag for the execution type (local or remote).
    int port;
    string ip;

    double linearVelocity(double theta);
    double angularVelocity(double theta);
    AL::ALValue walkParameters();
};
