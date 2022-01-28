#ifndef RTUITLAB_ROBOT_CONTROLLER_H

#define RTUITLAB_ROBOT_CONTROLLER_H

#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "Motion.hpp"

/**
 * @brief Class realizing low-level robot controller. Using UDP connection to comunicate with robot.
 * 
 */
class RobotController
{
public:
    RobotController(): safe(UNITREE_LEGGED_SDK::LeggedType::A1), udp(UNITREE_LEGGED_SDK::LOWLEVEL) {
        udp.InitCmdData(cmd);
    }

    // Sending command to robot
    void UDPSend();

    // Receiving command from robot
    void UDPRecv();

    // Callback, calling after every tick finishing 
    void RobotControl();

    /**
     * @brief Setting motion sequence
     * 
     * @param newMotionSeq Vector of motions that will be applied to robot
     */
    void SetMotionSeq(std::vector<Motion*> newMotionSeq);

    /**
     * @brief Setting position of every servo for one leg.
     * Counting target position for current tick as a linear dependence of target position from ticks.
     * 
     * @param startTime Tick when motion started
     * @param endTime Tick when motion ends (without pauseAfter)
     * @param legIndex Index of moving leg
     * @param targetPos Motion target position
     */
    void MoveOneLeg(int startTime, int endTime, int legIndex, const double * targetPos);

    /**
     * @brief Set callback that will be called after every tick
     * 
     * @param cb callback function (params: low state and low cmd) 
     */
    void SetCallback(void cb (UNITREE_LEGGED_SDK::LowState, UNITREE_LEGGED_SDK::LowCmd));

    // Tools to protect robot (position, power protection, ect)
    UNITREE_LEGGED_SDK::Safety safe;

    // UDP channel
    UNITREE_LEGGED_SDK::UDP udp;

    // Low level command to send to robot
    UNITREE_LEGGED_SDK::LowCmd cmd = {0};

    // Low level state received from robot
    UNITREE_LEGGED_SDK::LowState state = {0};

    /**
     * @brief Motion sequence that will be applied to robot
     * 
     * @todo Move to private
     */
    std::vector<Motion *> motionSeq;

    // Number of moution in vector
    int currMotionId;

    // Executiong motion
    Motion * currMotion;

    // Tick when current motion begins
    int currMotionBeginTime;

    // Tick when current motion ends (including pauseAfter)
    int currMotionEndTime;

    // Ticks counter
    int motiontime = 0;

    // Position deltas tht will be added to current position duriong each tick of current motion
    double currMotionTargetDeltas[12];

    // Tick duration (0.001-0.01 ms)
    float dt = 0.002;

    // Flag showing that motion sequence is completed
    bool isFinished = false;

private:
    void (*callback) (UNITREE_LEGGED_SDK::LowState, UNITREE_LEGGED_SDK::LowCmd);
};

#endif
