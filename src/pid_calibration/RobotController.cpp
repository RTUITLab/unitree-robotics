#include "RobotController.hpp"

void RobotController::UDPRecv()
{
    udp.Recv();
}

void RobotController::UDPSend()
{
    udp.Send();
}

void RobotController::SetMotionSeq(std::vector<Motion*> newMotionSeq)
{
    this->motionSeq = std::move(newMotionSeq);
}

void RobotController::RobotControl()
{
    // Set next tick
    motiontime++;

    // Get current state from robot
    udp.GetRecv(state);

    int currentDebug = 0;

    // Init robot parameters
    if (motiontime == 1)
    {
        // Init Kp, Kd gains for each servo
        for (int i = 0; i < 4; i++)
        {
            cmd.motorCmd[i*3+0].Kp = 270;
            cmd.motorCmd[i*3+0].Kd = 3;
            cmd.motorCmd[i*3+1].Kp = 180;
            cmd.motorCmd[i*3+1].Kd = 8;
            cmd.motorCmd[i*3+2].Kp = 300;
            cmd.motorCmd[i*3+2].Kd = 15;
        }

        // Init position mode for every servo
        for (int i = 0; i < 12; i++)
        {
            cmd.motorCmd[i].mode = 0x0A;
            cmd.motorCmd[i].dq = 0;
            cmd.motorCmd[i].tau = 0;
        }

        currMotionId = 0;
        currMotion = motionSeq[currMotionId];
        currMotionBeginTime = 500;
        currMotionEndTime = currMotionBeginTime + currMotion->duration + currMotion->pauseAfter;
    }

    // Wait 500 ticks before start executiong motion sequence
    if (motiontime < 500)
    {
        safe.PowerProtect(cmd, state, 7);
        udp.SetSend(cmd);
        return;
    }

    // Finish not last motion
    if (motiontime > currMotionEndTime && currMotionId < motionSeq.size() - 1)
    {
        currMotionId++;
        currMotion = motionSeq[currMotionId];
        currMotionBeginTime = motiontime-1;
        currMotionEndTime = currMotionBeginTime + currMotion->duration + currMotion->pauseAfter;
    }

    // Process motion
    if ((motiontime > currMotionBeginTime) && (motiontime <= currMotionEndTime - currMotion->pauseAfter))
    {
        for (int i = 0; i < 4; i++)
        {
            MoveOneLeg(currMotionBeginTime,
                       currMotionEndTime - currMotion->pauseAfter,
                       i,
                       motionSeq[currMotionId]->targetPositions);
        }
    }

    safe.PowerProtect(cmd, state, 8);

    // Send commant to robot
    udp.SetSend(cmd);

    // Execute callback function if setted
    if (callback != nullptr && motiontime > 500)
    {
        callback(state, cmd);
    }

    if (currMotionId == motionSeq.size() - 1)
    {
        isFinished = true;
    }
}

void RobotController::MoveOneLeg(int startTime, int endTime, int legIndex, const double * targetPos)
{
    // Count deltas for new motion
    if (motiontime == startTime + 1)
    {
        double motorPosition;
        for (int i = 3 * legIndex; i < 3 * legIndex + 3; i++)
        {
            motorPosition = state.motorState[i].q;
            currMotionTargetDeltas[i] = (targetPos[i] - motorPosition) / (endTime - startTime);
            cmd.motorCmd[i].q = state.motorState[i].q;
        }
    }

    // Count position of each servo of leg for current tick
    for (int i = 3 * legIndex; i < 3 * legIndex + 3; i++)
    {
        cmd.motorCmd[i].q += currMotionTargetDeltas[i];
    }
}

void RobotController::SetCallback(void (*cb) (UNITREE_LEGGED_SDK::LowState, UNITREE_LEGGED_SDK::LowCmd))
{
    callback = cb;
}
