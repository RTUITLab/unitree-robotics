#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <math.h>
#include <iostream>
#include <unistd.h>
#include <utility>
#include <vector>
#include <fstream>

using namespace UNITREE_LEGGED_SDK;

class Motion {
public:
    int duration;
    int pauseAfter;
    double targetPositions[12]{};

    Motion(int duration, int pauseAfter, const double * targetPositions) {
        this->duration = duration;
        this->pauseAfter = pauseAfter;
        for (int i = 0; i < 12; i++)
            this->targetPositions[i] = targetPositions[i];
    }
};

class Custom
{
public:
    Custom(): control(LeggedType::A1), udp(LOWLEVEL) {
        udp.InitCmdData(cmd);
    }
    void UDPSend();
    void UDPRecv();
    void RobotControl();
    void SetMotionSeq(std::vector<Motion*> newMotionSeq);
    void MoveOneLeg(int startTime, int endTime, int legIndex, const double * pos);

    Safety control;
    UDP udp;
    LowCmd cmd = {0};
    LowState state = {0};

    std::vector<Motion *> motionSeq;

    int curMotionID;
    Motion * curMotion;
    int cmBeginTime;
    int cmEndTime;

    int motiontime = 0;

    double curMotionTGTDeltas[12];
    double curMotionDeltas[12];
    double curMotionAcc[12];
    int curMotionAccTime;
    double cmstart;

    float dt = 0.002;     // 0.001~0.01
};

void Custom::UDPRecv()
{
    udp.Recv();
}

void Custom::UDPSend()
{
    udp.Send();
}

void Custom::SetMotionSeq(std::vector<Motion*> newMotionSeq) {
    this->motionSeq = std::move(newMotionSeq);
}

void Custom::RobotControl()
{
    motiontime++;
    udp.GetRecv(state);
    // gravity compensation

    int currentDebug = 0;

    if (motiontime == 1) {
        for (int i = 0; i < 4; i++) {
            cmd.motorCmd[i*3+0].Kp = 70;
            cmd.motorCmd[i*3+0].Kd = 3;
            cmd.motorCmd[i*3+1].Kp = 180;
            cmd.motorCmd[i*3+1].Kd = 8;
            cmd.motorCmd[i*3+2].Kp = 300;
            cmd.motorCmd[i*3+2].Kd = 15;
        }
        for (int i = 0; i < 12; i++){
            cmd.motorCmd[i].mode = 0x0A;
            cmd.motorCmd[i].dq = 0;
            cmd.motorCmd[i].tau = 0;
        }

        curMotionID = 0;
        curMotion = motionSeq[curMotionID];
        cmBeginTime = 500;
        cmEndTime = cmBeginTime + curMotion->duration + curMotion->pauseAfter;
    }

    if (motiontime < 500) {
        control.PowerProtect(cmd, state, 7);
        udp.SetSend(cmd);
        return;
    }

    if (motiontime > cmEndTime && curMotionID < motionSeq.size()-1) {
        curMotionID++;
        curMotion = motionSeq[curMotionID];
        cmBeginTime = motiontime-1;
        cmEndTime = cmBeginTime + curMotion->duration + curMotion->pauseAfter;
    }

    if ((motiontime > cmBeginTime) && (motiontime <= cmEndTime - curMotion->pauseAfter)) {
        for (int i = 0; i < 4; i++) {
            MoveOneLeg(cmBeginTime,
                       cmEndTime - curMotion->pauseAfter,
                       i,
                       motionSeq[curMotionID]->targetPositions);
        }
    }



//    if (motiontime % 100 == 0) {
//        for (int i = 0; i < 3; i++) {
//            std::cout << state.motorState[i].q << " ";
//        }
//        std::cout << "\n";
//    }

    control.PowerProtect(cmd, state, 8);

    udp.SetSend(cmd);
}

void Custom::MoveOneLeg(int startTime, int endTime, int legIndex, const double * pos) {
    if (motiontime == startTime + 1) {
        for (int i = 3 * legIndex; i < 3 * legIndex + 3; i++) {
            cmstart = state.motorState[i].q;
            curMotionTGTDeltas[i] = (pos[i] - cmstart) / (endTime - startTime);
            cmd.motorCmd[i].q = state.motorState[i].q;
        }
    }
    for (int i = 3 * legIndex; i < 3 * legIndex + 3; i++) {
        cmd.motorCmd[i].q += curMotionTGTDeltas[i];
    }
}

//template<typename T>
//void tryToReadVal(T * desiredVal) {
//
//}

std::vector<Motion*> getMotionSeqFromFile(const std::string& filename) {

    std::fstream fin;
    fin.open("config/" + filename + ".txt", std::ios::in);

    if (!fin) {
        std::cout << "\nERROR: Wrong file name!\n\n";
        throw std::exception();
    }

    int duration, pauseAfter;
    double targetPositions[12];
    std::vector<Motion*> motionSeq;

    while (!fin.eof()) {
        fin >> duration;

        fin >> pauseAfter;

        for (double & targetPosition : targetPositions)
            fin >> targetPosition;

        auto newMotion = new Motion(duration, pauseAfter, targetPositions);
        motionSeq.push_back(newMotion);
    }

    fin.close();
    return motionSeq;
}


int main(int argc, const char** argv)
{
    std::cout << "Communication level is set to LOW-level." << std::endl
              << "WARNING: Make sure the robot is hung up." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    Custom custom;

    std::string filename;
    if (argv[1] != nullptr){
        std::string firstArg(argv[1]);
        filename = firstArg;
    } else {
        std::cout << "\nERROR: Specify config file name!\n\n";
        throw std::exception();
    }

    std::vector<Motion*> motionSeq;
    motionSeq = getMotionSeqFromFile(filename);
    custom.SetMotionSeq(motionSeq);

    // InitEnvironment();
    LoopFunc loop_control("control_loop", custom.dt,    boost::bind(&Custom::RobotControl, &custom));
    LoopFunc loop_udpSend("udp_send",     custom.dt, 3, boost::bind(&Custom::UDPSend,      &custom));
    LoopFunc loop_udpRecv("udp_recv",     custom.dt, 3, boost::bind(&Custom::UDPRecv,      &custom));

    loop_udpSend.start();
    loop_udpRecv.start();
    loop_control.start();

    while(1){
        sleep(10);
    };

    return 0;
}