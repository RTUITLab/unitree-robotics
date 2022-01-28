#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <math.h>
#include <iostream>
#include <unistd.h>
#include <vector>

using namespace UNITREE_LEGGED_SDK;

class Custom
{
public:
    Custom(): control(LeggedType::A1), udp(LOWLEVEL) {
        udp.InitCmdData(cmd);
    }
    void UDPSend();
    void UDPRecv();
    void RobotControl();
    void MoveOneLeg(int startTime, int endTime, int legIndex, const double * pos);

    Safety control;
    UDP udp;
    LowCmd cmd = {0};
    LowState state = {0};
    int motiontime = 0;
    double curMotionTGTDeltas[12];
    double cmstart;
    bool flag = false;
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

void Custom::RobotControl()
{
    motiontime++;
    udp.GetRecv(state);
    // printf("%d\n", motiontime);
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
        //cmd.motorCmd[2].q = state.motorState[2].q;
    }
    if ((motiontime > 500) && (motiontime <= 1500)) {
        double pos[12] = {0.0, 0.67, -1.3, -0.0, 0.67, -1.3,
                          0.0, 0.67, -1.3, -0.0, 0.67, -1.3};
        for (int i = 0; i < 4; i++) {
            MoveOneLeg(500, 1500, i, pos);
        }
    }

    if ((motiontime > 2000) && (motiontime <= 2500)) {
        double pos[12] = {0.0, 1.6, -2.3, -0.0, 1.6, -2.3,
                           0.0, 1.6, -2.3, -0.0, 1.6, -2.3};
        for (int i = 0; i < 4; i++) {
            MoveOneLeg(2000, 2500, i, pos);
        }
    }

    int jumpEnd = 3080;
    int standDelta = 1000;

    if ((motiontime > 3000) && (motiontime <= jumpEnd)) {
        double pos[12] = {-0.0, 0.34, -0.83, -0.0, 0.34, -0.83,
                          0.0, 0.34, -0.83, -0.0, 0.34, -0.83};
        for (int i = 0; i < 4; i++) {
            MoveOneLeg(3000, jumpEnd, i, pos);
        }
    }

    if ((motiontime > jumpEnd) && (motiontime <= jumpEnd + standDelta)) {
        double pos[12] = {0.0, 0.67, -1.3, -0.0, 0.67, -1.3,
                          0.0, 0.67, -1.3, -0.0, 0.67, -1.3};
        for (int i = 0; i < 4; i++) {
            MoveOneLeg(jumpEnd, jumpEnd + standDelta, i, pos);
        }
    }

    if ((motiontime > 4500) && (motiontime <= 5500)) {
        double pos[12] = {0.0, 1.4, -2.5, -0.0, 1.4, -2.5,
                          0.0, 1.4, -2.5, -0.0, 1.4, -2.5};
        for (int i = 0; i < 4; i++) {
            MoveOneLeg(4500, 5500, i, pos);
        }
    }


//    if ((motiontime > 3500) && (motiontime <= 4500)) {
//        double pos[12] = {0.0, 0.67, -1.3, -0.0, 0.67, -1.3,
//                          0.0, 0.67, -1.3, -0.0, 0.67, -1.3};
//        for (int i = 0; i < 4; i++) {
//            MoveOneLeg(3500, 4500, i, pos);
//        }
//    }


    if (motiontime % 100 == 0) {
        for (int i = 0; i < 3; i++) {
            std::cout << state.motorState[i].q << " ";
        }
        std::cout << "\n";
    }

    control.PowerProtect(cmd, state, 7);

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

int main(void)
{
    std::cout << "Communication level is set to LOW-level." << std::endl
              << "WARNING: Make sure the robot is hung up." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    Custom custom;
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