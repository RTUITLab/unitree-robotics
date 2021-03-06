/************************************************************************
Copyright (c) 2020, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <math.h>
#include <iostream>

using namespace UNITREE_LEGGED_SDK;

class Custom
{
public:
    Custom(): control(LeggedType::A1), udp(LOWLEVEL) {
        udp.InitCmdData(cmd);
    }
    void UDPRecv();
    void UDPSend();
    void RobotControl();

    Safety control;
    UDP udp;
    LowCmd cmd = {0};
    LowState state = {0};
    int Tpi = 0;
    int motiontime = 0;
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

    cmd.motorCmd[FR_0].tau = -0.65f;
    cmd.motorCmd[FL_0].tau = +0.65f;
    cmd.motorCmd[RR_0].tau = -0.65f;
    cmd.motorCmd[RL_0].tau = +0.65f;

    if( motiontime >= 500){
        float speed = 1 * sin(3*M_PI*Tpi/1000.0);
        for (int i = 0; i < 4; ++i) {
            cmd.motorCmd[i*3+0].q = PosStopF;
            cmd.motorCmd[i*3+0].dq = 0;
            cmd.motorCmd[i*3+0].Kp = 0;
            cmd.motorCmd[i*3+0].Kd = 4;
            cmd.motorCmd[i*3+0].tau = 0.0f;

//            cmd.motorCmd[i*3+2].q = PosStopF;
//            cmd.motorCmd[i*3+2].dq = speed;
//            cmd.motorCmd[i*3+2].Kp = 0;
//            cmd.motorCmd[i*3+2].Kd = 4;
//            cmd.motorCmd[i*3+2].tau = 0.0f;
        }
        Tpi++;
    }

    if(motiontime > 10){
        control.PowerProtect(cmd, state, 1);
        control.PositionProtect(cmd, state, 0.087);
    }
    udp.SetSend(cmd);
}

int main(void)
{
    std::cout << "Control level is set to LOW-level." << std::endl
              << "WARNING: Make sure the robot is hung up." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    Custom custom;

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
