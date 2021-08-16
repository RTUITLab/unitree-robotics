#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <math.h>
#include <iostream>
#include <unistd.h>

using namespace UNITREE_LEGGED_SDK;

class Custom
{
public:
    Custom(): control(LeggedType::A1, LOWLEVEL), udp() {
        control.InitCmdData(cmd);
    }
    void UDPSend();
    void UDPRecv();
    void RobotControl();

    Control control;
    UDP udp;
    LowCmd cmd = {0};
    LowState state = {0};
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
    // printf("%d\n", motiontime);
    // gravity compensation

    if (motiontime == 1) {
        for(int i=0; i<4; i++){
            cmd.motorCmd[i*3+0].mode = 0x0A;
            cmd.motorCmd[i*3+0].Kp = 0;
            cmd.motorCmd[i*3+0].dq = 0;
            cmd.motorCmd[i*3+0].Kd = 0;
            cmd.motorCmd[i*3+0].tau = 0;
            cmd.motorCmd[i*3+1].mode = 0x0A;
            cmd.motorCmd[i*3+1].Kp = 10;
            cmd.motorCmd[i*3+1].dq = 0;
            cmd.motorCmd[i*3+1].Kd = 10;
            cmd.motorCmd[i*3+1].tau = 0;
            cmd.motorCmd[i*3+2].mode = 0x0A;
            cmd.motorCmd[i*3+2].Kp = 5;
            cmd.motorCmd[i*3+2].dq = 0;
            cmd.motorCmd[i*3+2].Kd = 1;
            cmd.motorCmd[i*3+2].tau = 0;
        }
        for(int i=0; i<12; i++){
            cmd.motorCmd[i].q = state.motorState[i].q;
        }
    }
    if ((motiontime > 500) && (motiontime <= 1500)) {
        double pos[12] = {0.0, 0.67, -1.3, -0.0, 0.67, -1.3,
                          0.0, 0.67, -1.3, -0.0, 0.67, -1.3};
        double lastPos[12], percent;
        for(int j=0; j<12; j++) lastPos[j] = state.motorState[j].q;
        percent = (double)(motiontime-500)/1000;
        for(int j=0; j<12; j++){
            cmd.motorCmd[j].q = lastPos[j]*(1-percent) + pos[j]*percent;
            if (motiontime % 100 == 0) {
                for (int i = 0; i < 12; i++) {
                    std::cout << state.motorState[i].q << " ";
                }
                std::cout << "\n";
                for (int i = 0; i < 12; i++) {
                    std::cout << cmd.motorCmd[i].q << " ";
                }
                std::cout << "\n";
            }
        }
    }

    control.PowerProtect(cmd, state, 1);

    udp.SetSend(cmd);
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