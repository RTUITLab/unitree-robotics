#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <cmath>
#include <iostream>
#include <unistd.h>

using namespace UNITREE_LEGGED_SDK;
using namespace std;

class Custom {
public:
    Custom(): control(LeggedType::A1), udp(LOWLEVEL){
        udp.InitCmdData(cmd);
    }
    void UDPSend();
    void UDPRecv();
    void RobotControl();

    Safety control;
    UDP udp;
    LowCmd cmd = {0};
    LowState state = {0};
    int motiontime = 0;
    float dt = 0.002;     // 0.001~0.01
};

void Custom::UDPRecv() { 
    udp.Recv();
}

void Custom::UDPSend() {  
    udp.Send();
}

double sqr_err = 0;
double err = 0;

double targetPos = 0;

void Custom::RobotControl() {
    motiontime++;
    udp.GetRecv(state);
    
    // gravity compensation
    cmd.motorCmd[FR_0].tau = -0.65f;
    cmd.motorCmd[FL_0].tau = +0.65f;
    cmd.motorCmd[RR_0].tau = -0.65f;
    cmd.motorCmd[RL_0].tau = +0.65f;

    cmd.motorCmd[FR_1].q = 0;

    if (motiontime == 499) {
        targetPos = state.motorState[FR_1].q + 0.25;
    }

    if(motiontime >= 500  && motiontime < 501){
        float torque = (0 - state.motorState[FR_1].q)*10.0f + (0 - state.motorState[FR_1].dq)*1.0f;

        if (motiontime % 100 == 0) {
            cout << motiontime << endl
                << "Calculated torque: " << torque << endl
                << "Angle:             " << state.motorState[FR_1].q << endl
                << "Velocity:          " << state.motorState[FR_1].dq << endl
                << "Acceleration:      " << state.motorState[FR_1].ddq << endl 
                << "Error:             " << (state.motorState[FR_1].q - targetPos) << endl << endl;
        }

        // if(torque > 5.0f) torque = 5.0f;
        // if(torque < -5.0f) torque = -5.0f;

        // cmd.motorCmd[FR_1].q = PosStopF;
        // cmd.motorCmd[FR_1].dq = VelStopF;
        // cmd.motorCmd[FR_1].Kp = 0;
        // cmd.motorCmd[FR_1].Kd = 0;
        // cmd.motorCmd[FR_1].tau = torque;

        sqr_err = sqr_err + (targetPos - state.motorState[FR_1].q) * (targetPos - state.motorState[FR_1].q);
        err = err + abs(targetPos - state.motorState[FR_1].q);

        cmd.motorCmd[FR_1].q = targetPos;
        cmd.motorCmd[FR_1].dq = 5;
        cmd.motorCmd[FR_1].Kp = 0;
        cmd.motorCmd[FR_1].Kd = 5;
        cmd.motorCmd[FR_1].tau = -0.0f;
    }

    if (motiontime == 550) {
        cout << motiontime << endl
             << "Cmd angle:         " << cmd.motorCmd[FR_1].q << endl
             << "Angle:             " << state.motorState[FR_1].q << endl
             << "Velocity:          " << state.motorState[FR_1].dq << endl
             << "Acceleration:      " << state.motorState[FR_1].ddq << endl 
             << "Error:             " << (state.motorState[FR_1].q - targetPos) << endl << endl;

        cout << "ERROR:     " << err / 2000 << endl;
        cout << "SQR ERROR: " << sqrt(sqr_err / 2000) << endl;
    }
    control.PowerProtect(cmd, state, 1);

    udp.SetSend(cmd);
}

int main(void) {
    cout << "Control level is set to LOW-level." << endl
              << "WARNING: Make sure the robot is hung up." << endl
              << "Press Enter to continue..." << endl;
    cin.ignore();

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
