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
    void setPosition(int joint, double target);

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

void Custom::setPosition(int joint, double target) {
    float torque = ((target - state.motorState[joint].q) * 5.0f + (0 - state.motorState[joint].dq) * 1.0f);
    cout << motiontime << ": " << torque << endl;
    if(torque > 5.0f) torque = 5.0f;
    if(torque < -5.0f) torque = -5.0f;

    cout << motiontime << ": " << torque << endl;

    cmd.motorCmd[joint].q = PosStopF;
    cmd.motorCmd[joint].dq = VelStopF;
    cmd.motorCmd[joint].Kp = 0;
    cmd.motorCmd[joint].Kd = 0;
    cmd.motorCmd[joint].tau = torque;
}

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
        targetPos = 0.6f;
    }

    if (motiontime == 1000) {
        targetPos = 1;
    }

    if (motiontime == 1234) {
        targetPos = -0.5;
    }

    if (motiontime == 1700) {
        targetPos = 0;
    }

    if(motiontime >= 500){

        // if (motiontime % 100 == 0) {
        //     cout << motiontime << endl
        //         << "Calculated torque: " << torque << endl
        //         << "Angle:             " << state.motorState[FR_1].q << endl
        //         << "Velocity:          " << state.motorState[FR_1].dq << endl
        //         << "Acceleration:      " << state.motorState[FR_1].ddq << endl 
        //         << "Error:             " << (state.motorState[FR_1].q - targetPos) << endl << endl;
        // }

        sqr_err = sqr_err + (targetPos - state.motorState[FR_1].q) * (targetPos - state.motorState[FR_1].q);
        err = err + abs(targetPos - state.motorState[FR_1].q);

        Custom::setPosition(FR_1, targetPos);
        Custom::setPosition(FL_1, targetPos);
        Custom::setPosition(RR_1, targetPos);
        Custom::setPosition(RL_1, targetPos);

        // cmd.motorCmd[FR_1].q = targetPos;
        // cmd.motorCmd[FR_1].dq = 5;
        // cmd.motorCmd[FR_1].Kp = 0;
        // cmd.motorCmd[FR_1].Kd = 5;
        // cmd.motorCmd[FR_1].tau = -0.0f;
    }

    if (motiontime == 2500) {
        cout << motiontime << endl
             << "Target:            " << targetPos << endl
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
