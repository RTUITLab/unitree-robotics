#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <math.h>
#include <iostream>
#include <stdio.h>
#include <stdint.h>

using namespace std;
using namespace UNITREE_LEGGED_SDK;

class Custom
{
public:
    Custom(): control(LeggedType::A1, LOWLEVEL), udp() {
        control.InitCmdData(cmd);
    }
    void UDPRecv();
    void UDPSend();
    void RobotControl();

    void printOneLegTemp(int legId);

    Control control;
    UDP udp;
    LowCmd cmd = {0};
    LowState state = {0};
    MotorState mState = {0};
    float qInit[12]={0};
    float qDes[3]={0};
    float sin_mid_q[3] = {0.0, 1.2, -2.0};
    float Kp[3] = {0};
    float Kd[3] = {0};
    double time_consume = 0;
    int rate_count = 0;
    int sin_count = 0;
    int motiontime = 0;
    float dt = 0.01;     // 0.001~0.01
};

void Custom::UDPRecv()
{
    udp.Recv();
}

void Custom::UDPSend()
{
    udp.Send();
}

double jointLinearInterpolation(double initPos, double targetPos, double rate)
{
    double p;
    rate = std::min(std::max(rate, 0.0), 1.0);
    p = initPos*(1-rate) + targetPos*rate;
    return p;
}

void Custom::RobotControl()
{
    motiontime++;
    udp.GetRecv(state);
    // printf("%d  %f\n", motiontime, state.motorState[FR_2].q);

    // gravity compensation
    cmd.motorCmd[FR_0].tau = -0.65f;
    cmd.motorCmd[FL_0].tau = +0.65f;
    cmd.motorCmd[RR_0].tau = -0.65f;
    cmd.motorCmd[RL_0].tau = +0.65f;

    // if( motiontime >= 100){
    if( motiontime >= 0){
        // first, get record initial position
        // if( motiontime >= 100 && motiontime < 500){
        if( motiontime >= 0 && motiontime < 10){

            for (int i = 0; i < 4; ++i) {
                qInit[i*3+0] = state.motorState[i*3+0].q;
                qInit[i*3+1] = state.motorState[i*3+1].q;
                qInit[i*3+2] = state.motorState[i*3+2].q;
            }
        }
        // second, move to the origin point of a sine movement with Kp Kd
        // if( motiontime >= 500 && motiontime < 1500){
        if( motiontime >= 10 && motiontime < 400){
            rate_count++;
            double rate = rate_count/200.0;                       // needs count to 200
            Kp[0] = 1.5; Kp[1] = 1.5; Kp[2] = 1.5;
            Kd[0] = 1.0; Kd[1] = 1.0; Kd[2] = 1.0;

            qDes[0] = jointLinearInterpolation(qInit[0], sin_mid_q[0], rate);
            qDes[1] = jointLinearInterpolation(qInit[1], sin_mid_q[1], rate);
            qDes[2] = jointLinearInterpolation(qInit[2], sin_mid_q[2], rate);
        }
        double sin_joint1, sin_joint2;
        // last, do sine wave
        if( motiontime >= 400){
            sin_count++;
            sin_joint1 = 0.6 * sin(3*M_PI*sin_count/1000.0);
            sin_joint2 = -0.6 * sin(1.8*M_PI*sin_count/1000.0);
            qDes[0] = sin_mid_q[0];
            qDes[1] = sin_mid_q[1];
            qDes[2] = sin_mid_q[2] + sin_joint2;
            // qDes[2] = sin_mid_q[2];
        }


        for (int i = 0; i < 4; ++i) {
            cmd.motorCmd[i*3+0].q = qDes[0];
            cmd.motorCmd[i*3+0].dq = 0;
            cmd.motorCmd[i*3+0].Kp = Kp[0];
            cmd.motorCmd[i*3+0].Kd = Kd[0];
            cmd.motorCmd[i*3+0].tau = state.motorState[i*3+0].tauEst;

            cmd.motorCmd[i*3+1].q = qDes[1];
            cmd.motorCmd[i*3+1].dq = 0;
            cmd.motorCmd[i*3+1].Kp = Kp[1];
            cmd.motorCmd[i*3+1].Kd = Kd[1];
            cmd.motorCmd[i*3+1].tau = state.motorState[i*3+1].tauEst;

            cmd.motorCmd[i*3+2].q =  qDes[2];
            cmd.motorCmd[i*3+2].dq = 0;
            cmd.motorCmd[i*3+2].Kp = Kp[2];
            cmd.motorCmd[i*3+2].Kd = Kd[2];
            cmd.motorCmd[i*3+2].tau = state.motorState[i*3+2].tauEst;

//            for (int j = 0; j < 4; j++)
//                cout << "ID:" << j << " T: " << state.footForce[j] - 0 << "\t";
//            cout << "\n";
            printOneLegTemp(i);

        }
    }

    if(motiontime > 10){
        control.PositionLimit(cmd);
        control.PowerProtect(cmd, state, 1);
        control.PositionProtect(cmd, state, 0.087);
    }


    udp.SetSend(cmd);

}

void Custom::printOneLegTemp(const int legId) {
    if (motiontime % 100 == 0) {
        for (int i = 0; i < 3; i++)
            cout << "ID:" << legId * 3 + i << " T: " << state.motorState[legId * 3 + i].temperature - 0 << "\t";
        cout << "\n";
    }
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
