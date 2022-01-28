// Include development kit for communication between PC and Controller board
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <math.h>
#include <iostream>
#include <vector>
#include <fstream>

#include "Motion.hpp"
#include "RobotController.hpp"
#include "CppCharts.hpp"

using namespace UNITREE_LEGGED_SDK;

// Vector collecting real servo position from robot (radians) 
std::vector<double> real1 { }, real2 { }, real3 = { };

// Vector collecting expected servo position for robot (radians)
std::vector<double> expected1 = { }, expected2 = { }, expected3 = { };

/**
 * @brief Read motions from file
 * 
 * @param filename Name of reading file. It should be at ./config with extension .txt (don't specify path and extension)
 * @return std::vector<Motion*> 
 * 
 * @throws std::exception Throws if file has wrong name
 */
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

// Callback to collect data for charts
void collectData(LowState state, LowCmd cmd)
{
    real1.push_back(state.motorState[0].q);
    real2.push_back(state.motorState[1].q);
    real3.push_back(state.motorState[2].q);

    expected1.push_back(cmd.motorCmd[0].q);
    expected2.push_back(cmd.motorCmd[1].q);
    expected3.push_back(cmd.motorCmd[2].q);
}

int main(int argc, const char** argv)
{
    std::cout << "Communication level is set to LOW-level." << std::endl
              << "WARNING: Make sure the robot is hung up." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    RobotController robotController;

    // Set callback to collect data for charts
    robotController.SetCallback(collectData);

    // Read file specified as a cmd parameter
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
    robotController.SetMotionSeq(motionSeq);

    // InitEnvironment();
    LoopFunc loop_control("control_loop", robotController.dt,    boost::bind(&RobotController::RobotControl, &robotController));
    LoopFunc loop_udpSend("udp_send",     robotController.dt, 3, boost::bind(&RobotController::UDPSend,      &robotController));
    LoopFunc loop_udpRecv("udp_recv",     robotController.dt, 3, boost::bind(&RobotController::UDPRecv,      &robotController));

    loop_udpSend.start();
    loop_udpRecv.start();
    loop_control.start();

    while(!robotController.isFinished){
        sleep(10);
    };

    loop_udpSend.shutdown();
    loop_udpRecv.shutdown();
    loop_control.shutdown();

    std::vector< std::vector<double> > data1 = { real1, expected1 };
    std::vector< std::vector<double> > data2 = { real2, expected2 };
    std::vector< std::vector<double> > data3 = { real3, expected3 };

    // drawChart(data1, (long) 0);
    drawChart(data2, (long) 1);
    drawChart(data3, (long) 2);

    return 0;
}