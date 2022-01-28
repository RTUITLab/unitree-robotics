#include "Motion.hpp"

Motion::Motion(int duration, int pauseAfter, const double * targetPositions) {
    this->duration = duration;
    this->pauseAfter = pauseAfter;
    for (int i = 0; i < 12; i++)
        this->targetPositions[i] = targetPositions[i];
}
