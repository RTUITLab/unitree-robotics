#ifndef RTUITLAB_MOTION_H

#define RTUITLAB_MOTION_H

/**
 * @brief Describes single not instant motion for a robot.
 * 
 */
class Motion {
public:
    // Amount of ticks
    int duration;

    // Pause after motion complition (ticks)
    int pauseAfter;

    // Target position for each servo
    double targetPositions[12]{};

    Motion(int duration, int pauseAfter, const double * targetPositions);
};

#endif
