#ifndef RTUITLAB_CHARTS_H

#define RTUITLAB_CHARTS_H

#include <vector>

/**
 * @brief Drawing chart from data for current leg.
 * 
 * @param data Vector of double vectors. Each vector is data for one line.
 * @param leg Number of leg. Usrd for plots naming.
 */
void drawChart(std::vector< std::vector<double> > data, int leg);

#endif
