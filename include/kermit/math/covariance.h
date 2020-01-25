/**
 * @author      : theo (theo.j.lincke@gmail.com)
 * @file        : statistics
 * @created     : Tuesday Jan 07, 2020 11:05:55 MST
 */

#ifndef STATISTICS_H

#define STATISTICS_H

#include <inttypes.h>

typedef struct {
        float measurement;
        float mean;
        uint64_t count;
} reading;

typedef struct {
        reading x;
        reading y;
        float xy_sum;
        float cov;
} covariance;

covariance initialize_covariance();
float update_covariance(covariance *m, float xm, float ym);

#endif /* end of include guard STATISTICS_H */
