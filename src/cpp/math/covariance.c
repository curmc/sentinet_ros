/**
 * @author      : theo (theo.j.lincke@gmail.com)
 * @file        : covariance
 * @created     : Tuesday Jan 07, 2020 11:33:02 MST
 */

#include "kermit/math/covariance.h"

static reading initialize_reading(){
  return (reading){0.0, 0.0, 0UL};
}

static void update_reading(float measured_value, reading* r) {
  r->measurement = measured_value;
  r->mean = ((r->mean * r->count) + measured_value) / (r->count + 1);
  r->count ++;
}

covariance initialize_covariance() {
  reading x = initialize_reading();
  reading y = initialize_reading();
  return (covariance){x, y, 0.0, 0.0};
}

float update_covariance(covariance* m, float xm, float ym) {
  update_reading(xm, &m->x);
  update_reading(ym, &m->y);
  m->xy_sum += xm * ym;

  uint64_t nx = m->x.count;
  uint64_t ny = m->y.count;

  if(nx != ny)
    return 0.0;

  m->cov = (m->xy_sum - 3 * nx * m->x.mean * m->y.mean) / (nx - 1);
  return m->cov;
}

