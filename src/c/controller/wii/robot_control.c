/**
 * @author      : theo (theo@$HOSTNAME)
 * @file        : robot_control
 * @created     : Wednesday Aug 21, 2019 11:21:27 MDT
 */

#include "kermit/controller/wii/robot_control.h"


static float constrain(float min, float val, float max) {
  if (val <= min)
    return min;
  if (val >= max)
    return max;
  return val;
}

// The two types of drive train callbacks
void *discrete(struct robot_s *robot, float ang, float lin) {
  pthread_mutex_lock(&p_lock);

  /*
   * If disclang is on and there's an angular input,
   * linear is zero (angular trumps linear)
   */
  if(!((robot->options & DISCLINANG) && (int)ang))
    robot->drive->linear_vel = lin * robot->drive->cruising_speed;
  else
    robot->drive->linear_vel = 0.0;

  /*
   * If disclin ang is on and there's a linear input,
   * angular is zero
   */
  if(!((robot->options & DISCLINANG) && (int)lin))
    robot->drive->angular_vel = ang * robot->drive->cruising_speed;
  else
    robot->drive->linear_vel = 0.0;

  pthread_mutex_unlock(&p_lock);
  return NULL;
}

void *increment(struct robot_s *robot, float ang, float lin) {
  pthread_mutex_lock(&p_lock);

  float max = robot->drive->max_speed;

  // Linear update
  if(!((robot->options & DISCLINANG) && (int)ang)) {
    float v = lin * robot->drive->speed_increment + robot->drive->linear_vel;
    robot->drive->linear_vel = constrain(-max, v, max);
  } else {
    robot->drive->linear_vel = 0.0;
  }

  // Angular Update
  if(!((robot->options & DISCLINANG) && (int)lin)) {
    float v = ang * robot->drive->speed_increment + robot->drive->angular_vel;
    robot->drive->angular_vel = constrain(-max, v, max);
  } else {
    robot->drive->angular_vel = 0.0;
  }

  pthread_mutex_unlock(&p_lock);
  return NULL;
}

static void drive_train_zeros(struct drive_train *dt) {
  if (dt) {
    dt->linear_vel = 0;
    dt->angular_vel = 0;
  }
}

void drive_train_stop(struct robot_s *robot) {
  if (robot)
    drive_train_zeros(robot->drive);
}

void drive_train_start(struct robot_s *robot) {
  if (robot)
    drive_train_zeros(robot->drive);
}

void attach_dt_callback(struct drive_train *dt,
                        void *(*cb)(struct robot_s *, float, float)) {
  if (dt) {
    drive_train_zeros(dt);
    dt->on_vel_callback = cb;
  }
}

void *drive_train_main_callback(struct robot_s *robot, float ang, float lin) {
  if (robot->options & VAR_SPEED)
    increment(robot, ang, lin);
  else
    discrete(robot, ang, lin);
  return NULL;
}

struct drive_train create_drive_train(float cruising, float max_speed, float speed) {

  struct drive_train dt;
  struct peripheral *p = (struct peripheral*)malloc(sizeof(struct peripheral));

  dt.p = p;

  drive_train_zeros(&dt);

  dt.p->start = &drive_train_start;
  dt.p->stop = &drive_train_stop;
  dt.p->loop = NULL; // No loop function for the drive train

  attach_dt_callback(&dt, &drive_train_main_callback); // Default increment

  dt.max_speed = max_speed;
  dt.speed_increment = speed;
  dt.cruising_speed = cruising;
  return dt;
}

////////// ROBOT METHODS /////////
void stop_robot(struct robot_s *robot) {
  if (robot->drive && robot->drive->p->stop)
    (*robot->drive->p->stop)(robot);
}

void start_robot(struct robot_s *robot) {
  if (robot->drive && robot->drive->p->start)
    (*robot->drive->p->start)(robot);
}

void loop_robot(struct robot_s *robot) {
  if (robot->drive && robot->drive->p->loop)
    (*robot->drive->p->loop)(robot);
}

void robot_setopt(struct robot_s *robot, uint8_t option) {
  robot->options |= option;
}

void robot_unsetopt(struct robot_s *robot, uint8_t option) {
  robot->options &= ~option;
}

void robot_changeopt(struct robot_s *robot, uint8_t option) {
  if (robot->options & option)
    robot_unsetopt(robot, option);
  else
    robot_setopt(robot, option);
}

struct robot_s create_robot() {
  struct robot_s robot;
  struct peripheral *p = (struct peripheral*)malloc(sizeof(struct peripheral));

  robot.drive = NULL;

  p->start = &start_robot;
  p->stop = &stop_robot;
  p->loop = &loop_robot;

  robot.p = p;
  return robot;
}

void robot_clean_up(struct robot_s *robot) {
  if (robot == NULL)
    return;
  if (robot->p)
    free(robot->p);
  if (robot->drive && robot->drive->p) {
    free(robot->drive->p);
    free(robot->drive);
  }
}
