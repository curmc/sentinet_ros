/**
 * @author      : theo (theo@varnsen)
 * @file        : robot_control
 * @brief A Wii controller logic controller - interracts with wiiuse and wii
 * controllers
 *
 * This file uses wiiuse to create controller and robot logic
 *
 * @created     : Friday Aug 09, 2019 12:18:16 MDT
 * @bugs  No known bugs
 */

#ifndef WII_H

#define WII_H

// C Includes
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

// Local Includes
#include "wiiuse.h"

#ifdef __cplusplus 
extern "C" {
#endif

#define MAX_WIIMOTES 1

#define VAR_SPEED (1 << 0)
#define VERBOSE (1 << 1)
#define INSYNC (1 << 2)
#define NONLIN (1 << 3)
#define DISCLINANG (1 << 4)
#define DEBUG (1 << 5)
#define ADVNCD (1 << 6)

////////// Data Structures //////////

// A lock for when we interact with the robot
pthread_mutex_t p_lock;

struct robot_s; // Forward declare

// In c++, you would call this a virtual class
struct peripheral {
  void (*stop)(struct robot_s *robot);
  void (*start)(struct robot_s *robot);
  void (*loop)(struct robot_s *robot); // normally null???
};

// A type of peripheral
struct drive_train {
  struct peripheral *p;

  // triggered on an event, calls this method
  void *(*on_vel_callback)(struct robot_s *robot, float lin, float ang);

  float linear_vel;
  float angular_vel;

  float cruising_speed;
  float speed_increment;
  float max_speed;
};

// A robot type with all the information a robot needs
struct robot_s {
  struct peripheral *p;

  struct drive_train *drive;

  uint8_t options;
  float period;
};

/**
 * @brief Start the robot and all its peripherals
 *
 * @param robot The robot and its peripherals
 */
void start_robot(struct robot_s *robot);

/**
 * @brief Stop the robot and all its peripherals
 *
 * @param robot The robot to stop
 */
void stop_robot(struct robot_s *robot);

/**
 * @brief Loop the robot
 *
 * @param robot The robot to loop
 */
void loop_robot(struct robot_s *robot);

/**
 * @brief Sets robot options by orring the option
 *
 * @param robot The robot to set option on
 * @param option The option - available options are at the top
 */
void robot_setopt(struct robot_s *robot, uint8_t option);

/**
 * @brief Unsets robot options by anding not the option
 *
 * @param robot The robot to remove the option on
 * @param option The option to set
 */
void robot_unsetopt(struct robot_s *robot, uint8_t option);

/**
 * @brief Changes option
 *
 * @param robot The robot to change the option on
 * @param option The option to change
 */
void robot_changeopt(struct robot_s *robot, uint8_t option);

/**
 * @brief Creates a default robot
 *
 * @return default robot, this is an empty robot, you need to set it's
 * attributes
 */
struct robot_s create_robot();

////////// DRIVE TRAIN METHODS //////////
/**
 * @brief Increments the robot's speed rather than just putting the speed on
 *
 * @param robot The robot to incriment
 * @param ang Angular magnitude to incriment by
 * @param lin Linear magnitude to incriment by
 */
void *increment(struct robot_s *robot, float ang, float lin);

/**
 * @brief Just makes the wheels turn, no incremental value
 *
 * @param robot The robot to change values
 * @param ang The angular to give the robot (1 -1 0)
 * @param lin the linear to give the robot
 */
void *discrete(struct robot_s *robot, float ang, float lin);

/**
 * @brief Stops the drive train, attatched to robot stop callback
 *
 * @param dt The drive train to stop
 */
void drive_train_stop(struct robot_s *dt);

/**
 * @brief Starts the drive train, attatched to robot start callback
 *
 * @param dt The drive train to start
 */
void drive_train_start(struct robot_s *dt);

/**
 * @brief Attatch a drivetrain callback to the robot
 *
 * @param dt The drive train to attatch to
 * @param type The type (0 incremental, 1 discrete)
 */
void attach_dt_callback(struct drive_train *dt,
                        void *(*cb)(struct robot_s *, float, float));

/**
 * @brief Creates a drive train
 *
 * @return A drive train
 */
struct drive_train create_drive_train(float cruising, float max_speed, float speed);

/**
 * @brief This is just linear, I'm lazy, haven't changed it yet
 *
 * @param current Current speed
 * @param incremental_speed Incrimental speed (robot->speed)
 * @param max_speed Maximum speed
 *
 * @return The nuew "current" sppeed
 */
// int non_linear(int current, int incremental_speed, int max_speed);

/**
 * @brief Cleans up a robot and all its stuff
 *
 * @param robot The robot to clean up
 */
void robot_clean_up(struct robot_s *robot);

#ifdef __cplusplus
}
#endif
#endif
