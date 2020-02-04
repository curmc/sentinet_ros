/**
 * @author      : theo (theo.j.lincke@gmail.com)
 * @file        : ControllerBaseInterface
 * @brief       : A brief description of the file
 *
 * A detailed description of the file
 *
 * @created     : Tuesday Feb 04, 2020 15:45:58 MST
 * @license     : MIT
 */

#ifndef CONTROLLERBASEINTERFACE_HPP

#define CONTROLLERBASEINTERFACE_HPP

// C++ includes

typedef uint16_t state_t;

/*
 * Simple now, but base interface
 * functionality must be present in the future
 */
class ControllerBaseInterface
{
  public:
        ControllerBaseInterface ();
        virtual ~ControllerBaseInterface ();

  // Getters
  public:
        // State Getters
        virtual std::string get_state_str() const = 0;
        virtual state_t get_state() const = 0;

        // Cmd Vel Getters
        virtual float get_linear_vel() const = 0;
        virtual float get_angular_vel() const = 0;

  // Setters
  public:
        // State Setters
        virtual void set_state(state_t) = 0;

        // Cmd Vel Getters
        virtual void set_linear_vel(const float lin) = 0;

        virtual void get_angular_vel(const float ang) = 0;

        inline void set_cmd_vel(const float lin, const float ang) {
          set_linear_vel(lin);
          set_angular_vel(ang);
        }
};

#endif /* end of include guard CONTROLLERBASEINTERFACE_HPP */

