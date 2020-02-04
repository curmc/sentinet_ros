/**
 * @author      : theo (theo.j.lincke@gmail.com)
 * @file        : LocalizerBaseInterface
 * @brief       : A brief description of the file
 *
 * A detailed description of the file
 *
 * @created     : Tuesday Feb 04, 2020 15:54:12 MST
 * @license     : MIT
 */

#ifndef LOCALIZERBASEINTERFACE_HPP

#define LOCALIZERBASEINTERFACE_HPP

// Local Includes
#include <kermit/localizer/SensorPacketInterface.hpp>

class LocalizerBaseInterface
{
  public:
        LocalizerBaseInterface ();
        virtual ~LocalizerBaseInterface ();

        virtual void register_callback(std::function<void(const SensorPacketInterface&)>);
        

  private:

        std::map<std::string, 
                 std::function<void(const SensorPacketInterface&)>> callbacks;
};

#endif /* end of include guard LOCALIZERBASEINTERFACE_HPP */

