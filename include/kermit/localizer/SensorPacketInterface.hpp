/**
 * @author      : theo (theo.j.lincke@gmail.com)
 * @file        : SensorPacketInterface
 * @brief       : A brief description of the file
 *
 * A detailed description of the file
 *
 * @created     : Tuesday Feb 04, 2020 15:56:44 MST
 * @license     : MIT
 */

#ifndef SENSORPACKETINTERFACE_HPP

#define SENSORPACKETINTERFACE_HPP

// C++ includes
#include <string>


class SensorPacketInterface
{
  public:
        SensorPacketInterface (const std::string id)
          : identifier(id) {};

        virtual ~SensorPacketInterface () = default;
        
        std::string get_type() const { return identifier; }

  private:

        /*
         * In the future when we want stats on messages,
         * we can add meta info, but best to start simple
         */

        // sensor type
        const std::string identifier;
};

#endif /* end of include guard SENSORPACKETINTERFACE_HPP */

