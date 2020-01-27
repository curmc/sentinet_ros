#include "controller.hpp"
#include "wiiuse.h"

namespace rmt {

// Wii controller implimentation
class WiiController : public Controller {

      public:
        // reads controller and flips bits on state when appropriate
        void poll() override;

        // Disable copy constructor/assignment on resource handling class
        WiiController(const WiiController &other) = delete;
        WiiController &operator=(const WiiController &other) = delete;

      private:
        // wiimote stuff
};
} // namespace rmt