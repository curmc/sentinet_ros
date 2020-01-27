#include "controller.hpp"
#include "kermit.hpp"

namespace rmt {
// CONNECTS CONTROLLER AND KERMIT
class RobotController {
      public:
        void poll();

      private:
        Controller controller{};
        Robot robot{};
};
} // namespace rmt