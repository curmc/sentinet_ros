#include "controller.hpp"
#include "wiiuse.h"

#include <array>

namespace rmt {

// Wii controller implimentation
class WiiController : public Controller {

      public:
        WiiController();
        WiiController(const WiiController &other) = delete;
        WiiController &operator=(const WiiController &other) = delete;
        WiiController(WiiController &&other);
        WiiController &operator=(WiiController &&other);
        ~WiiController();

        // reads controller and flips bits on state when appropriate
        void poll() override;

        // Disable copy constructor/assignment on resource handling class

        void connect();
        void output_status() override;
        int heartbeat() const;

        friend void swap(WiiController &first, WiiController &second);

      private:
        void event();

        wiimote_t *data{nullptr};

        int found{0};
        int connected{0};

        const bool verbose{false};

        static constexpr int MAXWIIMOTES{1};
        static constexpr std::array<std::pair<uint16_t, uint16_t>, 9>
            BUTTON_MAP {
                (WIIMOTE_BUTTON_UP, GENERIC_BUTTON::FORWARD),
                    (WIIMOTE_BUTTON_DOWN, GENERIC_BUTTON::BACKWARD),
                    (WIIMOTE_BUTTON_LEFT, GENERIC_BUTTON::LEFT),
                    (WIIMOTE_BUTTON_RIGHT, GENERIC_BUTTON::RIGHT),
                    (WIIMOTE_BUTTON_HOME, GENERIC_BUTTON::VERBOSE),
                    (WIIMOTE_BUTTON_MINUS, GENERIC_BUTTON::NONLIN),
                    (WIIMOTE_BUTTON_PLUS, GENERIC_BUTTON::DISCLINANG),
                    (WIIMOTE_BUTTON_ONE, GENERIC_BUTTON::DEBUG),
                    (WIIMOTE_BUTTON_TWO, GENERIC_BUTTON::ADVANCED)
        }
};
} // namespace rmt