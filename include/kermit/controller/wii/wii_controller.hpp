#include "kermit/controller/controller.hpp"
#include "wiiuse.h"

#include <cstdint>
#include <utility>

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
        static constexpr std::array<std::pair<unsigned short, unsigned short>,
                                    9>
            BUTTON_MAP{std::make_pair(UINT16_C(WIIMOTE_BUTTON_UP),
                                      to_under(GENERIC_BUTTON::FORWARD)),
                       std::make_pair(UINT16_C(WIIMOTE_BUTTON_DOWN),
                                      to_under(GENERIC_BUTTON::BACKWARD)),
                       std::make_pair(UINT16_C(WIIMOTE_BUTTON_LEFT),
                                      to_under(GENERIC_BUTTON::LEFT)),
                       std::make_pair(UINT16_C(WIIMOTE_BUTTON_RIGHT),
                                      to_under(GENERIC_BUTTON::RIGHT)),
                       std::make_pair(UINT16_C(WIIMOTE_BUTTON_HOME),
                                      to_under(GENERIC_BUTTON::VERBOSE)),
                       std::make_pair(UINT16_C(WIIMOTE_BUTTON_MINUS),
                                      to_under(GENERIC_BUTTON::NONLIN)),
                       std::make_pair(UINT16_C(WIIMOTE_BUTTON_PLUS),
                                      to_under(GENERIC_BUTTON::DISCLINANG)),
                       std::make_pair(UINT16_C(WIIMOTE_BUTTON_ONE),
                                      to_under(GENERIC_BUTTON::DEBUG)),
                       std::make_pair(UINT16_C(WIIMOTE_BUTTON_TWO),
                                      to_under(GENERIC_BUTTON::ADVANCED))};
};
} // namespace rmt