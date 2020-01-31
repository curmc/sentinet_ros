#include "kermit/controller/wii/wii_controller.hpp"

#include <iostream>

namespace rmt {

WiiController::WiiController() { &data = wiiuse_init(MAXWIIMOTES); }

WiiController::WiiController(WiiController &&other) {
        wiiuse_cleanup(&data, MAXWIIMOTES);
        found = 0;
        connected = 0;
        verbose = false;
        swap(*this, other);
}

WiiController::Wiicontroller &operator=(WiiController &&other) {
        wiiuse_cleanup(&data, MAXWIIMOTES);
        found = 0;
        connected = 0;
        verbose = false;
        swap(*this, other);
}

WiiController::~WiiController() { wiiuse_cleanup(&data, MAXWIIMOTES); }

inline void swap(WiiController &first, WiiController &second) {
        using std::swap;
        swap(first.data, second.data);
        swap(first.found, second.found);
        swap(first.connected, second.connected);
        swap(first.verbose, second.verbose);
}

void WiiController::connect() {

        found = wiiuse_find(&data, MAXWIIMOTES, 5);
        if (!found) {
                cout << "No wiimotes found." << endl;
                return -1;
        }

        connected = wiiuse_connect(&data, MAXWIIMOTES);
        if (connected) {
                cout << "Connected to " << connected << "wiimote";
                (connected > 1) ? cout << "s.\n" : cout << ".\n";
        } else {
                cout << "Failed to connect, although found " << found
                     << "wiimote";
                (found > 1) ? cout << "s.\n" : cout << ".\n";

                return -1;
        }

        wiiuse_set_leds(data[0], WIIMOTE_LED_1);

        // Saving battery
        wiiuse_motion_sensing(data[0], 0);
        wiiuse_set_ir(data[0], 0);
        wiiuse_set_motion_plus(data[0], 0);
}

int WiiController::heartbeat() const {
        if (data && WIIMOTE_IS_CONNECTED(*data)) {
                return 1;
        } else {
                cout << "HEARTBEAT FAILURE: Unexpected wiimote disconnect\n";
                return 0;
        }
        return 0;
}

void WiiController::poll() {
        if (heartbeat()) {
                if (wiiuse_poll(&data, MAXWIIMOTES)) {
                        switch (data->event) {
                        case WIIUSE_EVENT:
                                event();
                                break;

                        case WIIUSE_STATUS:
                                if (verbose)
                                        output_status();
                                break;

                        case WIIUSE_DISCONNECT:
                        case WIIUSE_UNEXPECTED_DISCONNECT:
                                // handle disconnects
                                break;

                        case WIIUSE_READ_DATA:
                                break;

                        default:
                                break;
                        }
                }
        }
}

// Alter controller state on button press
void WiiController::event() {
        const auto toggle_state = [&](GENERIC_BUTTON toggling,
                                      const uint16_t button) {
                if (IS_JUST_PRESSED(data, button)) {
                        state = !(state & toggling) ? state | toggling
                                                    : state - toggling;

                        /* Strange bug continuously toggles buttons until
                         * another one is pressed. Here's a fix */
                        data->btns &= WIIMOTE_BUTTON_ZACCEL_BIT4;
                }
        };

        // for (const auto &p: BUTTON_MAP) if no structured binding
        for (const auto [wiibutton, genericbutton] : BUTTON_MAP) {
                toggle_state(genericbutton, wiibutton);
        }
}

void WiiController::output_status() {

        std::cout << "---- CONTROLLER STATUS ----\n"
                  << "battery:      " << data[0]->battery_level << '\n'
                  << "STATE:\n";

        Controller::output_status();
}

return 0;
} // namespace rmt

} // namespace rmt