#include "kermit/controller/controller.hpp"

#include <iostream>
#include <string>

namespace rmt {

void Controller::output_status() {
        const auto output_state = [&](GENERIC_BUTTON status, std::string name) {
                std::string output = (state & to_under(status)) ? "1" : "0";
                std::cout << name << ": " << output << '\n';
        };

        output_state(GENERIC_BUTTON::FORWARD, "FORWARD");
        output_state(GENERIC_BUTTON::BACKWARD, "BACKWARD");
        output_state(GENERIC_BUTTON::LEFT, "LEFT");
        output_state(GENERIC_BUTTON::RIGHT, "RIGHT");
        output_state(GENERIC_BUTTON::VERBOSE, "VERBOSE");
        output_state(GENERIC_BUTTON::NONLIN, "NONLIN");
        output_state(GENERIC_BUTTON::DISCLINANG, "DISCLINANG");
        output_state(GENERIC_BUTTON::DEBUG, "DEBUG");
        output_state(GENERIC_BUTTON::ADVANCED, "ADVANCED");
}

} // namespace rmt
