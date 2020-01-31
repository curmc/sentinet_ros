// GENERIC CONTROLLER

namespace rmt {

enum class GENERIC_BUTTON : uint16_t {
        FORWARD = (1 << 0),
        BACKWARD = (1 << 1),
        LEFT = (1 << 2),
        RIGHT = (1 << 3),
        VERBOSE = (1 << 4),
        NONLIN = (1 << 5),
        DISCLINANG = (1 << 6),
        DEBUG = (1 << 7),
        ADVANCED = (1 << 8)
}

class Controller {
      public:
        virtual void poll() = 0;

        virtual void output_status();

      private:
        uint16_t state{0};
};
} // namespace rmt