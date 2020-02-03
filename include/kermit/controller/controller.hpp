// GENERIC CONTROLLER
#include <type_traits>

namespace rmt {

enum class GENERIC_BUTTON : unsigned short {
        FORWARD = (1 << 0),
        BACKWARD = (1 << 1),
        LEFT = (1 << 2),
        RIGHT = (1 << 3),
        VERBOSE = (1 << 4),
        NONLIN = (1 << 5),
        DISCLINANG = (1 << 6),
        DEBUG = (1 << 7),
        ADVANCED = (1 << 8)
};

template <typename E> constexpr auto to_under(E e) {
        return static_cast<typename std::underlying_type<E>::type>(e);
}

class Controller {
      public:
        virtual void poll() = 0;

        virtual void output_status();

      protected:
        unsigned short state{0};
};
} // namespace rmt