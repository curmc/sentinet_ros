
namespace rmt {
// DRIVE TRAIN, ROS METHODS, ROBOT CLASS

class Peripheral {
      public:
        virtual void stop() = 0;
        virtual void start() = 0;
        virtual void loop() = 0;
};

class DriveTrain : public Peripheral {
      public:
        void stop() override;
        void start() override;
        void loop() override;
};

class Robot {
      public:
        // Robot handles publishing linear and angular
        void publishLinAng();

      private:
        DriveTrain dt{};
}

} // namespace rmt
