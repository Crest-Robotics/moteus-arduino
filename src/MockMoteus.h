#include <Moteus.h>


namespace mm = mjbots::moteus;

inline int signum(double x) {
    return (x > 0) - (x < 0);
}

class MockMoteus : public Moteus {
public:
  MockMoteus(ACAN2517FD &can, const Moteus::Options &options = {})
      : Moteus(can, options) {
    mock_last_result_.values.velocity = 0.0;
    current_velocity = 0.0;
  }

  bool isMock() override { return true; }

  const Moteus::Result &last_result() const override {
    // Serial.println("last_result_mock");
    return mock_last_result_;
  }


  bool SetPosition(const mm::PositionMode::Command &cmd,
                   const mm::PositionMode::Format *command_override = nullptr,
                   const mm::Query::Format *query_override = nullptr) override {


    double static_friction = 0.10;  // Adjust as needed
    double viscous_friction = 0.05; // Adjust as needed


    // Calculate acceleration from torque
    double acceleration = cmd.feedforward_torque / (mass * radius);

    // Update velocity
    if (abs(current_velocity) > static_friction) {
        // Apply static friction only when the object is moving
        double friction = signum(current_velocity) * static_friction;
        current_velocity = current_velocity - friction + acceleration * dt - viscous_friction * current_velocity;
    } else if (abs(acceleration * dt) > static_friction) {
        current_velocity = signum(acceleration * dt) * (abs(acceleration * dt) - static_friction);
    }


    // Update the last_result_ member variable
    // Serial.print("torque command");
    // Serial.println(cmd.feedforward_torque);
    mock_last_result_.values.velocity = current_velocity;

    return true;
  }

private:
  double desired_velocity = 0.0;
  double current_velocity = 0.0;
  double t = 0.0;
  double mass = 1.0;
  double radius = 0.25;
  double dt = 0.1;
  Moteus::Result mock_last_result_;
};