#ifndef PID_CONTROLLER_HPP_
#define PID_CONTROLLER_HPP_

#include <algorithm> // For std::clamp
#include <chrono>

class PIDController
{
public:
  PIDController(double kp, double /*ki*/, double /*kd*/, double min_output, double max_output)
    : kp_(kp), min_output_(min_output), max_output_(max_output)
  {
  }

  // This method now computes the NEXT desired position, not a control effort.
  double compute(double setpoint, double current_value)
  {
    // Calculate the difference between where we want to be and where we are.
    double error = setpoint - current_value;

    // The new position is the current position plus a step proportional to the error.
    // This creates a smooth exponential approach to the setpoint.
    double output = current_value + kp_ * error;

    // Clamp the output to the defined joint limits.
    return std::clamp(output, min_output_, max_output_);
  }

  void reset()
  {
    // Nothing to reset in this simplified version.
  }

private:
  double kp_;
  double min_output_, max_output_;
};

#endif  // PID_CONTROLLER_HPP_