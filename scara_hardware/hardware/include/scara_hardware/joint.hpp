#ifndef SCARA_HARDWARE_JOINT_HPP
#define SCARA_HARDWARE_JOINT_HPP

#include <string>
#include <cmath>

class Joint
{
    public:

    std::string name = "";
    double ref = 0;
    double error = 0;
    double gain = 0; // <---------- input parameter
    double cmd = 0; 
    double counts_per_unit = 0;
    double velocity_cmd = 0;
    int period_cmd = 0;
    int us_min = 1; // <----------- input parameter
    int us_max = 10; // <---------- input parameter
    int position_cnts = 0;
    int position_offset = 0; // <-- input parameter
    double position = 0;
    double velocity = 0;

    Joint() = default;

    Joint(const std::string &joint_name, double controller_gain, int min, int max, int offset)
    {
      setup(joint_name, controller_gain, min, max, offset);
    }

    void setup(const std::string &joint_name, double controller_gain, int min, int max, int offset)
    {
      // M_PI es la variable de pi=3.14159
      name = joint_name;
      gain = controller_gain;
      us_min = min;
      us_max = max;
      position_offset = offset;
    }

    double calc_position(){
      return 0;
    }

    double calc_velocity(){
      return 0;
    }

    double calc_enc_angle()
    {
      return 0 ;//enc * rads_per_count;
    }

};

#endif // SCARA_HARDWARE_JOINT_HPP
