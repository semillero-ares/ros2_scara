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
    int direction_cmd = 0;
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

    void loop(double dt){
      measure(dt);
      control();
    }

  private: 

    int position_delta_ = 0;

    double calc_position(){
      position_delta_ = position_cnts + position_offset;
      return (double) position_delta_ / counts_per_unit;
    }

    void measure(double dt){
      double pos_prev = position;
      position = calc_position();
      velocity = (position - pos_prev) / dt;
    }

    void control(){
      error = ref - position;
      cmd = gain*error;
      velocity_cmd = cmd*counts_per_unit;
      if(velocity_cmd == 0){
        period_cmd = 0;
      }else{
        if(velocity_cmd>0){
          direction_cmd = 1;
        }else{
          direction_cmd = 0;
        }
        period_cmd = (int) (1000000/velocity_cmd);
        period_cmd = fmin(fmax(abs(period_cmd),us_min),us_max);
      }
    }

};

#endif // SCARA_HARDWARE_JOINT_HPP
