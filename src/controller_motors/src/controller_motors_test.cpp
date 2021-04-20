#include <controller_motors/WheelVelocity.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/Image.h"
#include <string>
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "message_filters/cache.h"


#include <iostream>
#include <chrono>
#include <thread>
#include <pthread.h>
#include <sys/mman.h>
#include <pid/signal_manager.h>

#include <ethercatcpp/master.h>
#include <ethercatcpp/epos4.h>

using namespace std;
using namespace std::chrono;
using namespace ethercatcpp;
using namespace pid;


using namespace message_filters;
using namespace sensor_msgs;

Epos4::control_mode_t control_mode;
std::string network_interface_name;
double target_value [6] = {0, 0, 0, 0, 0, 0};

static const int period = 40; // [ms]  1ms does not work

void wheel_callback(const controller_motors::WheelVelocity::ConstPtr& msg){
  /*
   * whenever a message of type WheelVeloicity is published on the /wheel_publisher topic
   * this callback is called
   *
   * wheel_callback updates the values of the target_value array which contains the reference 
   * individual wheel speeds expressed in RPMs
  */
  target_value[0] = msg->motor_L_1;
  target_value[1] = msg->motor_L_2;
  target_value[2] = msg->motor_L_3;
  target_value[3] = msg->motor_R_1;
  target_value[4] = msg->motor_R_2;
  target_value[5] = msg->motor_R_3;
}

void check_epos_state(Epos4& epos, Epos4::control_mode_t& control_mode){
  // Change state of Epos sfm to Lunch power
  if (epos.get_Device_State_In_String() == "Switch on disable") {
    epos.set_Device_State_Control_Word(Epos4::shutdown);
  }
  if (epos.get_Device_State_In_String() == "Ready to switch ON") {
    epos.set_Device_State_Control_Word(Epos4::switch_on_and_enable_op);
  }
  // Set type of control
  epos.set_Control_Mode(control_mode);
}

void command_epos(Epos4& epos, Epos4::control_mode_t& control_mode, float target_value){
  /*
  *
  *  TODO: handle the case when we want the motor to stop (now we have to send 0)
  *
  */
  if ( epos.get_Device_State_In_String() == "Operation enable") {
    if (control_mode == Epos4::position_CSP){
      epos.set_Target_Position_In_Qc(target_value);
      cout << "Desired position value = " << std::dec <<target_value << " qc" << "\n";
    }else if (control_mode == Epos4::velocity_CSV){
      epos.set_Target_Velocity_In_Rpm(target_value);
      cout << "Desired velocity value = " << std::dec <<target_value << " rpm" << "\n";
    }else if (control_mode == Epos4::torque_CST){
      epos.set_Target_Torque_In_Nm(target_value);
      cout << "Desired target value = " << std::dec <<target_value << " Nm"<< "\n";
    }
    
    if (control_mode == Epos4::profile_position_PPM){
      // unlock axle
      epos.halt_Axle(false);
      // Starting new positionning at receive new order (or wait finish before start new with "false state")
      epos.change_Starting_New_Pos_Config(true);
      // normal mode (not in endless)
      epos.active_Endless_Movement(false);

      //epos.active_Absolute_Positionning();
      epos.active_Relative_Positionning();

      if (!(epos.get_Device_State_In_String() == "Operation enable")) {
        epos.activate_Profile_Control(false);
      } else {
        cout << "************************************** \n";
        epos.activate_Profile_Control(true);
        epos.set_Target_Position_In_Qc(target_value);
        cout << "Desired position value = " << std::dec <<target_value << " qc" << "\n";
      }
    }

  }
}








int main(int argc, char **argv){

  ros::init(argc, argv, "controller_motors");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe<controller_motors::WheelVelocity>("wheel_publisher", 10, wheel_callback);

  cout << "ROS node initialized" << endl;

  network_interface_name = "eth1";
  std::string input_control_mode = "cyclic_velocity";
  std::string input_target = "10";
  //target_value = atof(input_target.c_str());

  //Check input control mode
  if (input_control_mode == "cyclic_position"){
    control_mode = Epos4::position_CSP ;
  }else if(input_control_mode == "cyclic_velocity"){
    control_mode = Epos4::velocity_CSV;
  }else if(input_control_mode == "cyclic_torque"){
    control_mode = Epos4::torque_CST;
  }else if(input_control_mode == "profile_position"){
    control_mode = Epos4::profile_position_PPM;
  }else {
    cout << "Invalid input: control mode (\"cyclic_position\" in qc,"
        << "\"cyclic_velocity\" in rpm, \"cyclic_torque\" in Nm) or"
        << "\"profile_position\" in qc"
        << "and target value " <<endl;
    exit (0);
  }

  // Master creation
  Master master_ethercat;

  // Bus creation
  EthercatBus robot;

  // Adding network interface
  master_ethercat.add_Interface_Primary ( network_interface_name );

  // Device definition
  Epos4 epos_L_1, epos_L_2, epos_L_3, epos_R_1, epos_R_2, epos_R_3;

  epos_L_1.set_Id("EPOS4", 0x000000fb, 0x69500000);
  epos_L_2.set_Id("EPOS4", 0x000000fb, 0x69500000);
  epos_L_3.set_Id("EPOS4", 0x000000fb, 0x69500000);
  epos_R_1.set_Id("EPOS4", 0x000000fb, 0x69500000);
  epos_R_2.set_Id("EPOS4", 0x000000fb, 0x69500000);
  epos_R_3.set_Id("EPOS4", 0x000000fb, 0x69500000);
  //epos_4.set_Id("EPOS4", 0x000000fb, 0x60500000);

  // Linking device to bus in hardware order !!
  robot.add_Device ( epos_L_1 );
  robot.add_Device ( epos_L_2 );
  robot.add_Device ( epos_L_3 );
  robot.add_Device ( epos_R_1 );
  robot.add_Device ( epos_R_2 );
  robot.add_Device ( epos_R_3 );
  //robot.add_Device ( epos_4 );

  //add bus to master
  master_ethercat.add_Bus( robot );

  cout << "Ethercat network online" << endl;

  high_resolution_clock::time_point time_point_start_loop;

  while(ros::ok()){ 
    // Get current time
    time_point_start_loop = high_resolution_clock::now();
    // check device status
    check_epos_state(epos_L_1, control_mode);
    check_epos_state(epos_L_2, control_mode);
    check_epos_state(epos_L_3, control_mode);
    check_epos_state(epos_R_1, control_mode);
    check_epos_state(epos_R_2, control_mode);
    check_epos_state(epos_R_3, control_mode);
    // command the devuce
    command_epos(epos_L_1, control_mode, target_value[0]);
    command_epos(epos_L_2, control_mode, target_value[1]);
    command_epos(epos_L_3, control_mode, target_value[2]);
    command_epos(epos_R_1, control_mode, target_value[3]);
    command_epos(epos_R_2, control_mode, target_value[4]);
    command_epos(epos_R_3, control_mode, target_value[5]);
    
        // // // epos_L_3.set_Digital_Output_State(Epos4::dig_out_1, false);
    // // epos_L_3.set_Digital_Output_State(Epos4::dig_out_2, false);
    // epos_L_3.set_Digital_Output_State(Epos4::dig_out_hs_1, false);

    bool wkc = master_ethercat.next_Cycle(); // Function used to launch next cycle of the EtherCat net

    if (wkc == true) {
      //EPOS_L_1
      cout << "State device : " << epos_L_1.get_Device_State_In_String() << "\n";
      cout << "Control mode = " << epos_L_1.get_Control_Mode_In_String() << "\n";
  /*
      cout << "Actual position : " << std::dec <<epos_L_2.get_Actual_Position_In_Qc() << " qc" << "\n";
      cout << "Actual position : " << std::dec <<epos_L_2.get_Actual_Position_In_Rad() << " rad" << "\n";

      cout << "Actual velocity : " << std::dec << epos_L_2.get_Actual_Velocity_In_Rads() << " rad/s"<<  "\n";
      cout << "Actual Average velocity : " << std::dec << epos_L_2.get_Actual_Average_Velocity_In_Rads() << " rad/s"<<  "\n";
      cout << "Actual velocity : " << std::dec << epos_L_2.get_Actual_Velocity_In_Rpm() << " rpm"<<  "\n";
      cout << "Actual Average velocity : " << std::dec << epos_L_2.get_Actual_Average_Velocity_In_Rpm() << " rpm"<<  "\n";
      cout << "Actual torque : " << std::dec << epos_L_2.get_Actual_Torque_In_Nm() << " Nm"<< "\n";
      cout << "Actual Average torque : " << std::dec << epos_L_2.get_Actual_Average_Torque_In_Nm() << " Nm"<< "\n";
      cout << "Actual torque : " << std::dec << epos_L_2.get_Actual_Torque_In_RT() << " per mile RT"<< "\n";
      cout << "Actual Average torque : " << std::dec << epos_L_2.get_Actual_Average_Torque_In_RT() << " per mile RT"<< "\n";
      cout << "Actual current : " << std::dec << epos_L_2.get_Actual_Current_In_A() << " A"<< "\n";
      cout << "Actual Average current : " << std::dec << epos_L_2.get_Actual_Average_Current_In_A() << " A"<< "\n";
  */
      // Specific for PPM mode
      if (control_mode == Epos4::profile_position_PPM){
        cout << "Target is reached : " << epos_L_1.check_target_reached() << "\n";
      }

      // cout << "Digital Input 1 = " << epos_L_2.get_Digital_Input_State(Epos4::dig_in_1) << "\n";
      // cout << "Digital Input 2 = " << epos_L_2.get_Digital_Input_State(Epos4::dig_in_2) << "\n";
      // cout << "Digital Input 3 = " << epos_L_2.get_Digital_Input_State(Epos4::dig_in_3) << "\n";
      // cout << "Digital Input 4 = " << epos_L_2.get_Digital_Input_State(Epos4::dig_in_4) << "\n";
      // cout << "Digital Input Hs 1 = " << epos_L_2.get_Digital_Input_State(Epos4::dig_in_hs_1) << "\n";
      // cout << "Digital Input Hs 2 = " << epos_L_2.get_Digital_Input_State(Epos4::dig_in_hs_2) << "\n";
      // cout << "Digital Input Hs 3 = " << epos_L_2.get_Digital_Input_State(Epos4::dig_in_hs_3) << "\n";
      // cout << "Digital Input Hs 4 = " << epos_L_2.get_Digital_Input_State(Epos4::dig_in_hs_4) << "\n";

      // cout << "Analog Input 1 = " << epos_L_2.get_Analog_Input(Epos4::analog_in_1) << " V" << "\n";
      // cout << "Analog Input 2 = " << epos_L_2.get_Analog_Input(Epos4::analog_in_2) << " V" << "\n";

      //EPOS_L_2
      cout << "State device : " << epos_L_2.get_Device_State_In_String() << "\n";
      cout << "Control mode = " << epos_L_2.get_Control_Mode_In_String() << "\n";
  /*
      cout << "Actual position : " << std::dec <<epos_L_2.get_Actual_Position_In_Qc() << " qc" << "\n";
      cout << "Actual position : " << std::dec <<epos_L_2.get_Actual_Position_In_Rad() << " rad" << "\n";

      cout << "Actual velocity : " << std::dec << epos_L_2.get_Actual_Velocity_In_Rads() << " rad/s"<<  "\n";
      cout << "Actual Average velocity : " << std::dec << epos_L_2.get_Actual_Average_Velocity_In_Rads() << " rad/s"<<  "\n";
      cout << "Actual velocity : " << std::dec << epos_L_2.get_Actual_Velocity_In_Rpm() << " rpm"<<  "\n";
      cout << "Actual Average velocity : " << std::dec << epos_L_2.get_Actual_Average_Velocity_In_Rpm() << " rpm"<<  "\n";
      cout << "Actual torque : " << std::dec << epos_L_2.get_Actual_Torque_In_Nm() << " Nm"<< "\n";
      cout << "Actual Average torque : " << std::dec << epos_L_2.get_Actual_Average_Torque_In_Nm() << " Nm"<< "\n";
      cout << "Actual torque : " << std::dec << epos_L_2.get_Actual_Torque_In_RT() << " per mile RT"<< "\n";
      cout << "Actual Average torque : " << std::dec << epos_L_2.get_Actual_Average_Torque_In_RT() << " per mile RT"<< "\n";
      cout << "Actual current : " << std::dec << epos_L_2.get_Actual_Current_In_A() << " A"<< "\n";
      cout << "Actual Average current : " << std::dec << epos_L_2.get_Actual_Average_Current_In_A() << " A"<< "\n";
  */
      // Specific for PPM mode
      if (control_mode == Epos4::profile_position_PPM){
        cout << "Target is reached : " << epos_L_2.check_target_reached() << "\n";
      }

      // cout << "Digital Input 1 = " << epos_L_2.get_Digital_Input_State(Epos4::dig_in_1) << "\n";
      // cout << "Digital Input 2 = " << epos_L_2.get_Digital_Input_State(Epos4::dig_in_2) << "\n";
      // cout << "Digital Input 3 = " << epos_L_2.get_Digital_Input_State(Epos4::dig_in_3) << "\n";
      // cout << "Digital Input 4 = " << epos_L_2.get_Digital_Input_State(Epos4::dig_in_4) << "\n";
      // cout << "Digital Input Hs 1 = " << epos_L_2.get_Digital_Input_State(Epos4::dig_in_hs_1) << "\n";
      // cout << "Digital Input Hs 2 = " << epos_L_2.get_Digital_Input_State(Epos4::dig_in_hs_2) << "\n";
      // cout << "Digital Input Hs 3 = " << epos_L_2.get_Digital_Input_State(Epos4::dig_in_hs_3) << "\n";
      // cout << "Digital Input Hs 4 = " << epos_L_2.get_Digital_Input_State(Epos4::dig_in_hs_4) << "\n";

      // cout << "Analog Input 1 = " << epos_L_2.get_Analog_Input(Epos4::analog_in_1) << " V" << "\n";
      // cout << "Analog Input 2 = " << epos_L_2.get_Analog_Input(Epos4::analog_in_2) << " V" << "\n";

      //EPOS_L_3
      cout << "State device : " << epos_L_3.get_Device_State_In_String() << "\n";
      cout << "Control mode = " << epos_L_3.get_Control_Mode_In_String() << "\n";
  /*
      cout << "Actual position : " << std::dec <<epos_L_3.get_Actual_Position_In_Qc() << " qc" << "\n";
      cout << "Actual position : " << std::dec <<epos_L_3.get_Actual_Position_In_Rad() << " rad" << "\n";

      cout << "Actual velocity : " << std::dec << epos_L_3.get_Actual_Velocity_In_Rads() << " rad/s"<<  "\n";
      cout << "Actual Average velocity : " << std::dec << epos_L_3.get_Actual_Average_Velocity_In_Rads() << " rad/s"<<  "\n";
      cout << "Actual velocity : " << std::dec << epos_L_3.get_Actual_Velocity_In_Rpm() << " rpm"<<  "\n";
      cout << "Actual Average velocity : " << std::dec << epos_L_3.get_Actual_Average_Velocity_In_Rpm() << " rpm"<<  "\n";
      cout << "Actual torque : " << std::dec << epos_L_3.get_Actual_Torque_In_Nm() << " Nm"<< "\n";
      cout << "Actual Average torque : " << std::dec << epos_L_3.get_Actual_Average_Torque_In_Nm() << " Nm"<< "\n";
      cout << "Actual torque : " << std::dec << epos_L_3.get_Actual_Torque_In_RT() << " per mile RT"<< "\n";
      cout << "Actual Average torque : " << std::dec << epos_L_3.get_Actual_Average_Torque_In_RT() << " per mile RT"<< "\n";
      cout << "Actual current : " << std::dec << epos_L_3.get_Actual_Current_In_A() << " A"<< "\n";
      cout << "Actual Average current : " << std::dec << epos_L_3.get_Actual_Average_Current_In_A() << " A"<< "\n";
  */
      // Specific for PPM mode
      if (control_mode == Epos4::profile_position_PPM){
        cout << "Target is reached : " << epos_L_3.check_target_reached() << "\n";
      }

      // cout << "Digital Input 1 = " << epos_L_3.get_Digital_Input_State(Epos4::dig_in_1) << "\n";
      // cout << "Digital Input 2 = " << epos_L_3.get_Digital_Input_State(Epos4::dig_in_2) << "\n";
      // cout << "Digital Input 3 = " << epos_L_3.get_Digital_Input_State(Epos4::dig_in_3) << "\n";
      // cout << "Digital Input 4 = " << epos_L_3.get_Digital_Input_State(Epos4::dig_in_4) << "\n";
      // cout << "Digital Input Hs 1 = " << epos_L_3.get_Digital_Input_State(Epos4::dig_in_hs_1) << "\n";
      // cout << "Digital Input Hs 2 = " << epos_L_3.get_Digital_Input_State(Epos4::dig_in_hs_2) << "\n";
      // cout << "Digital Input Hs 3 = " << epos_L_3.get_Digital_Input_State(Epos4::dig_in_hs_3) << "\n";
      // cout << "Digital Input Hs 4 = " << epos_L_3.get_Digital_Input_State(Epos4::dig_in_hs_4) << "\n";

      // cout << "Analog Input 1 = " << epos_L_3.get_Analog_Input(Epos4::analog_in_1) << " V" << "\n";
      // cout << "Analog Input 2 = " << epos_L_3.get_Analog_Input(Epos4::analog_in_2) << " V" << "\n";

       //EPOS_R_1
      cout << "State device : " << epos_R_1.get_Device_State_In_String() << "\n";
      cout << "Control mode = " << epos_R_1.get_Control_Mode_In_String() << "\n";
  /*
      cout << "Actual position : " << std::dec <<epos_L_2.get_Actual_Position_In_Qc() << " qc" << "\n";
      cout << "Actual position : " << std::dec <<epos_L_2.get_Actual_Position_In_Rad() << " rad" << "\n";

      cout << "Actual velocity : " << std::dec << epos_L_2.get_Actual_Velocity_In_Rads() << " rad/s"<<  "\n";
      cout << "Actual Average velocity : " << std::dec << epos_L_2.get_Actual_Average_Velocity_In_Rads() << " rad/s"<<  "\n";
      cout << "Actual velocity : " << std::dec << epos_L_2.get_Actual_Velocity_In_Rpm() << " rpm"<<  "\n";
      cout << "Actual Average velocity : " << std::dec << epos_L_2.get_Actual_Average_Velocity_In_Rpm() << " rpm"<<  "\n";
      cout << "Actual torque : " << std::dec << epos_L_2.get_Actual_Torque_In_Nm() << " Nm"<< "\n";
      cout << "Actual Average torque : " << std::dec << epos_L_2.get_Actual_Average_Torque_In_Nm() << " Nm"<< "\n";
      cout << "Actual torque : " << std::dec << epos_L_2.get_Actual_Torque_In_RT() << " per mile RT"<< "\n";
      cout << "Actual Average torque : " << std::dec << epos_L_2.get_Actual_Average_Torque_In_RT() << " per mile RT"<< "\n";
      cout << "Actual current : " << std::dec << epos_L_2.get_Actual_Current_In_A() << " A"<< "\n";
      cout << "Actual Average current : " << std::dec << epos_L_2.get_Actual_Average_Current_In_A() << " A"<< "\n";
  */
      // Specific for PPM mode
      if (control_mode == Epos4::profile_position_PPM){
        cout << "Target is reached : " << epos_R_1.check_target_reached() << "\n";
      }

      // cout << "Digital Input 1 = " << epos_L_2.get_Digital_Input_State(Epos4::dig_in_1) << "\n";
      // cout << "Digital Input 2 = " << epos_L_2.get_Digital_Input_State(Epos4::dig_in_2) << "\n";
      // cout << "Digital Input 3 = " << epos_L_2.get_Digital_Input_State(Epos4::dig_in_3) << "\n";
      // cout << "Digital Input 4 = " << epos_L_2.get_Digital_Input_State(Epos4::dig_in_4) << "\n";
      // cout << "Digital Input Hs 1 = " << epos_L_2.get_Digital_Input_State(Epos4::dig_in_hs_1) << "\n";
      // cout << "Digital Input Hs 2 = " << epos_L_2.get_Digital_Input_State(Epos4::dig_in_hs_2) << "\n";
      // cout << "Digital Input Hs 3 = " << epos_L_2.get_Digital_Input_State(Epos4::dig_in_hs_3) << "\n";
      // cout << "Digital Input Hs 4 = " << epos_L_2.get_Digital_Input_State(Epos4::dig_in_hs_4) << "\n";

      // cout << "Analog Input 1 = " << epos_L_2.get_Analog_Input(Epos4::analog_in_1) << " V" << "\n";
      // cout << "Analog Input 2 = " << epos_L_2.get_Analog_Input(Epos4::analog_in_2) << " V" << "\n";

      //EPOS_R_2
      cout << "State device : " << epos_R_2.get_Device_State_In_String() << "\n";
      cout << "Control mode = " << epos_R_2.get_Control_Mode_In_String() << "\n";
  /*
      cout << "Actual position : " << std::dec <<epos_L_2.get_Actual_Position_In_Qc() << " qc" << "\n";
      cout << "Actual position : " << std::dec <<epos_L_2.get_Actual_Position_In_Rad() << " rad" << "\n";

      cout << "Actual velocity : " << std::dec << epos_L_2.get_Actual_Velocity_In_Rads() << " rad/s"<<  "\n";
      cout << "Actual Average velocity : " << std::dec << epos_L_2.get_Actual_Average_Velocity_In_Rads() << " rad/s"<<  "\n";
      cout << "Actual velocity : " << std::dec << epos_L_2.get_Actual_Velocity_In_Rpm() << " rpm"<<  "\n";
      cout << "Actual Average velocity : " << std::dec << epos_L_2.get_Actual_Average_Velocity_In_Rpm() << " rpm"<<  "\n";
      cout << "Actual torque : " << std::dec << epos_L_2.get_Actual_Torque_In_Nm() << " Nm"<< "\n";
      cout << "Actual Average torque : " << std::dec << epos_L_2.get_Actual_Average_Torque_In_Nm() << " Nm"<< "\n";
      cout << "Actual torque : " << std::dec << epos_L_2.get_Actual_Torque_In_RT() << " per mile RT"<< "\n";
      cout << "Actual Average torque : " << std::dec << epos_L_2.get_Actual_Average_Torque_In_RT() << " per mile RT"<< "\n";
      cout << "Actual current : " << std::dec << epos_L_2.get_Actual_Current_In_A() << " A"<< "\n";
      cout << "Actual Average current : " << std::dec << epos_L_2.get_Actual_Average_Current_In_A() << " A"<< "\n";
  */
      // Specific for PPM mode
      if (control_mode == Epos4::profile_position_PPM){
        cout << "Target is reached : " << epos_R_2.check_target_reached() << "\n";
      }

      // cout << "Digital Input 1 = " << epos_L_2.get_Digital_Input_State(Epos4::dig_in_1) << "\n";
      // cout << "Digital Input 2 = " << epos_L_2.get_Digital_Input_State(Epos4::dig_in_2) << "\n";
      // cout << "Digital Input 3 = " << epos_L_2.get_Digital_Input_State(Epos4::dig_in_3) << "\n";
      // cout << "Digital Input 4 = " << epos_L_2.get_Digital_Input_State(Epos4::dig_in_4) << "\n";
      // cout << "Digital Input Hs 1 = " << epos_L_2.get_Digital_Input_State(Epos4::dig_in_hs_1) << "\n";
      // cout << "Digital Input Hs 2 = " << epos_L_2.get_Digital_Input_State(Epos4::dig_in_hs_2) << "\n";
      // cout << "Digital Input Hs 3 = " << epos_L_2.get_Digital_Input_State(Epos4::dig_in_hs_3) << "\n";
      // cout << "Digital Input Hs 4 = " << epos_L_2.get_Digital_Input_State(Epos4::dig_in_hs_4) << "\n";

      // cout << "Analog Input 1 = " << epos_L_2.get_Analog_Input(Epos4::analog_in_1) << " V" << "\n";
      // cout << "Analog Input 2 = " << epos_L_2.get_Analog_Input(Epos4::analog_in_2) << " V" << "\n";

      //EPOS_R_3
      cout << "State device : " << epos_R_3.get_Device_State_In_String() << "\n";
      cout << "Control mode = " << epos_R_3.get_Control_Mode_In_String() << "\n";
  /*
      cout << "Actual position : " << std::dec <<epos_L_3.get_Actual_Position_In_Qc() << " qc" << "\n";
      cout << "Actual position : " << std::dec <<epos_L_3.get_Actual_Position_In_Rad() << " rad" << "\n";

      cout << "Actual velocity : " << std::dec << epos_L_3.get_Actual_Velocity_In_Rads() << " rad/s"<<  "\n";
      cout << "Actual Average velocity : " << std::dec << epos_L_3.get_Actual_Average_Velocity_In_Rads() << " rad/s"<<  "\n";
      cout << "Actual velocity : " << std::dec << epos_L_3.get_Actual_Velocity_In_Rpm() << " rpm"<<  "\n";
      cout << "Actual Average velocity : " << std::dec << epos_L_3.get_Actual_Average_Velocity_In_Rpm() << " rpm"<<  "\n";
      cout << "Actual torque : " << std::dec << epos_L_3.get_Actual_Torque_In_Nm() << " Nm"<< "\n";
      cout << "Actual Average torque : " << std::dec << epos_L_3.get_Actual_Average_Torque_In_Nm() << " Nm"<< "\n";
      cout << "Actual torque : " << std::dec << epos_L_3.get_Actual_Torque_In_RT() << " per mile RT"<< "\n";
      cout << "Actual Average torque : " << std::dec << epos_L_3.get_Actual_Average_Torque_In_RT() << " per mile RT"<< "\n";
      cout << "Actual current : " << std::dec << epos_L_3.get_Actual_Current_In_A() << " A"<< "\n";
      cout << "Actual Average current : " << std::dec << epos_L_3.get_Actual_Average_Current_In_A() << " A"<< "\n";
  */
      // Specific for PPM mode
      if (control_mode == Epos4::profile_position_PPM){
        cout << "Target is reached : " << epos_R_3.check_target_reached() << "\n";
      }

      // cout << "Digital Input 1 = " << epos_L_3.get_Digital_Input_State(Epos4::dig_in_1) << "\n";
      // cout << "Digital Input 2 = " << epos_L_3.get_Digital_Input_State(Epos4::dig_in_2) << "\n";
      // cout << "Digital Input 3 = " << epos_L_3.get_Digital_Input_State(Epos4::dig_in_3) << "\n";
      // cout << "Digital Input 4 = " << epos_L_3.get_Digital_Input_State(Epos4::dig_in_4) << "\n";
      // cout << "Digital Input Hs 1 = " << epos_L_3.get_Digital_Input_State(Epos4::dig_in_hs_1) << "\n";
      // cout << "Digital Input Hs 2 = " << epos_L_3.get_Digital_Input_State(Epos4::dig_in_hs_2) << "\n";
      // cout << "Digital Input Hs 3 = " << epos_L_3.get_Digital_Input_State(Epos4::dig_in_hs_3) << "\n";
      // cout << "Digital Input Hs 4 = " << epos_L_3.get_Digital_Input_State(Epos4::dig_in_hs_4) << "\n";

      // cout << "Analog Input 1 = " << epos_L_3.get_Analog_Input(Epos4::analog_in_1) << " V" << "\n";
      // cout << "Analog Input 2 = " << epos_L_3.get_Analog_Input(Epos4::analog_in_2) << " V" << "\n";

    } //end of valid workcounter

    high_resolution_clock::time_point time_point_end_loop = high_resolution_clock::now();

    ros::spinOnce();

    // Wait end of period
    this_thread::sleep_until(time_point_start_loop + chrono::milliseconds(period));
    duration<double> time_duration_loop = duration_cast<duration<double>>(time_point_end_loop - time_point_start_loop);
    std::cout << "Time loop = " << time_duration_loop.count() << " seconds.\n";


    high_resolution_clock::time_point time_point_after_sleep_loop = high_resolution_clock::now();
    duration<double> time_duration_loop_after_sleep = duration_cast<duration<double>>(time_point_after_sleep_loop - time_point_start_loop);
    std::cout << "Time loop after sleep = " << time_duration_loop_after_sleep.count() << " seconds.\n";


    cout << "\n\n\n" << endl;
  }  

  cout << "End program" << endl;
  return 0;
}