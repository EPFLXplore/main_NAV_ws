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
double target_value = 0;

static const int period = 1; // 1ms

void* cyclic_Task(void* arg){

  (void)arg;
  high_resolution_clock::time_point time_point_start_loop;
  volatile bool stop = false;

  cpu_set_t cpuset;
  pthread_t thread = pthread_self();
  CPU_ZERO(&cpuset);
  CPU_SET(0, &cpuset);
  pthread_setaffinity_np(thread, sizeof(cpu_set_t), &cpuset);

  SignalManager::registerCallback(SignalManager::Interrupt, "SigInt lambda",
  [&stop](int sig) {
    stop = true;
  });

  // Master creation
  Master master_ethercat;

  // Bus creation
  EthercatBus robot;

  // Adding network interface
  master_ethercat.add_Interface_Primary ( network_interface_name );

  // Device definition
  Epos4 epos_1, epos_2, epos_3;//, epos_4;
  
  epos_1.set_Id("EPOS4", 0x000000fb, 0x69500000);
  epos_2.set_Id("EPOS4", 0x000000fb, 0x69500000);
  epos_3.set_Id("EPOS4", 0x000000fb, 0x69500000);
  //epos_4.set_Id("EPOS4", 0x000000fb, 0x60500000);

  // Linking device to bus in hardware order !!
  robot.add_Device ( epos_1 );
  robot.add_Device ( epos_2 );
  robot.add_Device ( epos_3 );
  //robot.add_Device ( epos_4 );

  //add bus to master
  master_ethercat.add_Bus( robot );

  cout << "\n\n\n" << endl;


  while(!stop){
    // Get current time
    time_point_start_loop = high_resolution_clock::now();

    // Change state of Epos sfm to Lunch power
    if (epos_2.get_Device_State_In_String() == "Switch on disable") {
      epos_2.set_Device_State_Control_Word(Epos4::shutdown);
    }
    if (epos_2.get_Device_State_In_String() == "Ready to switch ON") {
      epos_2.set_Device_State_Control_Word(Epos4::switch_on_and_enable_op);
    }

    // Set type of control
    epos_2.set_Control_Mode(control_mode);

    if ( epos_2.get_Device_State_In_String() == "Operation enable") {
      if (control_mode == Epos4::position_CSP){
        epos_2.set_Target_Position_In_Qc(target_value);
        cout << "Desired position value = " << std::dec <<target_value << " qc" << "\n";
      }else if (control_mode == Epos4::velocity_CSV){
        epos_2.set_Target_Velocity_In_Rpm(target_value);
        cout << "Desired velocity value = " << std::dec <<target_value << " rpm" << "\n";
      }else if (control_mode == Epos4::torque_CST){
        epos_2.set_Target_Torque_In_Nm(target_value);
        cout << "Desired target value = " << std::dec <<target_value << " Nm"<< "\n";
      }
    }
    if (control_mode == Epos4::profile_position_PPM){
      // unlock axle
      epos_2.halt_Axle(false);
      // Starting new positionning at receive new order (or wait finish before start new with "false state")
      epos_2.change_Starting_New_Pos_Config(true);
      // normal mode (not in endless)
      epos_2.active_Endless_Movement(false);

      //epos_2.active_Absolute_Positionning();
      epos_2.active_Relative_Positionning();

      if (!(epos_2.get_Device_State_In_String() == "Operation enable")) {
        epos_2.activate_Profile_Control(false);
      } else {
        cout << "************************************** \n";
        epos_2.activate_Profile_Control(true);
        epos_2.set_Target_Position_In_Qc(target_value);
        cout << "Desired position value = " << std::dec <<target_value << " qc" << "\n";
      }
    }

    // epos_2.set_Digital_Output_State(Epos4::dig_out_1, false);
    // epos_2.set_Digital_Output_State(Epos4::dig_out_2, false);
    // epos_2.set_Digital_Output_State(Epos4::dig_out_hs_1, false);

    // Change state of Epos sfm to Lunch power
    if (epos_3.get_Device_State_In_String() == "Switch on disable") {
      epos_3.set_Device_State_Control_Word(Epos4::shutdown);
    }
    if (epos_3.get_Device_State_In_String() == "Ready to switch ON") {
      epos_3.set_Device_State_Control_Word(Epos4::switch_on_and_enable_op);
    }

    // Set type of control
    epos_3.set_Control_Mode(control_mode);

    if ( epos_3.get_Device_State_In_String() == "Operation enable") {
      if (control_mode == Epos4::position_CSP){
        epos_3.set_Target_Position_In_Qc(target_value);
        cout << "Desired position value = " << std::dec <<target_value << " qc" << "\n";
      }else if (control_mode == Epos4::velocity_CSV){
        epos_3.set_Target_Velocity_In_Rpm(target_value);
        cout << "Desired velocity value = " << std::dec <<target_value << " rpm" << "\n";
      }else if (control_mode == Epos4::torque_CST){
        epos_3.set_Target_Torque_In_Nm(target_value);
        cout << "Desired target value = " << std::dec <<target_value << " Nm"<< "\n";
      }
    }
    if (control_mode == Epos4::profile_position_PPM){
      // unlock axle
      epos_3.halt_Axle(false);
      // Starting new positionning at receive new order (or wait finish before start new with "false state")
      epos_3.change_Starting_New_Pos_Config(true);
      // normal mode (not in endless)
      epos_3.active_Endless_Movement(false);

      //epos_3.active_Absolute_Positionning();
      epos_3.active_Relative_Positionning();

      if (!(epos_3.get_Device_State_In_String() == "Operation enable")) {
        epos_3.activate_Profile_Control(false);
      } else {
        cout << "************************************** \n";
        epos_3.activate_Profile_Control(true);
        epos_3.set_Target_Position_In_Qc(target_value);
        cout << "Desired position value = " << std::dec <<target_value << " qc" << "\n";
      }
    }

    // epos_3.set_Digital_Output_State(Epos4::dig_out_1, false);
    // epos_3.set_Digital_Output_State(Epos4::dig_out_2, false);
    // epos_3.set_Digital_Output_State(Epos4::dig_out_hs_1, false);
/*
    // Change state of Epos sfm to Lunch power
    if (epos_4.get_Device_State_In_String() == "Switch on disable") {
      epos_4.set_Device_State_Control_Word(Epos4::shutdown);
    }
    if (epos_4.get_Device_State_In_String() == "Ready to switch ON") {
      epos_4.set_Device_State_Control_Word(Epos4::switch_on_and_enable_op);
    }

    // Set type of control
    epos_4.set_Control_Mode(control_mode);

    if ( epos_4.get_Device_State_In_String() == "Operation enable") {
      if (control_mode == Epos4::position_CSP){
        epos_4.set_Target_Position_In_Qc(target_value);
        cout << "Desired position value = " << std::dec <<target_value << " qc" << "\n";
      }else if (control_mode == Epos4::velocity_CSV){
        epos_4.set_Target_Velocity_In_Rpm(target_value);
        cout << "Desired velocity value = " << std::dec <<target_value << " rpm" << "\n";
      }else if (control_mode == Epos4::torque_CST){
        epos_4.set_Target_Torque_In_Nm(target_value);
        cout << "Desired target value = " << std::dec <<target_value << " Nm"<< "\n";
      }
    }
    if (control_mode == Epos4::profile_position_PPM){
      // unlock axle
      epos_4.halt_Axle(false);
      // Starting new positionning at receive new order (or wait finish before start new with "false state")
      epos_4.change_Starting_New_Pos_Config(true);
      // normal mode (not in endless)
      epos_4.active_Endless_Movement(false);

      //epos_4.active_Absolute_Positionning();
      epos_4.active_Relative_Positionning();

      if (!(epos_4.get_Device_State_In_String() == "Operation enable")) {
        epos_4.activate_Profile_Control(false);
      } else {
        cout << "************************************** \n";
        epos_4.activate_Profile_Control(true);
        epos_4.set_Target_Position_In_Qc(target_value);
        cout << "Desired position value = " << std::dec <<target_value << " qc" << "\n";
      }
    }

    // epos_4.set_Digital_Output_State(Epos4::dig_out_1, false);
    // epos_4.set_Digital_Output_State(Epos4::dig_out_2, false);
    // epos_4.set_Digital_Output_State(Epos4::dig_out_hs_1, false);
*/
    bool wkc = master_ethercat.next_Cycle();

    if (wkc == true) {

      cout << "State device : " << epos_2.get_Device_State_In_String() << "\n";
      cout << "Control mode = " << epos_2.get_Control_Mode_In_String() << "\n";
/*
      cout << "Actual position : " << std::dec <<epos_2.get_Actual_Position_In_Qc() << " qc" << "\n";
      cout << "Actual position : " << std::dec <<epos_2.get_Actual_Position_In_Rad() << " rad" << "\n";

      cout << "Actual velocity : " << std::dec << epos_2.get_Actual_Velocity_In_Rads() << " rad/s"<<  "\n";
      cout << "Actual Average velocity : " << std::dec << epos_2.get_Actual_Average_Velocity_In_Rads() << " rad/s"<<  "\n";
      cout << "Actual velocity : " << std::dec << epos_2.get_Actual_Velocity_In_Rpm() << " rpm"<<  "\n";
      cout << "Actual Average velocity : " << std::dec << epos_2.get_Actual_Average_Velocity_In_Rpm() << " rpm"<<  "\n";
      cout << "Actual torque : " << std::dec << epos_2.get_Actual_Torque_In_Nm() << " Nm"<< "\n";
      cout << "Actual Average torque : " << std::dec << epos_2.get_Actual_Average_Torque_In_Nm() << " Nm"<< "\n";
      cout << "Actual torque : " << std::dec << epos_2.get_Actual_Torque_In_RT() << " per mile RT"<< "\n";
      cout << "Actual Average torque : " << std::dec << epos_2.get_Actual_Average_Torque_In_RT() << " per mile RT"<< "\n";
      cout << "Actual current : " << std::dec << epos_2.get_Actual_Current_In_A() << " A"<< "\n";
      cout << "Actual Average current : " << std::dec << epos_2.get_Actual_Average_Current_In_A() << " A"<< "\n";
*/
      // Specific for PPM mode
      if (control_mode == Epos4::profile_position_PPM){
        cout << "Target is reached : " << epos_2.check_target_reached() << "\n";
      }

      // cout << "Digital Input 1 = " << epos_2.get_Digital_Input_State(Epos4::dig_in_1) << "\n";
      // cout << "Digital Input 2 = " << epos_2.get_Digital_Input_State(Epos4::dig_in_2) << "\n";
      // cout << "Digital Input 3 = " << epos_2.get_Digital_Input_State(Epos4::dig_in_3) << "\n";
      // cout << "Digital Input 4 = " << epos_2.get_Digital_Input_State(Epos4::dig_in_4) << "\n";
      // cout << "Digital Input Hs 1 = " << epos_2.get_Digital_Input_State(Epos4::dig_in_hs_1) << "\n";
      // cout << "Digital Input Hs 2 = " << epos_2.get_Digital_Input_State(Epos4::dig_in_hs_2) << "\n";
      // cout << "Digital Input Hs 3 = " << epos_2.get_Digital_Input_State(Epos4::dig_in_hs_3) << "\n";
      // cout << "Digital Input Hs 4 = " << epos_2.get_Digital_Input_State(Epos4::dig_in_hs_4) << "\n";

      // cout << "Analog Input 1 = " << epos_2.get_Analog_Input(Epos4::analog_in_1) << " V" << "\n";
      // cout << "Analog Input 2 = " << epos_2.get_Analog_Input(Epos4::analog_in_2) << " V" << "\n";

      cout << "State device : " << epos_3.get_Device_State_In_String() << "\n";
      cout << "Control mode = " << epos_3.get_Control_Mode_In_String() << "\n";
/*
      cout << "Actual position : " << std::dec <<epos_3.get_Actual_Position_In_Qc() << " qc" << "\n";
      cout << "Actual position : " << std::dec <<epos_3.get_Actual_Position_In_Rad() << " rad" << "\n";

      cout << "Actual velocity : " << std::dec << epos_3.get_Actual_Velocity_In_Rads() << " rad/s"<<  "\n";
      cout << "Actual Average velocity : " << std::dec << epos_3.get_Actual_Average_Velocity_In_Rads() << " rad/s"<<  "\n";
      cout << "Actual velocity : " << std::dec << epos_3.get_Actual_Velocity_In_Rpm() << " rpm"<<  "\n";
      cout << "Actual Average velocity : " << std::dec << epos_3.get_Actual_Average_Velocity_In_Rpm() << " rpm"<<  "\n";
      cout << "Actual torque : " << std::dec << epos_3.get_Actual_Torque_In_Nm() << " Nm"<< "\n";
      cout << "Actual Average torque : " << std::dec << epos_3.get_Actual_Average_Torque_In_Nm() << " Nm"<< "\n";
      cout << "Actual torque : " << std::dec << epos_3.get_Actual_Torque_In_RT() << " per mile RT"<< "\n";
      cout << "Actual Average torque : " << std::dec << epos_3.get_Actual_Average_Torque_In_RT() << " per mile RT"<< "\n";
      cout << "Actual current : " << std::dec << epos_3.get_Actual_Current_In_A() << " A"<< "\n";
      cout << "Actual Average current : " << std::dec << epos_3.get_Actual_Average_Current_In_A() << " A"<< "\n";
*/
      // Specific for PPM mode
      if (control_mode == Epos4::profile_position_PPM){
        cout << "Target is reached : " << epos_3.check_target_reached() << "\n";
      }

      // cout << "Digital Input 1 = " << epos_3.get_Digital_Input_State(Epos4::dig_in_1) << "\n";
      // cout << "Digital Input 2 = " << epos_3.get_Digital_Input_State(Epos4::dig_in_2) << "\n";
      // cout << "Digital Input 3 = " << epos_3.get_Digital_Input_State(Epos4::dig_in_3) << "\n";
      // cout << "Digital Input 4 = " << epos_3.get_Digital_Input_State(Epos4::dig_in_4) << "\n";
      // cout << "Digital Input Hs 1 = " << epos_3.get_Digital_Input_State(Epos4::dig_in_hs_1) << "\n";
      // cout << "Digital Input Hs 2 = " << epos_3.get_Digital_Input_State(Epos4::dig_in_hs_2) << "\n";
      // cout << "Digital Input Hs 3 = " << epos_3.get_Digital_Input_State(Epos4::dig_in_hs_3) << "\n";
      // cout << "Digital Input Hs 4 = " << epos_3.get_Digital_Input_State(Epos4::dig_in_hs_4) << "\n";

      // cout << "Analog Input 1 = " << epos_3.get_Analog_Input(Epos4::analog_in_1) << " V" << "\n";
      // cout << "Analog Input 2 = " << epos_3.get_Analog_Input(Epos4::analog_in_2) << " V" << "\n";
/*
      cout << "State device : " << epos_4.get_Device_State_In_String() << "\n";
      cout << "Control mode = " << epos_4.get_Control_Mode_In_String() << "\n";

      cout << "Actual position : " << std::dec <<epos_4.get_Actual_Position_In_Qc() << " qc" << "\n";
      cout << "Actual position : " << std::dec <<epos_4.get_Actual_Position_In_Rad() << " rad" << "\n";

      cout << "Actual velocity : " << std::dec << epos_4.get_Actual_Velocity_In_Rads() << " rad/s"<<  "\n";
      cout << "Actual Average velocity : " << std::dec << epos_4.get_Actual_Average_Velocity_In_Rads() << " rad/s"<<  "\n";
      cout << "Actual velocity : " << std::dec << epos_4.get_Actual_Velocity_In_Rpm() << " rpm"<<  "\n";
      cout << "Actual Average velocity : " << std::dec << epos_4.get_Actual_Average_Velocity_In_Rpm() << " rpm"<<  "\n";
      cout << "Actual torque : " << std::dec << epos_4.get_Actual_Torque_In_Nm() << " Nm"<< "\n";
      cout << "Actual Average torque : " << std::dec << epos_4.get_Actual_Average_Torque_In_Nm() << " Nm"<< "\n";
      cout << "Actual torque : " << std::dec << epos_4.get_Actual_Torque_In_RT() << " per mile RT"<< "\n";
      cout << "Actual Average torque : " << std::dec << epos_4.get_Actual_Average_Torque_In_RT() << " per mile RT"<< "\n";
      cout << "Actual current : " << std::dec << epos_4.get_Actual_Current_In_A() << " A"<< "\n";
      cout << "Actual Average current : " << std::dec << epos_4.get_Actual_Average_Current_In_A() << " A"<< "\n";

      // Specific for PPM mode
      if (control_mode == Epos4::profile_position_PPM){
        cout << "Target is reached : " << epos_4.check_target_reached() << "\n";
      }

      // cout << "Digital Input 1 = " << epos_4.get_Digital_Input_State(Epos4::dig_in_1) << "\n";
      // cout << "Digital Input 2 = " << epos_4.get_Digital_Input_State(Epos4::dig_in_2) << "\n";
      // cout << "Digital Input 3 = " << epos_4.get_Digital_Input_State(Epos4::dig_in_3) << "\n";
      // cout << "Digital Input 4 = " << epos_4.get_Digital_Input_State(Epos4::dig_in_4) << "\n";
      // cout << "Digital Input Hs 1 = " << epos_4.get_Digital_Input_State(Epos4::dig_in_hs_1) << "\n";
      // cout << "Digital Input Hs 2 = " << epos_4.get_Digital_Input_State(Epos4::dig_in_hs_2) << "\n";
      // cout << "Digital Input Hs 3 = " << epos_4.get_Digital_Input_State(Epos4::dig_in_hs_3) << "\n";
      // cout << "Digital Input Hs 4 = " << epos_4.get_Digital_Input_State(Epos4::dig_in_hs_4) << "\n";

      // cout << "Analog Input 1 = " << epos_4.get_Analog_Input(Epos4::analog_in_1) << " V" << "\n";
      // cout << "Analog Input 2 = " << epos_4.get_Analog_Input(Epos4::analog_in_2) << " V" << "\n";
*/
    } //end of valid workcounter
    high_resolution_clock::time_point time_point_end_loop = high_resolution_clock::now();

    // Wait end of period
    this_thread::sleep_until(time_point_start_loop + chrono::milliseconds(period));
    duration<double> time_duration_loop = duration_cast<duration<double>>(time_point_end_loop - time_point_start_loop);
    std::cout << "Time loop = " << time_duration_loop.count() << " seconds.\n";


    high_resolution_clock::time_point time_point_after_sleep_loop = high_resolution_clock::now();
    duration<double> time_duration_loop_after_sleep = duration_cast<duration<double>>(time_point_after_sleep_loop - time_point_start_loop);
    std::cout << "Time loop after sleep = " << time_duration_loop_after_sleep.count() << " seconds.\n";


    cout << "\n\n\n" << endl;
  } //end of while

  SignalManager::unregisterCallback(SignalManager::Interrupt, "SigInt lambda");

  // close EtherCAT master and socket.
  master_ethercat.end();
  cout << "close master and end of cyclic task " << endl;
  return (nullptr);
}



int main(int argc, char **argv){

  ros::init(argc, argv, "controller_motor");

  cout << "here we go !!!!!!" << endl;

  ros::NodeHandle n;

  


  cout << "yolo"<<endl;

  pthread_t thread_cyclic_loop;
  pthread_attr_t attr_cyclic_loop;
  struct sched_param sched_param_cyclic_loop;

  cout << "Epos 4 command example" << endl ;
//   if(argc<4){
//     cout << "Invalid input: network interface name (ex: eth0), "
//          << " control mode (\"cyclic_position\" in qc,"
//          << "\"cyclic_velocity\" in rpm, \"cyclic_torque\" in Nm) or"
//          << "\"profile_position\" in qc"
//          << "and target value " <<endl;
//     exit (0);
//   }

  network_interface_name = "eth0";
  std::string input_control_mode = "cyclic_velocity";
  std::string input_target = "1000";
  target_value = atof(input_target.c_str());

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

  pthread_attr_init(&attr_cyclic_loop);
  pthread_attr_setschedpolicy(&attr_cyclic_loop, SCHED_FIFO);
  sched_param_cyclic_loop.sched_priority = 90;
  pthread_attr_setschedparam(&attr_cyclic_loop, &sched_param_cyclic_loop);
  mlockall(MCL_CURRENT|MCL_FUTURE);
  pthread_create(&thread_cyclic_loop, &attr_cyclic_loop, cyclic_Task, nullptr);

  pthread_join(thread_cyclic_loop, nullptr);
  munlockall();

  cout << "End program" << endl ;

  
  ros::spin();

  return 0;

}