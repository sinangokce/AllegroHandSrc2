#include <ros/ros.h>

#include "std_msgs/String.h"
//#include "std_msgs/Float32.h"
//#include "std_msgs/Int16.h"
//#include "sensor_msgs/JointState.h"

#include <iostream>
#include <string>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <unistd.h>
#include <vector>

using namespace std;

#define DOF_JOINTS 16

// http://nehe.gamedev.net/article/msdn_virtualkey_codes/15009/
#define VK_SPACE 0x20
#define VK_LSHIFT 0xA0


#define KEYCODE_RIGHT 0x43
#define KEYCODE_LEFT 0x44
#define KEYCODE_UP 0x41
#define KEYCODE_DOWN 0x42

#define KEYCODE_0 0x30
#define KEYCODE_1 0x31
#define KEYCODE_2 0x32
#define KEYCODE_3 0x33
#define KEYCODE_4 0x34
#define KEYCODE_5 0x35
#define KEYCODE_6 0x36
#define KEYCODE_7 0x37
#define KEYCODE_8 0x38
#define KEYCODE_9 0x39

#define KEYCODE_NUMPAD0 0x60
#define KEYCODE_NUMPAD1 0x61
#define KEYCODE_NUMPAD2 0x62
#define KEYCODE_NUMPAD3 0x63
#define KEYCODE_NUMPAD4 0x64
#define KEYCODE_NUMPAD5 0x65
#define KEYCODE_NUMPAD6 0x66
#define KEYCODE_NUMPAD7 0x67
#define KEYCODE_NUMPAD8 0x68
#define KEYCODE_NUMPAD9 0x69

#define KEYCODE_A 0x41
#define KEYCODE_B 0x42
#define KEYCODE_C 0x43
#define KEYCODE_D 0x44
#define KEYCODE_E 0x45
#define KEYCODE_F 0x46
#define KEYCODE_G 0x47
#define KEYCODE_H 0x48
#define KEYCODE_I 0x49
#define KEYCODE_J 0x4A
#define KEYCODE_K 0x4B
#define KEYCODE_L 0x4C
#define KEYCODE_M 0x4D
#define KEYCODE_N 0x4E
#define KEYCODE_O 0x4F
#define KEYCODE_P 0x50
#define KEYCODE_Q 0x51
#define KEYCODE_R 0x52
#define KEYCODE_S 0x53
#define KEYCODE_T 0x54
#define KEYCODE_U 0x55
#define KEYCODE_V 0x56
#define KEYCODE_W 0x57
#define KEYCODE_X 0x58
#define KEYCODE_Y 0x59
#define KEYCODE_Z 0x5A

#define KEYCODE_a 0x61
#define KEYCODE_b 0x62
#define KEYCODE_c 0x63
#define KEYCODE_d 0x64
#define KEYCODE_e 0x65
#define KEYCODE_f 0x66
#define KEYCODE_g 0x67
#define KEYCODE_h 0x68
#define KEYCODE_i 0x69
#define KEYCODE_j 0x6A
#define KEYCODE_k 0x6B
#define KEYCODE_l 0x6C
#define KEYCODE_m 0x6D
#define KEYCODE_n 0x6E
#define KEYCODE_o 0x6F
#define KEYCODE_p 0x70
#define KEYCODE_q 0x71
#define KEYCODE_r 0x72
#define KEYCODE_s 0x73
#define KEYCODE_t 0x74
#define KEYCODE_u 0x75
#define KEYCODE_v 0x76
#define KEYCODE_w 0x77
#define KEYCODE_x 0x78
#define KEYCODE_y 0x79
#define KEYCODE_z 0x7A

#define KEYCODE_slash 0x2F
#define KEYCORD_question 0x3F

class AHKeyboard
{
 public:
  AHKeyboard();
  void keyLoop();
  void printUsage();

 private:
  ros::NodeHandle nh_;
  int count_;
  float edit_;
  ros::Publisher cmd_pub_;
};

AHKeyboard::AHKeyboard():
    count_(0),
    edit_(0.0)
{
  cmd_pub_   = nh_.advertise<std_msgs::String>("allegroHand_0/libsss_cmd", 1);
}

//int finger_num;
//int knuckle_num;

int kfd = 0;
struct termios cooked, raw;

void quit(int sig)
{
  tcsetattr(kfd, TCSANOW, &cooked);
  ros::shutdown();
  exit(0);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "allegro_hand_emg_grasp_type");
  AHKeyboard allegro_hand_keyboard_cmd;

  signal(SIGINT,quit);

  allegro_hand_keyboard_cmd.keyLoop();

  return(0);
}

void AHKeyboard::printUsage() {
  std::cout << std::endl;
  std::cout << " -----------------------------------------------------------------------------" << std::endl;
  std::cout << "  Use the keyboard to send Allegro Hand 4 different grasp types" << std::endl;
  std::cout << " -----------------------------------------------------------------------------" << std::endl;
  std::cout << "\tHome Pose:\t\t\t'H'" << std::endl;
  std::cout << "\tPower Grasp:\t\t\t'O'" << std::endl;
  std::cout << "\tThumb with 2 fingers:\t\t'T'" << std::endl;
  std::cout << "\tPinch:\t\t\t\t'P'" << std::endl;
  std::cout << "\tLateral:\t\t\t'L'" << std::endl;
}

void AHKeyboard::keyLoop()
{
  char c;
  bool dirty=false;
  bool jointUpdate=false;


  // get the console in raw mode
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  // Setting a new line, then end of file
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  sleep(2);
  printUsage();

  for(;;)
  {
    std_msgs::String msg;
    std::stringstream ss;

    // get the next event from the keyboard
    if(read(kfd, &c, 1) < 0)
    {
      perror("read():");
      exit(-1);
    }

    //linear_=angular_=0;
    ROS_DEBUG("value: 0x%02X\n", c);
    switch(c)
    {
      case KEYCODE_h:
        ROS_DEBUG("h_key: Home");
        ss << "home";
        dirty = true;
        break;
      case KEYCODE_o:
        ROS_DEBUG("o_key: Power Grasp");
        ss << "power";
        dirty = true;
        break;
      case KEYCODE_t:
        ROS_DEBUG("t_key: Thumb with 2 fingers");
        ss << "thumb";
        dirty = true;
        break;
      case KEYCODE_p:
        ROS_DEBUG("p_key: Pinch");
        ss << "pinch";
        dirty = true;
        break;
      case KEYCODE_l:
        ROS_DEBUG("l_key: Lateral");
        ss << "lateral";
        dirty = true;
        break;


      case KEYCODE_y:
        ROS_DEBUG("y_key: Little Tactile");
        ss << "little_tactile";
        dirty = true;
        break;
      case KEYCODE_x:
        ROS_DEBUG("x_key: Middle Tactile");
        ss << "middle_tactile";
        dirty = true;
        break;
      case KEYCODE_c:
        ROS_DEBUG("c_key: Index Tactile");
        ss << "index_tactile";
        dirty = true;
        break;
      case KEYCODE_v:
        ROS_DEBUG("v_key: Thumb Tactile");
        ss << "thumb_tactile";
        dirty = true;
        break;  

      case KEYCODE_b:
        ROS_DEBUG("b_key: Back");
        ss << "back";
        dirty = true;
        break;   
    }

    if(dirty ==true)
    {
      msg.data = ss.str();
      //ROS_INFO("%s", msg.data.c_str());
      cmd_pub_.publish(msg);
      dirty=false;
    }

    else {
      std::cout << "Unknown grasp type" << std::endl;
    }
  }
  return;
}