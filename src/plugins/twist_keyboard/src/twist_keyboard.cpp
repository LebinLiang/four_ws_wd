#include <iostream>
#include "twist_keyboard.hpp"

void Command();
char command = '0';


double data_limit(const double input_value,double low,double high){
  double value;
  if(input_value<low) value = low;
  else if(input_value>high) value = high;
  else value = input_value;
  return value;
}
int scanKeyboard()
{
  system("stty -echo");
  int in;
  struct termios new_settings;
  struct termios stored_settings;
  tcgetattr(0,&stored_settings);
  new_settings = stored_settings;
  new_settings.c_lflag &= (~ICANON);
  new_settings.c_cc[VTIME] = 0;
  tcgetattr(0,&stored_settings);
  new_settings.c_cc[VMIN] = 1;
  tcsetattr(0,TCSANOW,&new_settings);

  in = getchar();

  tcsetattr(0,TCSANOW,&stored_settings);
  system("stty echo");
  return in;
}


void Command() {
  std::cout << "*******************" << std::endl;
  std::cout << "Moving around:" << std::endl
            << "q    w    e" << std::endl
            << "a    s    d" << std::endl
            << "z    x    c" << std::endl
            << "s: set x,y -> 0 w -> rotate_cmd_w" << std::endl
            << "n: disable rotate" << std::endl
            << "m: enable rotate" << std::endl
            << "u: x + 0.1" << std::endl
            << "j: x - 0.1" << std::endl
            << "i: y + 0.1" << std::endl
            << "k: y - 0.1" << std::endl
            << "o: w + 0.1 " << std::endl
            << "l: w - 0.1 " << std::endl
            << "p: rotate_w + 0.2" << std::endl
            << ";: rotate_w - 0.2" << std::endl
            << "esc: exit program" << std::endl;
  std::cout << "*******************" << std::endl;
  while (command != 27) {
    command = scanKeyboard();
  }
}


Keyboard::Keyboard(): nh(""), pnh_("~"){

  cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "twist_keyboard");
  Keyboard k;

  double cmd_x = 0.5;
  double cmd_y = 0.5;
  double cmd_z = 0.5;
  double rotate_cmd_z = 4.5;
  bool enable_rotate = true;
  std::cout << "default cmd: " << "   cmd_x: "<<cmd_x<< "   cmd_y: "<<
               cmd_y<< "   cmd_z: "<<cmd_z<< "   rotate_cmd_z: "<<rotate_cmd_z
               << "   enable_rotate: "<<enable_rotate<<std::endl;

  auto command_thread= std::thread(Command);

  ros::Rate loop_rate(100);


  double set_x = 0.0,set_y = 0.0,set_z = 0.0;
  while (ros::ok())
  {
    ros::spinOnce();
    switch (command) {
      case 'q': {set_x = cmd_x; set_y = -cmd_y;set_z = cmd_z;}  break;
      case 'w': {set_x = cmd_x; set_y = 0;set_z = cmd_z;}  break;
      case 'e': {set_x = cmd_x; set_y = cmd_y;set_z = cmd_z;}  break;
      case 'a': {set_x = 0; set_y = -cmd_y;set_z = cmd_z;}  break;
      case 's': {set_x = 0; set_y = 0;set_z = rotate_cmd_z;}  break;
      case 'd': {set_x = 0; set_y = cmd_y;set_z = cmd_z;}  break;
      case 'z': {set_x = -cmd_x; set_y = -cmd_y;set_z = cmd_z;}  break;
      case 'x': {set_x = -cmd_x; set_y = 0;set_z = cmd_z;}break;
      case 'c': {set_x = -cmd_x; set_y = cmd_y;set_z = cmd_z;}  break;
      case 'u': {cmd_x +=0.1; }  break;
      case 'j': {cmd_x =data_limit(cmd_x - 0.1,0,30);}  break;
      case 'i': {cmd_y +=0.1; }  break;
      case 'k': {cmd_y =data_limit(cmd_y - 0.1,0,30);}  break;
      case 'o': {cmd_z =data_limit(cmd_z + 0.1,-30,30);;}  break;
      case 'l': {cmd_z =data_limit(cmd_z - 0.1,-30,30); }  break;
      case 'p': {rotate_cmd_z +=0.2; }  break;
      case ';': {rotate_cmd_z =data_limit(rotate_cmd_z - 0.2,0,30);}  break;
      case 'n': {enable_rotate = 0; }  break;
      case 'm': {enable_rotate = 1; }  break;


      case 27:
        if (command_thread.joinable()){
          command_thread.join();
          //return;
        }
      default:
        break;
    }

   if(command=='o'||command=='l'||command=='u'||command=='j'||command=='i'||command=='k'){
      std::cout<<"set default cmd as: "<<" cmd_x: "<<cmd_x<<"    cmd_y: "<<cmd_y<<"    cmd_z: "<<cmd_z<<std::endl;
   }
   else if (command=='q'||command=='w'||command=='e'||command=='a'||command=='s'||command=='d'||
            command=='z'||command=='x'||command=='c') {
     std::cout<<"send cmd as: "<<" set_x: "<<set_x<<"    set_y: "<<set_y<<"    set_z: "<<(enable_rotate?set_z:0)<<std::endl;
   }
   else if (command=='p'||command==';') {
     std::cout<<"set default rotate_w as: "<<" rotate_cmd_z: "<<rotate_cmd_z<<std::endl;
   }
   else if (command=='n'||command=='m') {
     std::cout<<"set rotate mode as: "<<enable_rotate<<std::endl;
   }
   command = '0';

    k.WriteData(set_x,set_y,enable_rotate?set_z:0);
    k.PublishCmd();
    loop_rate.sleep();
  }
}

