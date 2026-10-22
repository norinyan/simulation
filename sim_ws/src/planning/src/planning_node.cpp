#include "navigator.h"
int main(int argc,char** argv){
 rclcpp::init(argc,argv);
 try{auto node=std::make_shared<rclcpp::Node>("planning_node");auto navigator=planning::createNavigator(*node);rclcpp::spin(node);navigator.reset();}
 catch(const std::exception& e){RCLCPP_ERROR(rclcpp::get_logger("planning_node"),"%s",e.what());rclcpp::shutdown();return 1;}
 rclcpp::shutdown();return 0;
}
