#include <chrono>
#include <functional>
#include <ros/callback_queue.h>
#include <robair_hw_iface.h>

void controlLoop(RobAirHWInterface &hw, controller_manager::ControllerManager &cm, std::chrono::system_clock::time_point &last_time)
{
    std::chrono::system_clock::time_point current_time = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_time = current_time - last_time;
    ros::Duration elapsed(elapsed_time.count());
    last_time = current_time;

    hw.read(elapsed);
    cm.update(ros::Time::now(), elapsed);
    hw.write();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "robair_base_node");

    RobAirHWInterface hw;
    controller_manager::ControllerManager cm(&hw, hw.nh);

    double control_frequency;
    hw.private_nh.param<double>("control_frequency", control_frequency, 10.0);

    ros::CallbackQueue robair_queue;
    ros::AsyncSpinner robair_spinner(1, &robair_queue);

    std::chrono::system_clock::time_point last_time = std::chrono::system_clock::now();
    ros::TimerOptions control_timer(
        ros::Duration(1 / control_frequency), 
        std::bind(controlLoop, std::ref(hw), std::ref(cm), std::ref(last_time)), 
        &robair_queue);
    ros::Timer control_loop = hw.nh.createTimer(control_timer);
    robair_spinner.start();
    ros::spin();

    return 0;
}