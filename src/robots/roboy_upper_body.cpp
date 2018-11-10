#include "kindyn/robot.hpp"
#include <thread>
#include <roboy_communication_middleware/MotorCommand.h>
#include <roboy_communication_middleware/ControlMode.h>
#include <common_utilities/CommonDefinitions.h>
#include <roboy_communication_control/SetControllerParameters.h>
#include <roboy_communication_middleware/MotorConfigService.h>
#include <roboy_communication_middleware/ArmStatus.h>
#include <std_msgs/Float32.h>

using namespace std;

class RoboyUpperBody: public cardsflow::kindyn::Robot{
public:
    /**
     * Constructor
     * @param urdf path to urdf
     * @param cardsflow_xml path to cardsflow xml
     */
    RoboyUpperBody(string urdf, string cardsflow_xml){
        if (!ros::isInitialized()) {
            int argc = 0;
            char **argv = NULL;
            ros::init(argc, argv, "roboy_upper_body");
        }
        nh = ros::NodeHandlePtr(new ros::NodeHandle);
        motor_command = nh->advertise<roboy_communication_middleware::MotorCommand>("/roboy/middleware/MotorCommand",1);
        motor_control_mode = nh->serviceClient<roboy_communication_middleware::ControlMode>("/roboy/shoulder_left/middleware/ControlMode");
        motor_config = nh->serviceClient<roboy_communication_middleware::MotorConfigService>("/roboy/shoulder_left/middleware/MotorConfig");
        sphere_left_axis0_params = nh->serviceClient<roboy_communication_control::SetControllerParameters>("/sphere_left_axis0/sphere_left_axis0/params");
        sphere_left_axis1_params = nh->serviceClient<roboy_communication_control::SetControllerParameters>("/sphere_left_axis1/sphere_left_axis1/params");
        sphere_left_axis2_params = nh->serviceClient<roboy_communication_control::SetControllerParameters>("/sphere_left_axis2/sphere_left_axis2/params");
        elbow_left = nh->advertise<std_msgs::Float32>("/roboy/middleware/elbow_left/JointAngle",1);
        wrist_left = nh->advertise<std_msgs::Float32>("/roboy/middleware/wrist_left/JointAngle",1);
        arm_status = nh->subscribe("/roboy/middleware/ArmStatus",1, &RoboyUpperBody::ArmStatus, this);
        vector<string> joint_names;
        nh->getParam("joint_names", joint_names);
        init(urdf,cardsflow_xml,joint_names);
        // if we do not get the robot state externally, we use the forwardKinematics function to integrate the robot state
        nh->getParam("external_robot_state", external_robot_state);
        roboy_communication_middleware::ControlMode msg;
        msg.request.control_mode = DISPLACEMENT;
        msg.request.setPoint = 0;
        if(!motor_control_mode.call(msg))
            ROS_WARN("failed to change control mode to velocity");
    };

    void ArmStatus(const roboy_communication_middleware::ArmStatusConstPtr &msg){
        q[7] = msg->elbow_joint_angle*M_PI/180.0;
        q[8] = msg->wrist_joint_angle*M_PI/180.0;
    }

    /**
     * Updates the robot model and integrates the robot model using the forwardKinematics function
     * with a small step length
     */
    void read(){
        update();
        if(!external_robot_state)
            forwardKinematics(0.001);
    };
    /**
     * Sends motor commands to the real robot
     */
    void write(){
        roboy_communication_middleware::MotorCommand msg;
        msg.id = SHOULDER_LEFT;
        msg.motors = left_arm_motors;
        stringstream str;
        for (int i = 0; i < left_arm_motors.size(); i++) {
            msg.setPoints.push_back(cable_forces[i]);
        }
        ROS_INFO_STREAM_THROTTLE(1,cable_forces.transpose().format(fmt));
        motor_command.publish(msg);

        std_msgs::Float32 msg2;
        msg2.data = q_target[7]*180.0/M_PI;
        elbow_left.publish(msg2);
        msg2.data = q_target[8]*180.0/M_PI;
        wrist_left.publish(msg2);
    };
    ros::NodeHandlePtr nh; /// ROS nodehandle
    ros::Publisher motor_command; /// motor command publisher
    ros::Publisher wrist_left, elbow_left;
    ros::Subscriber arm_status;
    ros::ServiceClient motor_control_mode, motor_config, sphere_left_axis0_params, sphere_left_axis1_params, sphere_left_axis2_params;
    bool external_robot_state; /// indicates if we get the robot state externally
    vector<short unsigned int> left_arm_motors = {0,1,2,3,4,5,6,7,8};
};

/**
 * controller manager update thread. Here you can define how fast your controllers should run
 * @param cm pointer to the controller manager
 */
void update(controller_manager::ControllerManager *cm) {
    ros::Time prev_time = ros::Time::now();
    ros::Rate rate(100); // changing this value affects the control speed of your running controllers
    while (ros::ok()) {
        const ros::Time time = ros::Time::now();
        const ros::Duration period = time - prev_time;
        cm->update(time, period);
        prev_time = time;
        rate.sleep();
    }
}

int main(int argc, char *argv[]) {
    if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "cardsflow_example_robot");
    }
    ros::NodeHandle nh;
    string urdf, cardsflow_xml;
    if(nh.hasParam("urdf_file_path") && nh.hasParam("cardsflow_xml")) {
        nh.getParam("urdf_file_path", urdf);
        nh.getParam("cardsflow_xml", cardsflow_xml);
    }else {
        ROS_FATAL("USAGE: rosrun kindyn test_robot path_to_urdf path_to_viapoints_xml");
        return 1;
    }
    ROS_INFO("\nurdf file path: %s\ncardsflow_xml %s", urdf.c_str(), cardsflow_xml.c_str());

    RoboyUpperBody robot(urdf, cardsflow_xml);

    controller_manager::ControllerManager cm(&robot);

    // we need an additional update thread, otherwise the controllers won't switch
    thread update_thread(update, &cm);
    update_thread.detach();

    ROS_INFO("STARTING ROBOT MAIN LOOP...");

    while(ros::ok()){
        robot.read();
        robot.write();
        ros::spinOnce();
    }

    ROS_INFO("TERMINATING...");

    update_thread.join();

    return 0;
}
