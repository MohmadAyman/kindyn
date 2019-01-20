#include "kindyn/robot.hpp"
#include <thread>
#include <roboy_middleware_msgs/MotorCommand.h>
#include <roboy_simulation_msgs/GymStep.h>
#include <roboy_simulation_msgs/GymReset.h>
#include <roboy_simulation_msgs/GymGoal.h>

#define NUMBER_OF_MOTORS 8
#define SPINDLERADIUS 0.00575
#define msjMeterPerEncoderTick(encoderTicks) (((encoderTicks)/(4096.0)*(2.0*M_PI*SPINDLERADIUS)))
#define msjEncoderTicksPerMeter(meter) ((meter)*(4096.0)/(2.0*M_PI*SPINDLERADIUS))

using namespace std;

class MsjPlatform: public cardsflow::kindyn::Robot{
public:
    /**
     * Constructor
     * @param urdf path to urdf
     * @param cardsflow_xml path to cardsflow xml
     */

    MsjPlatform(string urdf, string cardsflow_xml){
        if (!ros::isInitialized()) {
            int argc = 0;
            char **argv = NULL;
            ros::init(argc, argv, "msj_platform");
        }
        nh = ros::NodeHandlePtr(new ros::NodeHandle);
        motor_command = nh->advertise<roboy_middleware_msgs::MotorCommand>("/roboy/middleware/MotorCommand",1);
        // OpenAI gym services
        gym_step = nh->advertiseService("/gym_step", &MsjPlatform::GymStepService,this);
        gym_reset = nh->advertiseService("/gym_reset", &MsjPlatform::GymResetService,this);
        gym_goal = nh->advertiseService("/gym_goal", &MsjPlatform::GymGoalService,this);
        // first we retrieve the active joint names from the parameter server
        vector<string> joint_names;
        nh->getParam("joint_names", joint_names);
        // then we initialize the robot with the cardsflow xml and the active joints
        init(urdf,cardsflow_xml,joint_names);
        // if we do not get the robot state externally, we use the forwardKinematics function to integrate the robot state
        nh->getParam("external_robot_state", external_robot_state);
        // Get the limits of joints
        string path = ros::package::getPath("robots");
        path+="/msj_platform/joint_limits.txt";
        FILE*       file = fopen(path.c_str(),"r");
        cout << "file:" << file;
        if (NULL != file) {
            fscanf(file, "%*[^\n]\n", NULL);
            cout << "fscanf"  << fscanf(file, "%*[^\n]\n", NULL)<< endl;
            float qx,qy;
            int i =0;
            cout << "fscanf file qx qy: "<< fscanf(file,"%f %f\n",&qx,&qy) << endl;
            while(fscanf(file,"%f %f\n",&qx,&qy) == 2){
                limits[0].push_back(qx);
                limits[1].push_back(qy);
                cout << qx << "\t" << qy << endl;
                i++;
            }
            printf("read %d joint limit values\n", i);
        }else{
            cout << "could not open " << path << endl;
        }

        update();
        for(int i=0;i<NUMBER_OF_MOTORS;i++)
            l_offset[i] = l[i];

    };

    /**
     * Updates the robot model and if we do not use gazebo for simulation, we integrate using the forwardKinematics function
     * with a small step length
     */
    void read(){
        update();
        if(!external_robot_state)
            forwardKinematics(0.0001);
    };

    /**
     * Sends motor commands to the real robot
     */
    void write(){
        roboy_middleware_msgs::MotorCommand msg;
        msg.id = 5;
        stringstream str;
        for (int i = 0; i < NUMBER_OF_MOTORS; i++) {
            msg.motors.push_back(i);
            double l_change = l[i]-l_offset[i];
            msg.set_points.push_back(-msjEncoderTicksPerMeter(l_change)); //
            str << l_change << "\t";
        }
		//ROS_INFO_STREAM_THROTTLE(1,str.str());

        motor_command.publish(msg);
    };

    int pnpoly(vector<double> limits_x, vector<double> limits_y, double testx, double testy){
        int i, j, c = 0;
        for (i = 0, j = limits_x.size()-1; i < limits_x.size(); j = i++) {
            if ( ((limits_y[i]>testy) != (limits_y[j]>testy)) &&
                 (testx < (limits_x[j]-limits_x[i]) * (testy-limits_y[i]) / (limits_y[j]-limits_y[i]) + limits_x[i]) )
                c = !c;
        }
        return c;
    }
    bool GymGoalService(roboy_simulation_msgs::GymGoal::Request &req,
                        roboy_simulation_msgs::GymGoal::Response &res){
        bool not_feasible = true;
        float q0= 0.0,q1= 0.0,q2 = 0.0;
        double min[3] = {0,0,-1}, max[3] = {0,0,1};
        for(int i=0;i<limits[0].size();i++){
            if(limits[0][i]<min[0])
                min[0] = limits[0][i];
            if(limits[1][i]<min[1])
                min[1] = limits[1][i];
            if(limits[0][i]>max[0])
                max[0] = limits[0][i];
            if(limits[1][i]>max[1])
                max[1] = limits[1][i];
        }
        srand(static_cast<unsigned >(time(0)));
        while(not_feasible) {
            /*
            float q0 = rand() / (float) RAND_MAX * (max[0] - min[0]) + min[0];
            float q1 = rand() / (float) RAND_MAX * (max[1] - min[1]) + min[1];
            float q2 = rand() / (float) RAND_MAX * (max[2] - min[2]) + min[2];
             */
            q0 = min[0] + static_cast<float> (rand() /(static_cast<float> (RAND_MAX/(max[0]-min[0]))));
            q1 = min[1] + static_cast<float> (rand() /(static_cast<float> (RAND_MAX/(max[1]-min[1]))));
            q2 = min[2] + static_cast<float> (rand() /(static_cast<float> (RAND_MAX/(max[2]-min[2]))));
            if (pnpoly(limits[0], limits[1], q0, q1))
                not_feasible = false;
        }
        res.q.push_back(q0);
        res.q.push_back(q1);
        res.q.push_back(q1);

    }

    bool GymStepService(roboy_simulation_msgs::GymStep::Request &req,
                        roboy_simulation_msgs::GymStep::Response &res){
        
        if(req.set_points.size() != 0){ //If no set_point set then jut return observation.
        	//ROS_INFO("Gymstep is called");
        	update();
	        for(int i=0; i< number_of_cables; i++){
	        	//Set the commanded tendon velocity from RL agent to simulation 
	        	Ld[0][i] = req.set_points[i];
	        }
	        if(!external_robot_state)
	            forwardKinematics(req.step_size);

	        //ROS_INFO_STREAM_THROTTLE(5, "Ld = " << Ld.format(fmt));

	        ROS_INFO_STREAM_THROTTLE(5, "Ld = \n" << Ld[0].format(fmt));

	        write();
	        //ROS_INFO("Gymstep is done");
	    }
        for(int i=0; i< number_of_dofs; i++ ){
        	res.q.push_back(q[i]);
        	res.qdot.push_back(qd[i]);
        }
        if(pnpoly(limits[0],limits[1],q[0],q[1])){
            //task space is feasible
            res.feasible = true;
        }
        else{
            //task space is not feasible
            res.feasible = false;
        }
        return true;
    }
    bool GymResetService(roboy_simulation_msgs::GymReset::Request &req,
                        roboy_simulation_msgs::GymReset::Response &res){
    	//ROS_INFO("Gymreset is called");      
    	integration_time = 0.0;
        for(int i=0; i< number_of_dofs; i++){
	       	//Set the commanded tendon velocity from RL agent to simulation 
	       	//Set the joint states to arrange the initial condition or reset it. Not the q and qdot
			joint_state[i][0] = 0.0;		//Velocity of ith joint
			joint_state[i][1] = 0.0;		//Position of ith joint
            q[i] = 0.0;
            qd[i] = 0.0;
	    }

	    for(int i=0; i< number_of_cables; i++){
	        //Set the commanded tendon velocity from RL agent to simulation 
	        motor_state[i][0] =0.0; 	//Length of the ith cable
			motor_state[i][1] = 0.0;	//Velocity of the ith cable
	    }

	 	update();

        //ROS_INFO_STREAM_THROTTLE(5, "q = \n" << q.format(fmt));
        for(int i=0; i< number_of_dofs; i++ ){
        	res.q.push_back(q[i]);
        	res.qdot.push_back(qd[i]);
        }
        //ROS_INFO("Gymreset is done");
        return true;
    }
    bool external_robot_state; /// indicates if we get the robot state externally
    ros::NodeHandlePtr nh; /// ROS nodehandle
    ros::Publisher motor_command; /// motor command publisher
    ros::ServiceServer gym_step; //OpenAI Gym training environment step function, ros service instance
    ros::ServiceServer gym_reset; //OpenAI Gym training environment reset function, ros service instance
    ros::ServiceServer gym_goal; //OpenAI Gym training environment sets new goal function, ros service instance
    vector<double> limits[3];
    double l_offset[NUMBER_OF_MOTORS];

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

    MsjPlatform robot(urdf, cardsflow_xml);

    controller_manager::ControllerManager cm(&robot);

    // we need an additional update thread, otherwise the controllers won't switch
    thread update_thread(update, &cm);
    update_thread.detach();

    ROS_INFO("STARTING ROBOT MAIN LOOP...");

    while(ros::ok()){
        //robot.read();
        //robot.write();
        ros::spinOnce();
    }

    ROS_INFO("TERMINATING...");

    update_thread.join();

    return 0;
}
