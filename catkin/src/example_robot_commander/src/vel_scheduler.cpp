
// try this, e.g. with roslaunch stdr_launchers server_with_map_and_gui_plus_robot.launch
// or:  roslaunch cwru_376_launchers stdr_glennan_2.launch 
// watch resulting velocity commands with: rqt_plot /robot0/cmd_vel/linear/x (or jinx/cmd_vel...)

//intent of this program: modulate the velocity command to comply with a speed limit, v_max,
// acceleration limits, +/-a_max, and come to a halt gracefully at the end of
// an intended line segment

// notes on quaternions:
/*
From:
http://www.euclideanspace.com/maths/geometry/rotations/conversions/angleToQuaternion/

qx = ax * sin(angle/2)
qy = ay * sin(angle/2)
qz = az * sin(angle/2)
qw = cos(angle/2)


so, quaternion in 2-D plane (x,y,theta):
ax=0, ay=0, az = 1.0

qx = 0;
qy = 0;
qz = sin(angle/2)
qw = cos(angle/2)

therefore, theta = 2*atan2(qz,qw)
*/

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <math.h>


// set some dynamic limits...
const double v_max = 5.0; //1m/sec is a slow walk
const double v_min = 0.1; // if command velocity too low, robot won't move
const double a_max = 0.1; //1m/sec^2 is 0.1 g's
//const double a_max_decel = 0.1; // TEST
const double omega_max = 1.0; //1 rad/sec-> about 6 seconds to rotate 1 full rev
const double alpha_max = 0.5; //0.5 rad/sec^2-> takes 2 sec to get from rest to full omega
const double DT = 0.050; // choose an update rate of 20Hz; go faster with actual hardware

double odom_vel_ = 0.0; // measured/published system speed
double odom_omega_ = 0.0; // measured/published system yaw rate (spin)
double odom_x_ = 0.0;
double odom_y_ = 0.0;
double odom_phi_ = 0.0;
double dt_odom_ = 0.0;
ros::Time t_last_callback_;
double dt_callback_=0.0;

// receive odom messages and strip off the components we want to use
// tested this OK w/ stdr

// receive the pose and velocity estimates from the simulator (or the physical robot)
// copy the relevant values to global variables, for use by "main"
// Note: stdr updates odom only at 10Hz; Jinx is 50Hz (?)
void odomCallback(const nav_msgs::Odometry& odom_rcvd) {
    //here's a trick to compute the delta-time between successive callbacks:
    dt_callback_ = (ros::Time::now() - t_last_callback_).toSec();
    t_last_callback_ = ros::Time::now(); // let's remember the current time, and use it next iteration

    if (dt_callback_ > 0.15) { // on start-up, and with occasional hiccups, this delta-time can be unexpectedly large
        dt_callback_ = 0.1; // can choose to clamp a max value on this, if dt_callback is used for computations elsewhere
        ROS_WARN("large dt; dt = %lf", dt_callback_); // let's complain whenever this happens
    }
    odom_vel_ = odom_rcvd.twist.twist.linear.x;
    odom_omega_ = odom_rcvd.twist.twist.angular.z;
    odom_x_ = odom_rcvd.pose.pose.position.x;
    odom_y_ = odom_rcvd.pose.pose.position.y;
    double quat_z = odom_rcvd.pose.pose.orientation.z;
    double quat_w = odom_rcvd.pose.pose.orientation.w;
    odom_phi_ = 2.0*atan2(quat_z, quat_w); // cheap conversion from quaternion to heading for planar motion
    ROS_INFO("odom CB: x = %f, y= %f, phi = %f, v = %f, omega = %f", odom_x_, odom_y_, odom_phi_, odom_vel_, odom_omega_);
}

//this will stop the robot and reset the values to zero
void resetCmdValues(geometry_msgs::Twist cmd_vel){
    cmd_vel.linear.x = 0.0; // initialize these values to zero
    cmd_vel.linear.y = 0.0;
    cmd_vel.linear.z = 0.0;
    cmd_vel.angular.x = 0.0;
    cmd_vel.angular.y = 0.0;
    cmd_vel.angular.z = 0.0;
}


void checkOdom(ros::Rate rtimer){
    // let's wait for odom callback to start getting good values...
    odom_omega_ = 1000000; // absurdly high
    ROS_INFO("waiting for valid odom callback...");
    t_last_callback_ = ros::Time::now(); // initialize reference for computed update rate of callback
    while (odom_omega_ > 1000) {
        rtimer.sleep();
        ros::spinOnce();
    }
}
//
void moveOnSegment(int distance, ros::Publisher vel_cmd_publisher,ros::Rate rtimer){
    checkOdom(rtimer);
    double segment_length = distance;
    double segment_length_done = 0.0; // need to compute actual distance travelled within the current segment
    double start_x = 0.0; // fill these in with actual values once odom message is received
    double start_y = 0.0; // subsequent segment start coordinates should be specified relative to end of previous segment
    double start_phi = 0.0;
    double scheduled_vel = 0.0; //desired vel, assuming all is per plan
    double new_cmd_vel = 0.1; // value of speed to be commanded; update each iteration
    double new_cmd_omega = 0.0; // update spin rate command as well
    geometry_msgs::Twist cmd_vel; //create a variable of type "Twist" to publish speed/spin commands
    resetCmdValues(cmd_vel);
    ROS_INFO("received odom message; proceeding");
    start_x = odom_x_;
    start_y = odom_y_;
    start_phi = odom_phi_;
    ROS_INFO("start pose: x %f, y= %f, phi = %f", start_x, start_y, start_phi);
    // compute some properties of trapezoidal velocity profile plan:
    double T_accel = v_max / a_max; //...assumes start from rest
    double T_decel = v_max / a_max; //(for same decel as accel); assumes brake to full halt
    double dist_accel = 0.5 * a_max * (T_accel * T_accel); //distance rqd to ramp up to full speed
    double dist_decel = 0.5 * a_max * (T_decel * T_decel);; //same as ramp-up distance
    double dist_const_v = segment_length - dist_accel - dist_decel; //if this is <0, never get to full spd
    double T_const_v = dist_const_v / v_max; //will be <0 if don't get to full speed
    double T_segment_tot = T_accel + T_decel + T_const_v; // expected duration of this move
    while (ros::ok()) 
    {
        ros::spinOnce(); // allow callbacks to populate fresh data
        // compute distance travelled so far:
        double delta_x = odom_x_ - start_x;
        double delta_y = odom_y_ - start_y;
        segment_length_done = sqrt(delta_x * delta_x + delta_y * delta_y);
        ROS_INFO("dist travelled: %f", segment_length_done);
        double dist_to_go = segment_length - segment_length_done;

        if (dist_to_go<= 0.0) { // at goal, or overshot; stop!
            scheduled_vel=0.0;
        }
        else if (dist_to_go <= dist_decel) { //possibly should be braking to a halt
            scheduled_vel = sqrt(2 * dist_to_go * a_max);
            ROS_INFO("braking zone: v_sched = %f",scheduled_vel);
        }
        else { // not ready to decel, so target vel is v_max, either accel to it or hold it
            scheduled_vel = v_max;
        }
        if (odom_vel_ < scheduled_vel) {  // maybe we halted, e.g. due to estop or obstacle;
            double v_test = odom_vel_ + a_max*dt_callback_; // if callbacks are slow, this could be abrupt
            new_cmd_vel = (v_test < scheduled_vel) ? v_test : scheduled_vel; //choose lesser of two options
        } else if (odom_vel_ > scheduled_vel) { //travelling too fast--this could be trouble
            ROS_INFO("odom vel: %f; sched vel: %f",odom_vel_,scheduled_vel); //debug/analysis output; can comment this out
            double v_test = odom_vel_ - 1.2 * a_max*dt_callback_; //moving too fast--try decelerating faster than nominal a_max
            new_cmd_vel = (v_test > scheduled_vel) ? v_test : scheduled_vel; //choose larger of two don't overshoot scheduled_vel
        } else {
            new_cmd_vel = scheduled_vel; //silly third case: this is already true, if here.  Issue the scheduled velocity
        }
        ROS_INFO("cmd vel: %f",new_cmd_vel); // debug output
        cmd_vel.linear.x = new_cmd_vel;
        cmd_vel.angular.z = new_cmd_omega; // spin command; always zero, in this example
        if (dist_to_go <= 0.0) { //uh-oh...went too far already!
            cmd_vel.linear.x = 0.0;  //command vel=0
        }
        vel_cmd_publisher.publish(cmd_vel); // publish the command to robot0/cmd_vel
        rtimer.sleep(); // sleep for remainder of timed iteration
        if (dist_to_go <= 0.0) break; // halt this node when this segment is complete.
    }
    resetCmdValues(cmd_vel);
    vel_cmd_publisher.publish(cmd_vel);
    ROS_INFO("completed move distance");
}

void turnInDeg(double deg,ros::Publisher ang_cmd_publisher,ros::Rate rtimer){
    checkOdom(rtimer);
    //insert setup variables
    double phi_rotate = deg ;//calculates the rotatin in quaternion
    double rotate_done = 0.0; // need to compute actual distance travelled within the current segment
    double start_z = 0.0; // fill these in with actual values once odom message is received
    double scheduled_omega = 0.0; //desired vel, assuming all is per plan
    double new_cmd_omega = 0.0; // update spin rate command as well
    geometry_msgs::Twist cmd_ang; //create a variable of type "Twist" to publish speed/spin commands
    resetCmdValues(cmd_ang);
    ROS_INFO("received odom message; proceeding");
    double start_phi = odom_phi_;
    double phi_done = 0;
    ROS_INFO("start pose: phi = %f", start_phi);
    // compute some properties of trapezoidal velocity profile plan:
    double T_alpha = omega_max/alpha_max;
    double phi_alpha = 0.5 * a_max * (T_alpha * T_lpha); //angle required to reach omega max, might be wrong
    double phi_const_omega = rotateQ - dist_accel - dist_decel; //if this is <0, never get to full spd
    double T_const_omega = phi_const_omega / omega_max; //will be <0 if don't get to full speed
    double T_rotate_tot = T_alpha + T_alpha + T_const_omega; // expected duration of this move
    while(ros::ok()){
        ros::spinOnce(); //allows callback
        phi_done = start_z + start_phi - odom_phi;
        double phi_to_go = phi_rotate - phi_done;
        ROS_INFO("Phi rotated: %f", alpha_done);
        if(phi_to_go<= 0.0){
            scheduled_omega = 0.0;
        }
        else if(phi_togo <= phi_alpha ){
            //break to halt
        }
        else{
            //not ready to decel so omegamax is target
        }
        if(odom_omega_ < scheduled_omega){
            //handle estop or obstacle halts

        }
        else if(odom_omega_ > scheduled_omega){
            //travelling too fast
        }
        else{
            new_cmd_omega = scheduled_omega;
        }

        cmd_
        //turn till break
        ang_cmd_publisher.publish(cmd_ang);
    }

    
    //initial reset
//    resetCmdValues();

}

int main(int argc, char **argv) {
    ros::init(argc, argv, "vel_scheduler"); // name of this node will be "minimal_publisher1"
    ros::NodeHandle nh; // get a ros nodehandle; standard yadda-yadda
    ros::Publisher vel_cmd_publisher = nh.advertise<geometry_msgs::Twist>("robot0/cmd_vel", 1);
    ros::Subscriber sub = nh.subscribe("/robot0/odom", 1, odomCallback);
    ros::Rate rtimer(1 / DT); // frequency corresponding to chosen sample period DT; the main loop will run this fast
    //move the robot from the elevator to the first turn
    moveOnSegment(5.5,vel_cmd_publisher,rtimer);
    //turn the robot 90 deg to the right
    turnInDeg(90,rtimer);
    //move the robot down the hallway to the next turn

    //turn the robot to the hallway with the vending machines

    //move the robot towards the vending machine
}
