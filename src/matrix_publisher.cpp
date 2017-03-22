#include <ros/ros.h>
#include <cyborg_led_sim/bool_array.h>
#include <cyborg_led_sim/bool_matrix.h>
#include <iostream>
#include <stdlib.h>
#include <time.h>
#include <vector>

#define N_VALS       60
#define TIME_HORIZON 10

/**
    param n_vals: corresponds to the number of electrodes
                  data is obtained from. Number of rows in
                  the matrix
    param horizon: the number of time steps back in time
                   for which the data is still valid
*/
cyborg_led_sim::bool_matrix generate_message(const int n_vals, const int horizon) {
    cyborg_led_sim::bool_array v;
    cyborg_led_sim::bool_matrix output;

    for(int i = 0; i < n_vals; i++) {
        for(int j = 0; j < horizon; j++) {
            v.data.push_back(2.0f);
        }
    }

    return output;
}

int main(int argc, char** argv) {
    ROS_INFO("[matrix_publisher]");

    srand(time(NULL));
    ros::init(argc, argv, "matrix_publisher");
    ros::NodeHandle n;

    ros::Rate loop_rate(2);

    cyborg_led_sim::bool_matrix spiked_msg;

    while(ros::ok()) {
        loop_rate.sleep();
    }
}
