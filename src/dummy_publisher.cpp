#include <ros/ros.h>
#include <std_msgs/String.h>
#include <cyborg_led_sim/rgba.h>
#include <iostream>
#include <stdlib.h>
#include <time.h>
#include <vector>

#define N_VALS 60

void fill_random(std::vector<double>& channel, const int N) {
    for(int i = 0; i < N; i++) {
        int random = rand() % 10;
        channel.push_back((float) random / 10.0f);
    }
}

// TODO: Find a smarter way to copy data between program and msg arrays
cyborg_led_sim::rgba generate_message(std::vector<double> r, std::vector<double> g, std::vector<double> b, std::vector<double> a, const int N) {
    cyborg_led_sim::rgba msg;
    msg.n_vals = N_VALS;

    msg.r = r;
    msg.g = g;
    msg.b = b;
    msg.a = a;
/*
    for(int i = 0; i < N; i++) {
        msg.r[i] = r[i]; 
        msg.g[i] = g[i]; 
        msg.b[i] = b[i]; 
        msg.a[i] = a[i]; 
    }
*/
    msg.stamp = ros::Time::now();

    return msg;
}

int main(int argc, char** argv) {

    srand(time(NULL));
    ros::init(argc, argv, "dummy_publisher");
    ros::NodeHandle nh;

    ros::Publisher pub_rgba = nh.advertise<cyborg_led_sim::rgba>("RGBA_data", 100);

    ros::Rate loop_rate(2);

    cyborg_led_sim::rgba msg;

    std::vector<double> r; 
    std::vector<double> g; 
    std::vector<double> b; 
    std::vector<double> a; 

    while(ros::ok()) {
        fill_random(r, N_VALS);
        fill_random(g, N_VALS);
        fill_random(b, N_VALS);
        fill_random(a, N_VALS);
        cyborg_led_sim::rgba msg = generate_message(r, g, b, a, N_VALS);
        pub_rgba.publish(msg); 

        std::cout << "#" << std::endl;
        loop_rate.sleep();
    }

    return 0;
}
