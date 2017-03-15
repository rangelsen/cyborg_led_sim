#include <ros/ros.h>
#include <std_msgs/String.h>
#include <cyborg_led_sim/rgba.h>
#include <iostream>
#include <stdlib.h>
#include <time.h>

#define N_VALS 60

void fill_random(float* channel, const int N) {
    for(int i = 0; i < N; i++) {
        int random = rand() % 10;
        channel[i] = (float) random / 10.0f;
    }
}

// TODO: Find a smarter way to copy data between msg arrays
cyborg_led_sim::rgba generate_message(float* r, float* g, float* b, float* a, const int N) {
    cyborg_led_sim::rgba msg;
    for(int i = 0; i < N; i++) {

    }
    return msg;
}

int main(int argc, char** argv) {

    srand(time(NULL));
    ros::init(argc, argv, "dummy_publisher");
    ros::NodeHandle nh;

    ros::Publisher pub_rgba = nh.advertise<cyborg_led_sim::rgba>("RGBA_data", 100);

    ros::Rate loop_rate(1);

    cyborg_led_sim::rgba msg;

    float r[N_VALS];
    float g[N_VALS];
    float b[N_VALS];
    float a[N_VALS];
    
    while(ros::ok()) {
        fill_random(r, N_VALS);
        fill_random(g, N_VALS);
        fill_random(b, N_VALS);
        fill_random(a, N_VALS);
        cyborg_led_sim::rgba msg = generate_message(r, g, b, a, N_VALS);
        pub_rgba.publish(msg); 

        loop_rate.sleep();
    }
}
