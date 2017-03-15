#include <iostream>
#include <glm/glm.hpp>
#include <glm/gtx/string_cast.hpp>
#include <ros/ros.h>
#include <cyborg_led_sim/rgba.h>

#include "LED.hpp"
#include "LEDGrid.hpp"
#include "Display.hpp"

#define INF 999999

// TODO: Redefine message 
// int hue
// int sat
// int val
// bool fade
// int fadetime

const int N_LEDS = 60;
const int LED_WIDTH = 70;
const int LED_HEIGHT = 70;
const int V_PAD = 5;
const int H_PAD = 5;

LEDGrid* grid;
Display* display;

void compute_grid_size(const int n_leds, float aspect_ratio, int& m, int& n) {
    float min_ratio_diff = INF;
    int rows = 1;
    int cols = n_leds;

    for(int i = 2; i < n_leds; i++) {

        if((n_leds % i) == 0) {
            float div = (float) n_leds / (float) i;
            float ratio = div / (float) i;
            float ratio_diff = abs(aspect_ratio - ratio);
            
            if(ratio_diff < min_ratio_diff) {
                min_ratio_diff = ratio_diff;

                if(div < cols)
                    cols = div;
                if(i > rows)
                    rows = i;
            }
        }
    }

    m = rows;
    n = cols;
}

void callback_RGBA(const cyborg_led_sim::rgba msg) {
    ROS_INFO("Received data");

    for(int i = 0; i < msg.n_vals; i++) {
        grid->get_led(i).set_color(glm::vec4(msg.r[i], msg.g[i], msg.b[i], msg.a[i]));
    }
}

int main(int argc, char** argv) {

    // RGB grid setup
    int m, n;
    compute_grid_size(N_LEDS, 4.0f/3.0f, m, n);

    const int DISPLAY_WIDTH  = (LED_WIDTH  + V_PAD) * n;
    const int DISPLAY_HEIGHT = (LED_HEIGHT + H_PAD) * m;

    display = new Display(DISPLAY_WIDTH + V_PAD, DISPLAY_HEIGHT + H_PAD, "Cyborg LED sim");
    grid = new LEDGrid(m, n, LED_WIDTH, LED_HEIGHT, V_PAD, H_PAD, DISPLAY_HEIGHT); 

    glClearColor(.4f, .4f, .4f, 0.6f);

    // ROS setup
    ros::init(argc, argv, "LED_simulator");
    ros::NodeHandle nh;

    ros::Subscriber sub_rgba = nh.subscribe("RGBA_data", 100, callback_RGBA);

    ros::Rate loop_rate(30);

    while(ros::ok() && !display->is_closed()) {
        ros::spinOnce();

        glClear(GL_COLOR_BUFFER_BIT);
        grid->draw();
        display->update();

        loop_rate.sleep();
    }

    delete grid;
    delete display;

    return 0;
}

