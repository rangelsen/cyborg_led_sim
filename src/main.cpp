#include <iostream>
#include <glm/glm.hpp>
#include <glm/gtx/string_cast.hpp>
#include <ros/ros.h>
#include <cyborg_led_sim/rgba.h>
#include <led_driver/LedCommandArray.h>
#include <opencv2/opencv.hpp>

#include "LED.hpp"
#include "LEDGrid.hpp"
#include "Display.hpp"

#define INF 999999

const int N_LEDS = 180;
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
    for(int i = 0; i < msg.n_vals; i++) {
        grid->get_led(i).set_color(glm::vec4(msg.r[i], msg.g[i], msg.b[i], msg.a[i]));
    }
}

cv::Mat3b ledCmdsToMat(std::vector<led_driver::LedCommand> cmds) {
    
    cv::Mat3b output(1, cmds.size(), cv::Vec3b(0, 0, 0));

    for(int i = 0; i < cmds.size(); i++) {
        output.at<cv::Vec3b>(0, i)[0] = cmds[i].hue;
        output.at<cv::Vec3b>(0, i)[1] = cmds[i].saturation;
        output.at<cv::Vec3b>(0, i)[2] = cmds[i].value;
    }

    return output;
}

float map_value(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void driverCallback(const led_driver::LedCommandArray msg) {


    std::vector<led_driver::LedCommand> led_cmds = msg.data;    
    cv::Mat3b hsv = ledCmdsToMat(led_cmds);



    cv::Mat rgb;
    cv::cvtColor(hsv, rgb, CV_HSV2RGB);

#if DEBUG
    ROS_INFO("[SIM] Received driver data");

    ROS_INFO("[SIM] HSV matrix");
    std::cout << hsv << std::endl;

    ROS_INFO("[SIM] RGB matrix");
    std::cout << rgb << std::endl;
#endif
    
    for(int i = 0; i < rgb.cols; i++) {
        float r = (float) rgb.at<cv::Vec3b>(0, i)[0] / 255.0f;
        float g = (float) rgb.at<cv::Vec3b>(0, i)[1] / 255.0f;
        float b = (float) rgb.at<cv::Vec3b>(0, i)[2] / 255.0f;

        grid->get_led(i).set_color(glm::vec4(r, g, b, 1.0f));
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

    // ros::Subscriber sub_rgba = nh.subscribe("RGBA_data", 100, callback_RGBA);
    ros::Subscriber sub_driver = nh.subscribe("driver", 100, driverCallback);

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

