#include <iostream>
#include <assert.h>
#include <math.h>

#include "LEDGrid.hpp"
#include "LED.hpp"

using namespace std;

LEDGrid::LEDGrid(const int m, const int n, const int w_led, const int h_led,
                 const int v_pad, const int h_pad, const int h) : m_(m),
                                                                  n_(n),
                                                                  w_led_(w_led),
                                                                  h_led_(h_led),
                                                                  v_pad_(v_pad),
                                                                  h_pad_(h_pad),
                                                                  h_(h) {

    for(int i = 0; i < m_; i++) {

        for(int j = 0; j < n_; j++) {
            float x_pos = v_pad_ + (w_led_ + v_pad_) * j;
            float y_pos = h_ - (h_led_ + h_pad_) * i;

            leds_.push_back(LED(x_pos, y_pos, w_led_, h_led_));
        }
    }
}

void LEDGrid::draw() {
    for(size_t i = 0; i < leds_.size(); i++) {
        leds_.at(i).draw();
    }
}

LED& LEDGrid::get_led(int m, int n) {
    assert(m < m_);
    assert(n < n_);
    return leds_.at(m * n_ + n);
}

