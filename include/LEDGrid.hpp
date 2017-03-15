#ifndef LEDGRID_H
#define LEDGRID_H

#include <vector>

#include "LED.hpp"

class LEDGrid {

private:
    int m_, n_;
    int v_pad_, h_pad_;
    int w_led_, h_led_;
    int h_;

    std::vector<LED> leds_;

public:
    LEDGrid(const int m, const int n, const int w_led, const int h_led,
            const int v_pad, const int h_pad, const int h);
    void draw();
    LED& get_led(int m, int n);
};

#endif // LEDGRID_H
