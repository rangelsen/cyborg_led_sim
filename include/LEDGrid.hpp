#ifndef LEDGRID_H
#define LEDGRID_H

#include <vector>

#include "LED.hpp"

/*
 * Class to manage a grid of LEDs with.
 */
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
    LED& get_led(int i);
};

#endif // LEDGRID_H
