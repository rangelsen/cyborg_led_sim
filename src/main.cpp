#include <iostream>
#include <glm/glm.hpp>
#include <glm/gtx/string_cast.hpp>

#include "LED.hpp"
#include "LEDGrid.hpp"
#include "Display.hpp"

#define INF 999999

using namespace std;

const int N_LEDS = 60;
const int LED_WIDTH = 70;
const int LED_HEIGHT = 70;
const int V_PAD = 5;
const int H_PAD = 5;

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

int main(int argc, char** argv) {

    int m, n;
    compute_grid_size(N_LEDS, 4.0f/3.0f, m, n);

    const int DISPLAY_WIDTH  = (LED_WIDTH  + V_PAD) * n;
    const int DISPLAY_HEIGHT = (LED_HEIGHT + H_PAD) * m;

    Display display(DISPLAY_WIDTH + V_PAD, DISPLAY_HEIGHT + H_PAD, "Cyborg RGB sim");

    LEDGrid grid(m, n, LED_WIDTH, LED_HEIGHT, V_PAD, H_PAD, DISPLAY_HEIGHT); 
    grid.get_led(4, 2).set_color(glm::vec4(0.0f, 1.0f, 0.0f, 0.7f));

    glClearColor(.4f, .4f, .4f, 0.6f);

    while(!display.is_closed()) {
        glClear(GL_COLOR_BUFFER_BIT);

        grid.draw();
        display.update();
    }

    return 0;
}
