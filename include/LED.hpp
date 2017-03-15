#ifndef LED_H
#define LED_H

#include <GL/gl.h>
#include <glm/glm.hpp>

class LED {

private:
    glm::vec3 pos_;
    glm::vec4 color_;
    float w_;
    float h_;

    GLuint vao_;
    GLuint vbo_;

public:
    LED(const LED& rhs);
    LED();
    LED(float x, float y, float w, float h);
    ~LED();

    void draw();
    void print();
    glm::vec3 get_pos() const;
    glm::vec4 get_color() const;
    void set_color(glm::vec4 color);
    float get_width() const;
    float get_height() const;
    void setup();
};

#endif // LED_H
