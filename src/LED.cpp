#include <GL/glew.h>
#include <GL/gl.h>
#include <glm/glm.hpp>
#include <glm/gtx/string_cast.hpp>
#include <iostream>

#include "LED.hpp"

#define Z_POS 0.0f

using namespace std;

LED::LED(float x, float y, float w, float h) : w_(w), h_(h) {
                                                     
    pos_   = glm::vec3(x, y, Z_POS);
    color_ = glm::vec4(1.0f, 0.0f, 0.0f, 0.5f);

    setup();
}

LED::LED(const LED& rhs) {
    pos_ = rhs.get_pos();
    color_ = rhs.get_color();
    w_ = rhs.get_width();
    h_ = rhs.get_height();

    setup();    
}

LED::LED() {
    pos_ = glm::vec3(0.0f, 0.0f, Z_POS);
    color_ = glm::vec4(0.0f, 0.0f, 0.0f, 1.0f);
    w_ = 0;
    h_ = 0;

    setup();
}

LED::~LED() {
    glDeleteVertexArrays(1, &vao_);
    glDeleteBuffers(1, &vbo_);
}

void LED::setup() {
    glm::vec3 vertices[4] = {
        pos_,
        pos_ + glm::vec3(0.0f, -h_, 0.0f),
        pos_ + glm::vec3(w_, -h_, 0.0f),
        pos_ + glm::vec3(w_, 0.0f, 0.0f)
    };

    glColor4f(color_[0], color_[1], color_[2], color_[3]);

    glGenVertexArrays(1, &vao_);
    glBindVertexArray(vao_);

    glGenBuffers(1, &vbo_);
    glBindBuffer(GL_ARRAY_BUFFER, vbo_);
    glBufferData(GL_ARRAY_BUFFER, 4 * sizeof(vertices[0]), vertices, GL_STATIC_DRAW);
    glEnableClientState(GL_VERTEX_ARRAY);
    glVertexPointer(3, GL_FLOAT, 0, NULL);
    
    glBindBuffer(GL_ARRAY_BUFFER, vbo_);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, NULL);
    glEnableVertexAttribArray(0);

    glBindVertexArray(0);
    glDisableClientState(GL_VERTEX_ARRAY);
}

void LED::draw() {
    glBindVertexArray(vao_);
    glColor4f(color_[0], color_[1], color_[2], color_[3]);
    glDrawArrays(GL_QUADS, 0, 4);
}

void LED::print() {
    
    cout << "LED info: " << endl;
    cout << "pos: " << glm::to_string(pos_) << endl;
    cout << "color: " << glm::to_string(color_) << endl;
}

glm::vec3 LED::get_pos() const {
    return pos_;
}

glm::vec4 LED::get_color() const {
    return color_;
}

void LED::set_color(glm::vec4 color) {
    color_ = color;
}

float LED::get_width() const {
    return w_;
}

float LED::get_height() const {
    return h_;
}

