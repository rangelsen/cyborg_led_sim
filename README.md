# LED Simulator 

ROS node for displaying neuron activity

## Dependencies

### OpenGL 3.0 (Open Source Graphics Library)

Cross platform graphics driver API for drawing stuff. Should be shipped with Ubuntu.
If not, check out [this](https://en.wikibooks.org/wiki/OpenGL_Programming/Installation/Linux)

#### GLEW (OpenGL Extention Wrangler)

Used for OpenGL function loading. Should also be shipped with Ubuntu but it
can easily be obtained from

```
sudo apt-get install libglew-dev
```

### GLM (OpenGL Mathematics) 

Header only C++ math library. Can be obtained through

```
sudo apt-get install libglm-dev
```

### SDL2 (Simple Direct Media Layer)

Used for OpenGL context handling

```
sudo apt-get install libsdl2-dev
```

### OpenCV 2.4.9 (Open Source Computer Vision)

Used to tranform from the HSV color space to RGBA. HSV is used in the LED drivers and RGBA is
used by OpenGL. This should also be shipped with Ubuntu.
