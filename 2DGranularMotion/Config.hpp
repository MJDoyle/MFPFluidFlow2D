#ifndef CONFIG_HPP
#define CONFIG_HPP

#include <iostream>

#include <SFML/Network.hpp>
#include <SFML/Window.hpp>
#include <SFML/Graphics.hpp>
#include <SFML/Audio.hpp>

#include "Eigen/Dense"


#define PI 3.14159265359



extern const short SCREEN_WIDTH;

extern const short SCREEN_HEIGHT;

extern const float SCROLL_FACTOR;

extern const short FPS;


extern const short PUMP_SPRITE_OFFSET;

extern const short PUMP_SPRITE_OFFSET_2;

extern const short WATER_SPRITE_OFFSET;

extern const short MODULE_SIZE;


extern const short MODULE_MASS;

extern const short MAX_FLOW_RATE;

extern const float DT;

extern const float PUMP_THRUST;

extern const float DRAG_COEFFICIENT;

//Types of robots

extern const short RECTANGULAR;

extern const short RANDOM;


const enum {UP, RIGHT, DOWN, LEFT};

const enum {IN, OUT};

//Resistances

extern const float R;

extern const float S;

//Voltage

extern const float V;

//Colors

extern const int RED;

extern const int GREEN;

extern const int BLUE;

//Commands to faces

enum {TOGGLE_ACTIVATE, TOGGLE_BLOCK};

//Movement force

extern const float THRUST_COEFFICIENT;

#endif
