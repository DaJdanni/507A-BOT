#include "vex.h"
#include "cholib/tracking/coordinates.hpp"

cholib::Coordinates::Coordinates(float x, float y, float theta) {
    this->x = x;
    this->y = y;
    this->theta = theta;
}

cholib::Coordinates cholib::Coordinates::operator+(cholib::Coordinates& coordinates) {
    return cholib::Coordinates(this->x + coordinates.x, this->y + coordinates.y, this->theta);
}

cholib::Coordinates cholib::Coordinates::operator-(cholib::Coordinates& coordinates) {
    return cholib::Coordinates(this->x - coordinates.x, this->y - coordinates.y, this->theta);    
}

cholib::Coordinates cholib::Coordinates::operator*(float scale) {
    return cholib::Coordinates(this->x * scale, this->y * scale, this->theta);    
}

float cholib::Coordinates::operator*(cholib::Coordinates& coordinates) {
    return (this->x * coordinates.x) + (this->y * coordinates.y);
}

cholib::Coordinates cholib::Coordinates::operator/(cholib::Coordinates& coordinates) {
    return cholib::Coordinates(this->x / coordinates.x, this->y / coordinates.y, this->theta);    
}

bool cholib::Coordinates::operator<(cholib::Coordinates& coordinates) {
    if 
    (
        this->x < coordinates.x &&
        this->y < coordinates.y &&
        this->theta < coordinates.theta
    ) return true;
     else return false;
}

bool cholib::Coordinates::operator>(cholib::Coordinates& coordinates) {
    if 
    (
        this->x > coordinates.x &&
        this->y > coordinates.y &&
        this->theta > coordinates.theta
    ) return true;
     else return false;
}

bool cholib::Coordinates::operator<=(cholib::Coordinates& coordinates) {
    if 
    (
        this->x <= coordinates.x &&
        this->y <= coordinates.y &&
        this->theta <= coordinates.theta
    ) return true;
     else return false;
}

bool cholib::Coordinates::operator>=(cholib::Coordinates& coordinates) {
    if 
    (
        this->x >= coordinates.x &&
        this->y >= coordinates.y &&
        this->theta >= coordinates.theta
    ) return true;
     else return false;
}

bool cholib::Coordinates::operator<(float number) {
    if 
    (
        this->x < number &&
        this->y < number &&
        this->theta < number
    ) return true;
     else return false;
}

bool cholib::Coordinates::operator>(float number) {
    if 
    (
        this->x > number &&
        this->y > number &&
        this->theta > number
    ) return true;
     else return false;
}

bool cholib::Coordinates::operator<=(float number) {
    if 
    (
        this->x <= number &&
        this->y <= number &&
        this->theta <= number
    ) return true;
     else return false;
}

bool cholib::Coordinates::operator>=(float number) {
    if 
    (
        this->x >= number &&
        this->y >= number &&
        this->theta >= number
    ) return true;
     else return false;
}

cholib::Coordinates cholib::Coordinates::lerp(cholib::Coordinates coordinates, float t) {
    return cholib::Coordinates(this->x + (coordinates.x - this->x) * t, this->y + (coordinates.y - this->y) * t, this->theta);
}

float cholib::Coordinates::distance(cholib::Coordinates coordinates) {
   // printf("xdif: %f, ydif: %f \n", this->x - coordinates.x, this->y - coordinates.y);
    return std::hypotf(this->x - coordinates.x, this->y - coordinates.y);
}

float cholib::Coordinates::angle(cholib::Coordinates coordinates) {
    return std::atan2(coordinates.x - this->x, coordinates.y - this->y);
    // this returns angles with 0 degrees starting on the positive y axis
}