#pragma once
#include <stdio.h>
#include <stdlib.h>
#include <math.h> 

template<typename T> class Vector2 {
public:
    T x, y;
    Vector2() : x(0), y(0) { }
    template<typename U> Vector2(U x, U y) : x(T(x)), y(T(y)) { }
    template<typename U> Vector2(Vector2<U>& other) { this->x = T(other.x); this->y = T(other.y); }
    T dot(const Vector2<T>& other) const { return this->x*other.x + this->y*other.y; }
    void normalize() { T abs = this->abs(); this->x /= abs; this->y /= abs; }
    double abs() const { return sqrt(double(this->x*this->x + this->y*this->y)); }
    void rotate(double theta);
    Vector2<T> operator+(const Vector2<T>& other) const { return Vector2<T>(this->x + other.x, this->y + other.y); }
    void operator+=(const Vector2<T>& other) { this->x += other.x; this->y += other.y; }
    Vector2<T> operator-(const Vector2<T>& other) const { return Vector2(this->x - other.x, this->y - other.y); }
    void operator-=(const Vector2<T>& other) { this->x -= other.x; this->y -= other.y; }
    Vector2<T> operator/(const T scalar) const { return Vector2<T>(this->x/scalar, this->y/scalar); }
    void operator/=(const T scalar) { this->x /= scalar; this->y /= scalar; }
    Vector2<T> operator*(const T scalar) const { return Vector2<T>(this->x*scalar, this->y*scalar); }
};

template<typename T> void Vector2<T>::rotate(double theta) {
    double ct = cos(theta);
    double st = sin(theta);
    T x = this->x;
    T y = this->y;
    this->x = x * ct - y * st;
    this->y = y * ct + x * st;
}

template<typename T> class Vector3 {
public:
    T x, y, z;

    Vector3() : x(0), y(0), z(0) { ; }
    Vector3(T x, T y, T z) : x(x), y(y), z(z) { ; }
    template<typename U> Vector3(Vector3<U>& other) { this->x = T(other.x); this->y = T(other.y); this->z = T(other.z); }

    T dot(const Vector3<T>& other) const { return this->x*other.x + this->y*other.y + this->z*other.z; }
    void normalize() { T abs = this->abs(); this->x /= abs; this->y /= abs; this->z /= abs; }
    T abs() const { return sqrt(this->x*this->x + this->y*this->y + this->z*this->z); }
    void rotate(double theta);
    Vector3<T> rotate(Vector3<T>& other, double theta) const;
    Vector3<T> operator+(const Vector3<T>& other) const { return Vector3<T>(this->x + other.x, this->y + other.y, this->z + other.z); }
    Vector3<T> operator-(const Vector3<T>& other) const { return Vector3(this->x - other.x, this->y - other.y, this->z - other.z); }
    Vector3<T> operator/(const T scalar) const { return Vector3<T>(this->x/scalar, this->y/scalar, this->z/scalar); }
    Vector3<T> operator*(const T scalar) const { return Vector3<T>(this->x*scalar, this->y*scalar, this->z*scalar); }
    operator Vector3<double>() const { return Vector3<double>(this->x, this->y, this->z); }
    operator Vector3<int>() const { return Vector3<int>(this->x, this->y, this->z); }
    operator Vector2<T>() const { return Vector2<T>(this->x, this->y); }
    void operator+=(const Vector3<T>& other) { this->x += other.x; this->y += other.y; this->z += other.z; }
    void operator-=(const Vector3<T>& other) { this->x -= other.x; this->y -= other.y; this->z -= other.z; }
    void operator/=(const T scalar) { this->x /= scalar; this->y /= scalar; this->z /= scalar; }
    /*Vector3<T>& operator=(const Vector3<T>& other) { 
      if (this != &other)
        this->x = T(other).x; this->y = T(other).y;  this->z = T(other).z; 
      return *this;
    }*/
};

template<typename T> Vector3<T> Vector3<T>::rotate(Vector3<T>& other, double theta) const {
    // The following code has been adapted from rotate_around method of the python library euclid
    // Return the vector rotated around axis through angle theta. Right hand rule applies

    // Adapted from equations published by Glenn Murray.
    // http://inside.mines.edu/~gmurray/ArbitraryAxisRotation/ArbitraryAxisRotation.html

    // Extracted common factors for simplicity and efficiency
    double r2 = other.x*other.x + other.y*other.y + other.z*other.z;
    double r = sqrt(r2);
    double ct = cos(theta);
    double st = sin(theta) / r;
    double dt = (other.x*this->x + other.y*this->y + other.z*this->z) * (1 - ct) / r2;
    return Vector3<T>((other.x * dt + this->x * ct + (-other.z * this->y + other.y * this->z) * st),
            (other.y * dt + this->y * ct + ( other.z * this->x - other.x * this->z) * st),
            (other.z * dt + this->z * ct + (-other.y * this->x + other.x * this->y) * st));
}

template<typename T> void Vector3<T>::rotate(double theta) {
    double ct = cos(theta);
    double st = sin(theta);
    T x = this->x;
    T y = this->y;
    this->x = x * ct - y * st;
    this->y = y * ct + x * st;
}
