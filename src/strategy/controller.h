#ifndef VSSS
#define VSSS

#include "pb/command.pb.h"
#include "pb/common.pb.h"
#include "pb/packet.pb.h"
#include "pb/replacement.pb.h"

#include "util/mathutil.h"
#include <cmath>

namespace ctrl
{
    /**
     * @brief Defines a 2D point with double precision
     * 
     */
    struct vec2
    {
        double x;
        double y;

        /**
         * @brief Construct a new vec2 object
         * 
         * @param _x x coordinate
         * @param _y y coordinate
         */
        vec2(double _x, double _y) : x(_x), y(_y) {}
        /**
         * @param r x and y coordinates
         */
        vec2(double r) : x(r), y(r) {}
        /**
         * @brief Construct a new vec2 object at (0.0,0.0)
         * 
         */
        vec2() : x(0.0), y(0.0) {}
        /**
         * @param b fira_message::Ball position x,y
         */
        vec2(const fira_message::Ball &b) : x(b.x()), y(b.y()) {}
        /**
         * @param r fira_message::Robot position x,y
         */
        vec2(const fira_message::Robot &r) : x(r.x()), y(r.y()) {}

        vec2 operator+(const vec2 &p) const { return vec2(x + p.x, y + p.y); }
        vec2 &operator+=(const vec2 &p)
        {
            x += p.x;
            y += p.y;
            return *this;
        }
        vec2 operator-() const { return vec2(-x, -y); }
        vec2 operator-(const vec2 &p) const { return vec2(x - p.x, y - p.y); }
        vec2 &operator-=(const vec2 &p)
        {
            x -= p.x;
            y -= p.y;
            return *this;
        }
        /**
         * @brief Scalar product double * vec2
         * 
         * @param d 
         * @return vec2 
         */
        vec2 operator*(const double d) const { return vec2(d * x, d * y); }
        /**
         * @brief Commutative scalar product 
         * @return vec2 
         */
        friend vec2 operator*(const double d, const vec2 &p) { return vec2(d * p.x, d * p.y); }
        vec2 &operator*=(const double d)
        {
            x *= d;
            y *= d;
            return *this;
        }

        /**
         * @brief Defines dot vector product
         * 
         * @param p ctrl::vec2
         * @return double x * p.x + y * p.y
         */
        double operator*(const vec2 &p) const { return (x * p.x + y * p.y); }

        bool operator==(const vec2 &p) const { return (x == p.x && y == p.y); }
        /**
         * @brief Access x,y coordinates by axis (v[0] == v.x && v[1] == v.y)
         * 
         * @param i 
         * @return double 
         */
        double operator[](const unsigned int i)
        {
            if (i % 2 == 0)
                return x;
            return y;
        }
        /**
         * @brief Returns Euclidian norm of vector
         * 
         * @return double 
         */
        double abs() const { return std::sqrt(x * x + y * y); }
        /**
         * @brief Returns [-pi, pi] angle with x-axis
         * 
         * @return double 
         */
        double theta() const { return std::atan2(y, x); }
        /**
         * @brief Returns z-coordinate of cross product
         * 
         * @param p ctrl::vec2
         * @return double x * p.y - y * p.x
         */
        double cross(const vec2 &p) const { return (x * p.y - y * p.x); }

        /**
         * @brief Returns the normalized vector (safe to x,y range)
         * 
         * @return vec2 
         */
        vec2 normalized() const
        {
            double s, c;
            sincos(this->theta(), &s, &c);
            return vec2(c, s);
        }

        double distance(vec2 &p) { return ((*this - p).abs()); }
        
        double distance(const vec2 &p) { return ((*this - p).abs()); }
    };

    ctrl::vec2 move_robot(fira_message::Robot &robot, ctrl::vec2 vector, double k, double v);
    ctrl::vec2 future_position(fira_message::Robot &r1, fira_message::Robot &r2, double dt);
    ctrl::vec2 future_position(fira_message::Ball &b, fira_message::Robot &r, double dt);

}; // namespace ctrl

#endif