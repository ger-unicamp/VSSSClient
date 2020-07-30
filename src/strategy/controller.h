#ifndef VSSS
#define VSSS

#include "pb/command.pb.h"
#include "pb/common.pb.h"
#include "pb/packet.pb.h"
#include "pb/replacement.pb.h"

#include <cmath>

#define PI (3.14159265359)

namespace ctrl
{
    /**
     * @brief Returns the signal of number 
     * 
     * @tparam T could be a long, a short, a double, a float, ...
     * @param val 
     * @return int -1, 0 or 1
     */
    template <typename T>
    int sgn(T val)
    {
        return (T(0) < val) - (val < T(0));
    }

    /**
     * @brief Defines a 2D point
     * 
     */
    struct vec2
    {
        double x;
        double y;

        vec2(double _x, double _y) : x(_x), y(_y) {}
        vec2(double r): x(r), y(r) {}
        vec2(const fira_message::Ball &b) : x(b.x()), y(b.y()) {}
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

        vec2 operator*(const double d) const { return vec2(d * x, d * y); }
        // Scalar product is commutative
        friend vec2 operator*(const double d, const vec2 &p) { return vec2(d * p.x, d * p.y); }
        vec2 &operator*=(const double d)
        {
            x *= d;
            y *= d;
            return *this;
        }

        // Defines a dot product
        double operator*(const vec2 &p) const { return (x * p.x + y * p.y); }

        bool operator==(const vec2 &p) const { return (x == p.x && y == p.y); }

        double operator[](const unsigned int i)
        {
            if (i % 2 == 0)
                return x;
            return y;
        }

        double abs() const { return std::sqrt(x * x + y * y); }
        double theta() const { return std::atan2(y, x); }
        // z-coordinate of cross product
        double cross(const vec2 &p) const { return (x * p.y - y * p.x); }
    };

    vec2 get_speed_to(const fira_message::Robot &robot, const vec2 &ball);

}; // namespace ctrl

#endif