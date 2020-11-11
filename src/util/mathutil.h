#ifndef _MATH_UTIL
#define _MATH_UTIL

#include <cmath>

#define PI (3.14159265358979323846)

#define TWO_PI (6.28318530717958647692)

#define HALF_PI (1.57079632679489661923)

namespace math
{
    /** USEFUL MATH FUNCTIONS **/

    /**
     * @brief Returns the signal of number 
     * 
     * @tparam T could be a long, a short, a double, a float, ...
     * @param val 
     * @return int -1, 0 or 1
     */
    template <typename T>
    inline int sgn(T val)
    {
        return (T(0) < val) - (val < T(0));
    }

    /**
     * @brief Wrap a radian angle in [-PI, PI]
     * 
     * @param t 
     * @return real 
     */
    inline double wrap_to_pi(double t)
    {
        t = fmod(t + PI, TWO_PI);
        t += (t < 0) ? TWO_PI : 0.0;
        return t - PI;
    }

    /**
     * @brief Returns exp(-r^2/(2*s^2))
     * 
     * @param r variable event (zero padded)
     * @param s standard deviation, sigma
     * @return double 
     */
    inline double gaussian(double r, double s)
    {
        return std::exp((-0.5 * r * r) / (s * s));
    }

    /**
     * @brief Returns gaussian probability density function
     * 
     * @param r 
     * @param s 
     * @return double 
     */
    inline double normal_gaussian(double r, double s)
    {
        return (1.0 / (s * std::sqrt(TWO_PI))) * gaussian(r, s);
    }

    /**
     * @brief Limit a number to a interval [min, max]
     * 
     * @param x 
     * @param min 
     * @param max 
     * @return double 
     */
    inline double bound(double x, double min, double max)
    {
        if(x > max)
            return max;
        if(x < min)
            return min;
        return x;
    }

}; // namespace math

#endif