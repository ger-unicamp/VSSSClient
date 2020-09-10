#include "strategy/controller.h"
#include <iostream>

ctrl::vec2 ctrl::get_speed_to(const fira_message::Robot &robot, const ctrl::vec2 &ball)
{
    double kp = 10.0, kt = 50.0, kv = 30.0;
    const double eps = 0.1;

    vec2 set_point = ball - vec2(robot);
    double set_angle = set_point.theta() - robot.orientation();
    if (set_angle < PI / 2.0 + eps && set_angle > PI / 2.0 - eps)
    {
        set_angle = PI / 2.0;
        kv = 0.0;
    }
    else if (set_angle < -PI / 2.0 + eps && set_angle > -PI / 2.0 - eps)
    {
        set_angle = -PI / 2.0;
        kv = 0.0;
    }
    else if (set_angle > PI / 2.0)
    {
        set_angle -= PI;
        kv *= -1.0;
    }
    else if (set_angle < -PI / 2.0)
    {
        set_angle += PI;
        kv *= -1.0;
    }
    std::cout << set_angle << std::endl;
    return kt * vec2(-set_angle, set_angle) + kv * vec2(1);
}