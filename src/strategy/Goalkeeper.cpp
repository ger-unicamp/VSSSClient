#include "Goalkeeper.h"

Goalkeeper::Goalkeeper(fira_message::Robot &robot): Player(robot) {}


/**
 * @brief defend our goal by following ball y projection using a vertical line univector field
 * 
 * @param ball_pos predicted position of ball on current frame
 * @return ctrl::vec2 
 */
ctrl::vec2 Goalkeeper::defend_goal_from(ctrl::vec2 ball_pos)
{
    ctrl::vec2 target, univec;
    double y_target, phi;

    // if ball is too close, goalkeeper can get out of y limit to reach ball
    if (ball_pos.x < Goalkeeper::X_LIMIT_BALL)
        y_target = ball_pos.y;
    else
        y_target = math::bound(ball_pos.y, -Goalkeeper::Y_LIMIT_GKP, Goalkeeper::Y_LIMIT_GKP);
    
    // if ball is behind goalkeeper x limit, he follows ball x position
    target = ctrl::vec2(std::min(Goalkeeper::X_LIMIT_GKP, ball_pos.x), y_target);
    phi = this->univec_vertical_line_field(target);

    sincos(phi, &univec.y, &univec.x);

    return univec;
}

/**
 * @brief returns goalkeeper final motors speed
 * 
 * @param ball 
 * @return ctrl::vec2 
 */
ctrl::vec2 Goalkeeper::play(fira_message::Ball &ball)
{
    ctrl::vec2 univec, motors_speed, ball_fut_pos;

    ball_fut_pos = this->future_position_of(ball, Goalkeeper::DT_GKP);

    if (this->get_pos().distance(ball_fut_pos) <= Goalkeeper::KICK_DIST)
    {
        univec = this->spin((this->robot.y() < ball_fut_pos.y));
    }
    else
    {
        univec = this->defend_goal_from(ball_fut_pos);
        motors_speed = this->move(univec);
    }

    return motors_speed;
}