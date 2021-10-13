#include "Goalkeeper.h"

Goalkeeper::Goalkeeper(fira_message::Robot &robot)
    : Player(robot) 
{}

unsigned int Goalkeeper::lock_count = 0;

/**
 * @brief Get the slope of bisector line of the triangle formed between the ball and the goal
 * 
 * @param ball_pos 
 * @return double 
 */
double Goalkeeper::get_bisector_slope(ctrl::vec2 ball_pos)
{
    double y_half_size_goal = 0.2;
    double x_goal_position = -0.75;

    ctrl::vec2 top_goal_limit = ctrl::vec2( x_goal_position , y_half_size_goal);
    ctrl::vec2 bottom_goal_limit = ctrl::vec2( x_goal_position , -y_half_size_goal);

    double dist_ball_top = top_goal_limit.distance(ball_pos);
    double dist_ball_bottom = bottom_goal_limit.distance(ball_pos);

    double theta = acos( (pow(dist_ball_top , 2) + pow(dist_ball_bottom , 2) - pow(2*y_half_size_goal, 2))/(2 * dist_ball_top * dist_ball_bottom));
    double bisector = theta/2;

    double alpha, slope;
    if (ball_pos.y > 0)
    {
        alpha = asin(abs(ball_pos.y - y_half_size_goal)/dist_ball_top);
    }
    else
    {
        //y position is negative
        alpha = asin(abs(ball_pos.y + y_half_size_goal)/dist_ball_bottom);
    }

    //if vertical position of the ball is inside the goal limits
    if (ball_pos.y < y_half_size_goal && ball_pos.y > -y_half_size_goal)
    {   
        slope = tan(bisector - alpha);
    }
    else{

        slope = tan(bisector + alpha);
        
    }
    //if ball's vertical position is negative, slope must be negative too
    if (ball_pos.y < 0)
    {
        slope *= -1;
    }

    return slope;
}

/**
 * @brief Get the Y position of goalkeeper
 * 
 * @param ball_pos 
 * @param slope 
 * @return double 
 */
double Goalkeeper::get_Y_position(ctrl::vec2 ball_pos, double slope)
{
    //intersection between X_LIMIT_GPK and bisector line
    double linear_b = ball_pos.y - slope * ball_pos.x;
    
    double target_y = slope * Goalkeeper::X_LIMIT_GKP + linear_b;

    return target_y;
}



/**
 * @brief defend our goal by following ball y projection using a vertical sigmoid univector field
 * 
 * @param ball_pos predicted position of ball on current frame
 * @return ctrl::vec2 
 */
ctrl::vec2 Goalkeeper::defend_goal_from(ctrl::vec2 ball_pos)
{
    ctrl::vec2 target, univec;
    double y_target, phi, slope;

    //defense changes
    slope = this->get_bisector_slope(ball_pos);
    
    y_target = this->get_Y_position(ball_pos, slope);

    // The Goalkeeper has a Y limit
    y_target = math::bound(y_target, -Goalkeeper::Y_LIMIT_GKP, Goalkeeper::Y_LIMIT_GKP);

    // if ball is behind goalkeeper x limit, he follows ball x position
    target = ctrl::vec2(std::min(Goalkeeper::X_LIMIT_GKP, ball_pos.x), y_target);
    phi = this->univec_vertical_sigmoid_field(target);

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
    // handle locked robot
    if (is_locked(this->lock_count))
    {
        bool cw = this->robot.y() < ball.y();
        return spin(cw);
    }
    
    ctrl::vec2 univec, motors_speeds, ball_fut_pos;

    ball_fut_pos = this->future_position_relative_to(ball, Goalkeeper::DT_GKP);

    // if ball is close enought to the goalkeeper, we should spin in order to kick it 
    if (this->get_pos().distance(ball_fut_pos) <= Goalkeeper::KICK_DIST)
    {
        motors_speeds = this->spin((this->robot.y() < ball_fut_pos.y)); // kick ball
    }
    else
    {
        univec = this->defend_goal_from(ball_fut_pos);
        motors_speeds = this->move(univec);
    }

    return motors_speeds;
}