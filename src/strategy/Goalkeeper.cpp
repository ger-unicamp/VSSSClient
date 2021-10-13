#include "Goalkeeper.h"
#include "Game.h"

Goalkeeper::Goalkeeper(fira_message::Robot &robot)
    : Player(robot) 
{}

unsigned int Goalkeeper::lock_count = 0;

/**
 * @brief returns goalkeeper final motors speed
 * 
 * @param ball 
 * @return ctrl::vec2 
 */
ctrl::vec2 Goalkeeper::play(fira_message::Ball &ball)
{
    ctrl::vec2 target, ball_fut_pos, motors_speeds, univec;
    double y_target, phi;

    // handle locked robot
    if (is_locked(this->lock_count))
    {
        bool cw = this->robot.y() < ball.y();
        return spin(cw);
    }

    ball_fut_pos = this->future_position_relative_to(ball, Goalkeeper::DT_GKP);

    // if ball is close enought to the goalkeeper, we should spin in order to kick it 
    if (this->get_pos().distance(ball_fut_pos) <= Goalkeeper::KICK_DIST)
    {
        motors_speeds = this->spin((this->robot.y() < ball_fut_pos.y)); // kick ball
    }
    else
    {
        // The Goalkeeper has a Y limit
        y_target = math::bound(ball_fut_pos.y, -Goalkeeper::Y_LIMIT_GKP, Goalkeeper::Y_LIMIT_GKP);

        target = ctrl::vec2(Goalkeeper::GKP_X_AXIS, y_target);


        //If the keeper is out of its lane, it reajusts itself
        double tolerance = 0.1; //the keeper's lane has a margin of tolerance

        if(this->robot.x() <= Goalkeeper::GKP_X_AXIS - tolerance){
            phi = ((this->robot.y() < target.y ? 70 : 110)* PI)/180;
        }
        else if(this->robot.x() > Goalkeeper::GKP_X_AXIS + tolerance){
            phi = ((this->robot.y() < target.y ? -70 : -110)* PI)/180;
        }
        else{
            phi = ((this->robot.y() < target.y ? 90 : -90)* PI)/180;
        }

        sincos(phi, &univec.y, &univec.x);

        motors_speeds = this->move(univec);
    }

    return motors_speeds;
}