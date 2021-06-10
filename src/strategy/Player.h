#ifndef PLAYER_HEADER
#define PLAYER_HEADER

#include "vss_client.h"
#include "util/vec2.h"

class Player
{
private:
    static constexpr double K_TURNING = 0.443467;
    static constexpr double K_VEL = 39.9596;
    static constexpr double DT = 0.147302; 
    static constexpr double SPIN_SPEED = 150;

protected:
    // protected constructor
    Player(fira_message::Robot &robot);

    fira_message::Robot robot;
    ctrl::vec2 future_position_of(fira_message::Robot &r, double dt);
    ctrl::vec2 future_position_of(fira_message::Ball &b, double dt);

public:
    ctrl::vec2 move( ctrl::vec2 vector);
    ctrl::vec2 spin(bool cw);
    bool is_locked(unsigned int &stopped_count);

    // virtual function that returns final motors speed for sendind to simulator
    virtual ctrl::vec2 play();
};

#endif