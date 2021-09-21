#include <iostream> 
#include "strategy/RefereeResponder.h"
#include "pb/vssref_common.pb.h"

RefereeResponder::RefereeResponder(bool is_yellow) 
    : is_yellow(is_yellow)
{
    game_on = false;
}

void RefereeResponder::set_replacement() 
{
    this->replacement = new VSSRef::Frame();

    replacement->set_teamcolor(is_yellow ? VSSRef::YELLOW : VSSRef::BLUE);
    
    auto robot0 = replacement->add_robots();
    auto robot1 = replacement->add_robots();
    auto robot2 = replacement->add_robots();

    robot0->set_robot_id(0);
    robot1->set_robot_id(1);
    robot2->set_robot_id(2);

    robot0->set_x(answer_placement[0].x);
    robot0->set_y(answer_placement[0].y);
    robot0->set_orientation(90); // think how to better set robot orientations
    robot1->set_x(answer_placement[1].x);
    robot1->set_y(answer_placement[1].y);
    robot1->set_orientation(0);
    robot2->set_x(answer_placement[2].x);
    robot2->set_y(answer_placement[2].y);
    robot2->set_orientation(0);
}

int RefereeResponder::check_foul() 
{
    unsigned int foul = this->ref_packet.foul();
    std::cout << "-----Referee Foul: " << foul << std::endl;

    return foul;
}

void RefereeResponder::answer_free_ball()
{
    if (ref_packet.foulquadrant() == VSSRef::QUADRANT_2 || ref_packet.foulquadrant() == VSSRef::QUADRANT_3)
    {
        answer_placement = is_yellow ? plc::FB_Q23_Y : plc::FB_Q23_B;
    }

    else if (ref_packet.foulquadrant() == VSSRef::QUADRANT_1 || ref_packet.foulquadrant() == VSSRef::QUADRANT_4)
    {
        answer_placement = is_yellow ? plc::FB_Q14_Y : plc::FB_Q14_B;
    }
}

void RefereeResponder::answer_penalty_kick()
{
    if (ref_packet.teamcolor() == VSSRef::BLUE)
    {
        answer_placement = is_yellow ? plc::PK_B_Y : plc::PK_B_B;
    }

    else if (ref_packet.teamcolor() == VSSRef::YELLOW)
    {
        answer_placement = is_yellow ? plc::PK_Y_Y : plc::PK_Y_B;
    }
}

void RefereeResponder::answer_kick_off()
{
    answer_placement = is_yellow ? plc::KO_Y : plc::KO_B;
}

void RefereeResponder::answer_goal_kick()
{
    if (ref_packet.teamcolor() == VSSRef::BLUE)
    {
        answer_placement = is_yellow ? plc::GK_B_Y : plc::GK_B_B;
    }

    else if (ref_packet.teamcolor() == VSSRef::YELLOW)
    {
        answer_placement = is_yellow ? plc::GK_Y_Y : plc::GK_Y_B;
    }   
}

/**
 * @brief get answer placement for the robots
 * 
 * @return 
 */
void RefereeResponder::set_answer_placement()
{
    switch (check_foul())
    {
    case VSSRef::GAME_ON:
        game_on = true;
        break;

    case VSSRef::STOP:
        game_on = false;
        break;

    case VSSRef::FREE_BALL: 
        answer_free_ball();
        break;
    
    case VSSRef::PENALTY_KICK:
        answer_penalty_kick();
        break;

    case VSSRef::KICKOFF:
        answer_kick_off();
        break;

    case VSSRef::GOAL_KICK:
        answer_goal_kick();
        break;         
    }
}


// ---------> PUBLIC METHODS <----------

void RefereeResponder::set_ref_packet(VSSRef::ref_to_team::VSSRef_Command &ref_pack)
{
    this->ref_packet = ref_pack;
}

bool RefereeResponder::get_game_on()
{
    return game_on;
}

VSSRef::team_to_ref::VSSRef_Placement RefereeResponder::answer()
{
    set_answer_placement();
    set_replacement();
    cmd.set_allocated_world(replacement);
    
    return cmd;
}