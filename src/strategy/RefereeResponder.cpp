#include "strategy/RefereeResponder.h"

#include "pb/vssref_command.pb.h"
#include "pb/vssref_common.pb.h"
#include "pb/vssref_placement.pb.h"

RefereeResponder::RefereeResponder(bool is_yellow, VSSRef::ref_to_team::VSSRef_Command &ref_packet) 
    : is_yellow(is_yellow)
    , ref_packet(ref_packet) 
{}

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
        this->placement = this->answer
    }

    else if (ref_packet.foulquadrant() == VSSRef::QUADRANT_1 || ref_packet.foulquadrant() == VSSRef::QUADRANT_4)
    {
        if (!yellow) positions = {{-0.7, 0.0}, {0.175, 0.4}, {0.175, -0.4}};
        else positions = {{0.7, 0.0}, {0.55, -0.4}, {0.55, 0.4}};
    }

    set_replacement_robots(robot0, robot1, robot2, positions);
}

/**
 * @brief answer referee signals, always considering blue team
 * 
 * @return VSSRef::team_to_ref::VSSRef_Placement 
 */
VSSRef::team_to_ref::VSSRef_Placement RefereeResponder::answer()
{
    switch (check_foul())
    {
    case VSSRef::GAME_ON:
        break;

    case VSSRef::STOP:
        break;

    case VSSRef::FREE_BALL: 
        answer_free_ball();
        break;
    
    case VSSRef::PENALTY_KICK:
        

    case VSSRef::KICKOFF:
        if (!yellow) positions = {{-0.7, 0.0}, {-0.25, 0.0}, {-0.4, -0.2}};
        else positions = {{0.7, 0.0}, {0.25, 0.0}, {0.4, 0.2}};
        replace_robots(robot0, robot1, robot2, positions);
        
        break;

    case VSSRef::GOAL_KICK:
        if (ref_packet.teamcolor() == VSSRef::BLUE)
        {   
            if (!yellow)
            {
                positions = {{-0.65, -0.3}, {-0.4, -0.25}, {-0.5, -0.55}};
                replace_robots(robot0, robot1, robot2, positions);
                robot0->set_orientation(135);
            }
            else
            {
                positions = {{0.7, 0.0}, {-0.25, -0.2}, {-0.25, 0.2}};
                replace_robots(robot0, robot1, robot2, positions);
            }   
        }

        else if (ref_packet.teamcolor() == VSSRef::YELLOW)
        {
            if (!yellow)
            {
                positions = {{-0.7, 0.0}, {0.25, 0.2}, {0.25, -0.2}};
                replace_robots(robot0, robot1, robot2, positions);
            }
            else
            {
                positions = {{0.7, 0.25}, {0.4, 0.25}, {0.5, 0.55}};
                replace_robots(robot0, robot1, robot2, positions);
                robot0->set_orientation(135);
            }
        }
        break;         
    }
}