#ifndef REFEREE_RESPONDER_HEADER
#define REFEREE_RESPONDER_HEADER

#include <array>

#include "pb/replacement.pb.h"
#include "pb/vssref_command.pb.h"
#include "pb/vssref_placement.pb.h"

#include "util/vec2.h"

typedef std::array<ctrl::vec2, 3> placement;

namespace plc
{
    const placement FB_Q23_B = {{{}, {}, {}}}; // Free Ball Quadrants 2, 3 - Blue answer
    const placement FB_Q14_B = {{{}, {}, {}}}; // Free Ball Quadrants 1, 4 - Blue answer
    const placement PK_B_B = {{{}, {}, {}}}; // Blue Penalty Kick - Blue answer
    const placement PK_Y_B = {{{}, {}, {}}}; // Yellow Penalty Kick - Blue answer
    const placement KO_B = {{{}, {}, {}}}; // Kick Off - Blue answer
    const placement GK_B_B = {{{}, {}, {}}}; // Blue Goal Kick - Blue answer
    const placement GK_Y_B = {{{}, {}, {}}}; // Yellow Goal Kick - Blue answer

    const placement FB_Q23_Y = {{{}, {}, {}}}; // Free Ball Quadrants 2, 3 - Yellow answer
    const placement FB_Q14_Y = {{{}, {}, {}}}; // Free Ball Quadrants 1, 4 - Yellow answer
    const placement PK_B_Y = {{{}, {}, {}}}; // Blue Penalty Kick - Yellow answer
    const placement PK_Y_Y = {{{}, {}, {}}}; // Yellow Penalty Kick - Yellow answer
    const placement KO_Y = {{{}, {}, {}}}; // Kick Off - Yellow answer
    const placement GK_B_Y = {{{}, {}, {}}}; // Blue Goal Kick - Yellow answer
    const placement GK_Y_Y = {{{}, {}, {}}}; // Yellow Goal Kick - Yellow answer
};

class RefereeResponder 
{
private:
    bool game_on;
    bool is_yellow;
    VSSRef::ref_to_team::VSSRef_Command ref_packet;
    VSSRef::team_to_ref::VSSRef_Placement cmd;
    VSSRef::Frame *replacement;

    placement answer_placement;

    void set_replacement();

    int check_foul();
    void answer_stop();
    void answer_free_ball();
    void answer_penalty_kick();
    void answer_kick_off();
    void answer_goal_kick();
    void set_answer_placement();

public:
    RefereeResponder(bool is_yellow);

    void set_ref_packet(VSSRef::ref_to_team::VSSRef_Command &ref_packet);
    bool get_game_on();
    VSSRef::team_to_ref::VSSRef_Placement answer();
};

#endif