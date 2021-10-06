#ifndef REFEREE_RESPONDER_HEADER
#define REFEREE_RESPONDER_HEADER

#include <array>
#include <map>

#include "pb/replacement.pb.h"
#include "pb/vssref_command.pb.h"
#include "pb/vssref_placement.pb.h"

#include "util/vec2.h"

struct placement {
    double x;
    double y;
    double angle;
};


typedef std::array<placement, 3> placements;

namespace plc
{
    const placements FB_Q1_B = {{{-0.4, 0.3, 90}, {-0.4, 0.0, 0.0}, {-0.4, -0.3, 0.0}}}; // Free Ball Quadrant 1 - Blue answer
    const placements FB_Q2_B = {{{-0.4, 0.3, 0.0}, {-0.4, 0.0, 0.0}, {-0.4, -0.3, 0.0}}}; // Free Ball Quadrant 2 - Blue answer
    const placements FB_Q3_B = {{{-0.4, 0.3, 0.0}, {-0.4, 0.0, 0.0}, {-0.4, -0.3, 0.0}}}; // Free Ball Quadrant 3 - Blue answer
    const placements FB_Q4_B = {{{-0.4, 0.3, 0.0}, {-0.4, 0.0, 0.0}, {-0.4, -0.3, 0.0}}}; // Free Ball Quadrant 4 - Blue answer
    const placements PK_B_B = {{{-0.4, 0.3, 0.0}, {-0.4, 0.0, 0.0}, {-0.4, -0.3, 0.0}}}; // Blue Penalty Kick - Blue answer
    const placements PK_Y_B = {{{-0.4, 0.3, 0.0}, {-0.4, 0.0, 0.0}, {-0.4, -0.3, 0.0}}}; // Yellow Penalty Kick - Blue answer
    const placements KO_Y_B = {{{-0.4, 0.3, 0.0}, {-0.4, 0.0, 0.0}, {-0.4, -0.3, 0.0}}}; // Kick Off Yellow - Blue answer
    const placements KO_B_B = {{{-0.4, 0.3, 0.0}, {-0.4, 0.0, 0.0}, {-0.4, -0.3, 0.0}}}; // Kick Off Blue - Blue answer
    const placements GK_B_B = {{{-0.4, 0.3, 0.0}, {-0.4, 0.0, 0.0}, {-0.4, -0.3, 0.0}}}; // Blue Goal Kick - Blue answer
    const placements GK_Y_B = {{{-0.4, 0.3, 0.0}, {-0.4, 0.0, 0.0}, {-0.4, -0.3, 0.0}}}; // Yellow Goal Kick - Blue answer

    const placements FB_Q1_Y = {{{-0.4, 0.3, 0.0}, {-0.4, 0.0, 0.0}, {-0.4, -0.3, 0.0}}}; // Free Ball Quadrant 1 - Yellow answer
    const placements FB_Q2_Y = {{{-0.4, 0.3, 0.0}, {-0.4, 0.0, 0.0}, {-0.4, -0.3, 0.0}}}; // Free Ball Quadrant 2 - Yellow answer
    const placements FB_Q3_Y = {{{-0.4, 0.3, 0.0}, {-0.4, 0.0, 0.0}, {-0.4, -0.3, 0.0}}}; // Free Ball Quadrant 3 - Yellow answer
    const placements FB_Q4_Y = {{{-0.4, 0.3, 0.0}, {-0.4, 0.0, 0.0}, {-0.4, -0.3, 0.0}}}; // Free Ball Quadrant 4 - Yellow answer
    const placements PK_B_Y = {{{-0.4, 0.3, 0.0}, {-0.4, 0.0, 0.0}, {-0.4, -0.3, 0.0}}}; // Blue Penalty Kick - Yellow answer
    const placements PK_Y_Y = {{{-0.4, 0.3, 0.0}, {-0.4, 0.0, 0.0}, {-0.4, -0.3, 0.0}}}; // Yellow Penalty Kick - Yellow answer
    const placements KO_Y_Y = {{{-0.4, 0.3, 0.0}, {-0.4, 0.0, 0.0}, {-0.4, -0.3, 0.0}}}; // Kick Off Yellow - Yellow answer
    const placements KO_B_Y = {{{-0.4, 0.3, 0.0}, {-0.4, 0.0, 0.0}, {-0.4, -0.3, 0.0}}}; // Kick Off Blue - Yellow answer
    const placements GK_B_Y = {{{-0.4, 0.3, 0.0}, {-0.4, 0.0, 0.0}, {-0.4, -0.3, 0.0}}}; // Blue Goal Kick - Yellow answer
    const placements GK_Y_Y = {{{-0.4, 0.3, 0.0}, {-0.4, 0.0, 0.0}, {-0.4, -0.3, 0.0}}}; // Yellow Goal Kick - Yellow answer
};

class RefereeResponder 
{
private:
    bool game_on;
    bool is_yellow;
    VSSRef::ref_to_team::VSSRef_Command ref_packet;
    VSSRef::Frame *replacement;

    placements answer_placement;

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