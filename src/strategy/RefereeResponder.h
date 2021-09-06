#ifndef REFEREE_RESPONDER_HEADER
#define REFEREE_RESPONDER_HEADER

#include <array>

#include "pb/replacement.pb.h"

#include "util/vec2.h"

typedef std::array<ctrl::vec2, 3> placement;

namespace placements
{
    constexpr placement FB_Q23_B = {{}, {}, {}}; // Free Ball Quadrants 2, 3 - Blue answer
    constexpr placement FB_Q14_B = {{}, {}, {}}; // Free Ball Quadrants 1, 4 - Blue answer
    constexpr placement PK_B_B = {{}, {}, {}}; // Blue Penalty Kick - Blue answer
    constexpr placement PK_Y_B = {{}, {}, {}}; // Yellow Penalty Kick - Blue answer
    constexpr placement KO_B = {{}, {}, {}}; // Kick Off - Blue answer
    constexpr placement GK_B_B = {{}, {}, {}}; // Blue Goal Kick - Blue answer
    constexpr placement GK_Y_B = {{}, {}, {}}; // Yellow Goal Kick - Blue answer

    constexpr placement FB_Q23_Y = {{}, {}, {}}; // Free Ball Quadrants 2, 3 - Yellow answer
    constexpr placement FB_Q14_Y = {{}, {}, {}}; // Free Ball Quadrants 1, 4 - Yellow answer
    constexpr placement PK_B_Y = {{}, {}, {}}; // Blue Penalty Kick - Yellow answer
    constexpr placement PK_Y_Y = {{}, {}, {}}; // Yellow Penalty Kick - Yellow answer
    constexpr placement KO_Y = {{}, {}, {}}; // Kick Off - Yellow answer
    constexpr placement GK_B_Y = {{}, {}, {}}; // Blue Goal Kick - Yellow answer
    constexpr placement GK_Y_Y = {{}, {}, {}}; // Yellow Goal Kick - Yellow answer
};

class RefereeResponder 
{
private:
    bool is_yellow;
    VSSRef::ref_to_team::VSSRef_Command ref_packet;
    VSSRef::team_to_ref::VSSRef_Placement cmd;
    VSSRef::Frame *replacement;

    void set_replacement();

    int check_foul();
    void answer_stop();

    void answer_free_ball();
    void answer_penalty_kick();
    void answer_kick_off();
    void answer_goal_kick();

    void send_command(VSSRef::team_to_ref::VSSRef_Placement cmd);

public:
    RefereeResponder(bool is_yellow, VSSRef::ref_to_team::VSSRef_Command &ref_packet);
    VSSRef::team_to_ref::VSSRef_Placement answer();
};

#endif