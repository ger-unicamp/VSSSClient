#include "vss_client.h"
#include "util/vec2.h"

class MeuRobo {
private:
    fira_message::Robot robo;

public:
    MeuRobo(fira_message::Robot robo);
    void move(ctrl::vec2 vec);
};



MeuRobo::MeuRobo(fira_message::Robot robo)
{
    this->robo = robo;
}

void MeuRobo::move(ctrl::vec2 vec)
{
    ;
}

MeuRobo jorge(fira_message::Robot());
MeuRobo irene(fira_message::Robot());