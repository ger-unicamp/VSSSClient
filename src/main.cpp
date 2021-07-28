/**
 * @file main.cpp
 * @author Renato Sousa and GER Unicamp
 * @brief 
 * @version 0.1
 * @date 2020-07-09
 * 
 */

#include "strategy/Game.h"

int main(int argc, char *argv[])
{
    Game myGame(argc, argv);

    myGame.run();

    return 0;
}



