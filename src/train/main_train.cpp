#include <iostream>
#include <thread>

#include "net/robocup_ssl_client.h"
#include "net/vss_client.h"
#include "strategy/APF.h"
#include "strategy/controller.h"
#include "train/genetics.h"
#include "train/vss_replacer.h"
#include "util/timer.h"
#include "util/mathutil.h"

#include "pb/command.pb.h"
#include "pb/common.pb.h"
#include "pb/packet.pb.h"
#include "pb/replacement.pb.h"

extern Random randr;
const size_t MAX_GEN = 200;
const size_t N_POP = 20;
const size_t N_GENES = 4;
const double min_value = 0.0;
const double max_value = 1.0;
const double mr = 0.15; 

struct measure
{
    double time;
    double x_error;
    double y_error;
    double t_error;
    double vy_error;

    /**
     * @brief Returns evaluation function (Kim et al.)
     * 
     * @return double 
     */
    double operator()()
    {
        double tmp = this->time;
        if (tmp < 1.5)
        {
            tmp /= 10.0;
        }
        double eval = 10.0 * tmp * tmp;
        eval += 10.0 * this->t_error * this->t_error;
        eval += 2.0 * this->x_error * this->x_error;
        eval += 2.0 * this->y_error * this->y_error;
        eval += 5.0 * this->vy_error * this->vy_error;
        return eval;
    }
};

std::function<void(ctrl::vec2 &start, DNA &d, measure *res)> simulate;
double fitness_simulate(DNA &d);
double test_fitness(DNA &d);

int main()
{
    Population pop(N_POP, N_GENES, min_value, max_value, mr, MAX_GEN);
    Timer time;
    RoboCupSSLClient client(10002, "224.0.0.1");
    VSSClient sim_client(20011, "127.0.0.1");
    VSSReplacer replacer(20011, "127.0.0.1");

    fira_message::sim_to_ref::Environment packet;
    client.open(false);

    /**
     * @brief Define a simulation (lambda) function
     * This function should call apf field and
     * move robot, and check with the vision if
     * robot reached the target.
     * 
     */
    simulate = [&](ctrl::vec2 &start, DNA &d, measure *res) {
        while (true)
        {
            if (client.receive(packet))
            {
                replacer.setBallPos(0.0, 0.0);
                replacer.setRobotPos(false, 0, start.x, start.y, 0.0);
                break;
            }
        }
        // Limit simulation execution to 5 seconds
        res->time = 6;
        for (long start = time(); time() - start < 3000;)
        {
            if (client.receive(packet))
            {
                auto ball = packet.frame().ball();
                auto robot = packet.frame().robots_blue(0);
                
                // Update measure
                double tmp = robot.orientation();
                if (tmp > HALF_PI)
                {
                    tmp = math::wrap_to_pi(tmp - PI);
                }
                else if (tmp < -HALF_PI)
                {
                    tmp = math::wrap_to_pi(tmp + PI);
                }
                
                res->t_error = tmp;
                res->y_error = robot.y();
                res->x_error = robot.x() + 0.059;
                res->vy_error = ball.vy();

                /**
                 * @brief This comparisson may change
                 * 
                 */
                if ((ctrl::vec2(ball) - robot).abs() <= 0.075 && robot.x() - ball.x() < -0.0 && ball.vx() > 0.0)
                {
                    res->time = (time() - start) / 1000.0;
                    break;
                }
                ctrl::vec2 apf_vec = apf::ball_field(robot, ball, d.genes[0], d.genes[1]);
                ctrl::vec2 command = ctrl::move_robot(robot, apf_vec, d.genes[2], 40.0*d.genes[3] + 10.0);
                sim_client.sendCommand(0, command[0], command[1]);
            }
        }
    };

    // Set robots outside of the field
    while (true)
    {
        if (client.receive(packet))
        {
            replacer.setRobotsOutside(true, packet.frame().robots_yellow_size());
            replacer.setRobotsOutside(false, packet.frame().robots_blue_size());
            break;
        }
    }
    replacer.setBallPos(0.1, 0.0);
    // Begin training
    for (size_t i = 0; i < MAX_GEN; i++)
    {
        pop.evaluate(fitness_simulate);
        // Print results
        pop.sort();
        std::cout << "Generation " << i + 1 << " ";
        std::cout << pop << " (Best individual - ";
        std::cout << *pop[0] << ")" << std::endl;
        // Select the best
        pop.select();
        // Generate a new population
        pop.generate();
    }
    return 0;
}

double fitness_simulate(DNA &d)
{
    ctrl::vec2 starts[] = {{0.5, 0.3}, {0.0, 0.4}, {-0.5, 0.3}, {-0.5, 0.0}};
    double fitness = 0.0;
    for (size_t i = 0; i < 4; i++)
    {
        measure f;
        simulate(starts[i], d, &f);
        fitness += f();
    }

    return 1.0 / (1.0 + fitness);
}

double test_fitness(DNA &d)
{
    double set_point[] = {2.0, 3.0, 3.5, 2.5};
    double fit = 0.0;
    for (size_t i = 0; i < d.len; i++)
        fit += (d.genes[i] - set_point[i]) * (d.genes[i] - set_point[i]);

    return 1.0 / (1.0 + fit);
}