#include <iostream>
#include "train/genetics.h"

extern Random randr;
const size_t MAX_GEN = 200;
const size_t N_POP = 100;

double test_fitness(DNA &d)
{
    double set_point = 2.0;
    double fit = 0.0;
    for (size_t i = 0; i < d.len; i++)
        fit += (d.genes[i] - set_point) * (d.genes[i] - set_point);

    return 1.0 / (1.0 + fit);
}

int main()
{
    Population pop(N_POP, 4, 0.0, 4.0, 0.2, MAX_GEN);
    for (size_t i = 0; i < MAX_GEN; i++)
    {
        pop.evaluate(test_fitness);
        // Print results
        pop.sort();
        std::cout << "Generation " << i + 1 << " Pop";
        std::cout << pop << " (BEST - ";
        std::cout << *pop[0] << ")" << std::endl;
        // Select the best
        pop.select();
        // Generate a new population
        pop.generate();
    }
    return 0;
}
