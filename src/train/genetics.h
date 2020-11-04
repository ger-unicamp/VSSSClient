#ifndef GENETICS
#define GENETICS

#include <functional>
#include <iostream>
#include <random>
#include <string>
#include <vector>

/**
 * @brief Global Class to generate uniform distribution random numbers in [0,1]
 * 
 */
class Random
{
    std::random_device r;
    std::mt19937_64 engine;
    std::uniform_real_distribution<double> uniform_dist;

public:
    Random() : engine(r()), uniform_dist(0.0, std::nextafter(1.0, std::numeric_limits<double>::max())) {}
    double operator()()
    {
        return this->uniform_dist(this->engine);
    }
    double operator()(double max)
    {
        return max * this->uniform_dist(this->engine);
    }
    double operator()(double min, double max)
    {
        double max_temp = (max > min) ? max : min;
        min = (min < max) ? min : max;
        max = max_temp;
        return min + (max - min) * this->uniform_dist(this->engine);
    }
};

/**
 * @brief Individual class on Genetic Algorithm (or other evolutionary programming)
 * 
 */
class DNA
{
public:
    double *genes;
    unsigned int len;
    double fitness, min, max;

    DNA(unsigned int len);
    DNA(unsigned int len, double min, double max);
    DNA *crossover(DNA &partner);
    void mutate(double mutation_rate, double mu);

    friend std::ostream &operator<<(std::ostream &out, const DNA &d)
    {
        out << "Fit:" << d.fitness;
        for (unsigned int i = 0; i < d.len; i++)
            out << " G" << i << ":" << d.genes[i];
        return out;
    }
    ~DNA() { delete[] this->genes; }
};

/**
 * @brief Collection of Individuals of an Genetic Algorithm
 * 
 */
class Population
{
public:
    std::vector<DNA *> population;
    std::vector<DNA *> selection_pool;
    size_t generations, max_generations;
    double mutation_rate;

    Population(unsigned int len, unsigned int n_genes, double min, double max, double mut_rate, unsigned int max_gen);
    void evaluate(std::function<double(DNA &)> callable);
    void select();
    void generate();
    void sort();
    double average_fitness() const;
    friend std::ostream &operator<<(std::ostream &out, const Population &p);
    size_t size() const
    {
        return this->population.size();
    }
    DNA *operator[](size_t i) const
    {
        return this->population[i];
    }
    ~Population();
};

#endif