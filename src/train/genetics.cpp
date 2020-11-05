#include "genetics.h"
#include <algorithm>
#include <cmath>

/**
 * @brief Make this a global variable
 * 
 */
Random randr;

/**
 * @brief Construct a new DNA object (without initialize genes)
 * 
 * @param len 
 */
DNA::DNA(unsigned int len)
{
    this->len = len;
    this->genes = new double[len];
    this->min = 0.0;
    this->max = 1.0;
    this->fitness = 0.0;
}

/**
 * @brief Construct a new DNA::DNA object
 * 
 * @param len Number of float pointer genes
 * @param min Min gene value
 * @param max Max gene value
 */
DNA::DNA(unsigned int len, double min, double max)
{
    this->len = len;
    this->genes = new double[len];
    this->min = min;
    this->max = max;
    this->fitness = 0.0;

    for (size_t i = 0; i < len; i++)
        this->genes[i] = randr(min, max);
}

/**
 * @brief Performs a non-uniform mutation in selected genes
 * 
 * @param mutation_rate 
 * @param mu Time ratio (generation/max_generation)
 */
void DNA::mutate(double mutation_rate, double mu)
{
    for (size_t i = 0; i < this->len; i++)
    {
        if (randr() < mutation_rate)
        {
            double delta = 1.0 - std::pow(randr(), std::pow(1.0 - mu, 5.0));
            delta = (this->max - this->min) * delta;
            double max_value = std::min(this->max, this->genes[i] + delta); // Bound value
            double min_value = std::max(this->min, this->genes[i] - delta);
            std::uniform_real_distribution<double> uniform_dist;
            // Mutate the gene
            this->genes[i] = randr(min_value, max_value);
        }
    }
}

/**
 * @brief Returns the child
 * 
 * @param partner 
 * @return DNA* 
 */
DNA *DNA::crossover(DNA &partner)
{
    DNA *child = new DNA(this->len);
    child->min = this->min;
    child->max = this->max;
    child->fitness = this->fitness;
    // Allows midpoint == this->len (no crossover)
    size_t midpoint = std::round(randr(this->len));
    for (size_t i = 0; i < this->len; i++)
    {
        if (i > midpoint)
            child->genes[i] = partner.genes[i];
        else
            child->genes[i] = this->genes[i];
    }

    return child;
}

/**
 * @brief Construct a new Population:: Population object
 * 
 * @param len Number of individuals
 * @param n_genes Number of genes on each individual
 * @param min Min value of gene (double)
 * @param max Max value of gene (double)
 * @param mut_rate Mutation rate
 * @param max_gen Maximum number of generations
 */
Population::Population(unsigned int len, unsigned int n_genes, double min, double max, double mut_rate, unsigned int max_gen)
    : population(len), selection_pool(len)
{
    this->generations = 0;
    this->max_generations = max_gen;
    this->mutation_rate = mut_rate;
    for (size_t i = 0; i < len; i++)
    {
        this->population[i] = new DNA(n_genes, min, max);
        this->selection_pool[i] = NULL;
    }
}

/**
 * @brief Evaluate each individual through callable fitness function
 * 
 * @param callable Fitness function tha uses a DNA genes
 */
void Population::evaluate(std::function<double(DNA &)> callable)
{
    for (unsigned int i = 0; i < this->population.size(); i++)
    {
        this->population[i]->fitness = callable(*(this->population[i]));
    }
}

/**
 * @brief Select individuals in a tournament selection
 * 
 */
void Population::select()
{
    std::random_device rd;
    auto rng = std::mt19937_64(rd());

    int total = this->population.size();
    for (int i = 0; i < total; i++)
    {
        std::shuffle(this->population.begin(), this->population.end(), rng);
        DNA *best = NULL;
        // Take three turns, with reposition
        for (int j = 0; j < 3; j++)
        {
            DNA *d = this->population[j % total];
            if (best == NULL or d->fitness > best->fitness)
                best = d;
        }
        this->selection_pool[i] = best;
    }
}

/**
 * @brief Generate a new population after evaluation+selection
 * 
 */
void Population::generate()
{
    std::vector<DNA *> old_pop = this->population;
    std::random_device rd;
    auto rng = std::default_random_engine(rd());
    std::shuffle(this->selection_pool.begin(), this->selection_pool.end(), rng);

    const size_t total_len = this->population.size();
    for (size_t i = 0; i < total_len; i++)
    {
        // Creates circular match
        this->population[i] = this->selection_pool[i]->crossover(*(this->selection_pool[(i + 1) % total_len]));
        double mu = double(this->generations) / this->max_generations;
        this->population[i]->mutate(this->mutation_rate, mu);
    }

    for (size_t i = 0; i < total_len; i++)
    {
        delete old_pop[i];
        this->selection_pool[i] = NULL;
    }

    this->generations += 1;
}

/**
 * @brief  Sorts population so this->population[0] is the
 *         best fitness (higest) 
 * 
 */
void Population::sort()
{
    auto comp = [](DNA *d1, DNA *d2) -> bool {
        return d1->fitness > d2->fitness;
    };
    std::sort(this->population.begin(), this->population.end(), comp);
}

/**
 * @brief Returns the average population fitness
 * 
 * @return double 
 */
double Population::average_fitness() const
{
    double av = 0.0;
    for (size_t i = 0; i < this->population.size(); i++)
        av += this->population[i]->fitness;

    return av / double(this->population.size());
}

Population::~Population()
{
    for (unsigned int i = 0; i < this->population.size(); i++)
        delete this->population[i];
}

std::ostream &operator<<(std::ostream &out, const Population &p)
{
    out << "Size:" << p.size();
    out << " AVG Fit:" << p.average_fitness();
    return out;
}