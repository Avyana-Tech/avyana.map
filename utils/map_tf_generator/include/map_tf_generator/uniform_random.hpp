#ifndef MAP_TF_GENERATOR__UNIFORM_RANDOM_HPP_
#define MAP_TF_GENERATOR__UNIFORM_RANDOM_HPP_

#include <random>
#include <vector>

std::vector<size_t> UniformRandom(const size_t max_exclusive, const size_t n)
{
  std::default_random_engine generator;
  std::uniform_int_distribution<size_t> distribution(0, max_exclusive - 1);

  std::vector<size_t> v(n);
  for (size_t i = 0; i < n; i++) {
    v[i] = distribution(generator);
  }
  return v;
}

#endif  // MAP_TF_GENERATOR__UNIFORM_RANDOM_HPP_
