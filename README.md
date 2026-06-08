# TSP files explanation

There are 4 `.cpp` and 3 `.txt` files:

`.cpp` files:
- `GA_standard.cpp` - explained as the canonical **Ordered Crossover (OX)** baseline.
- `GA_advanced.cpp` - detailed with the **Generalized Partition Crossover (GPX3)**, graph topology analysis, and **Path Collapsing** operations.
- `GA.cpp` - presented as a hybrid **Memetic Optimization System** pairing global GPX3 recombination with fast **2-opt local search**.
- `GA_constrained.cpp` - structured around the **External Penalty Method** with $\mathcal{P}_{inf} = 10^{12}$ constraint enforcement.

`.txt` files:
- `16.txt` - contains `16` cities, mostly used for a base case for weaker algorithms. This files requires you to use GEO metric instead of euclidian.
- `52.txt` - contains `52` cities. Uses euclidian metric.
- `101.txt` - contains `101` cities, used as a benchmark for better algorithms. Uses euclidian metric.

# How to use

In order to read a specific `.txt` file you need to change this string in a `.cpp` file in the `main()` function:

```c++
  std::string filename = "number_of_cities.txt";
```

Every `.cpp` file has this line.
