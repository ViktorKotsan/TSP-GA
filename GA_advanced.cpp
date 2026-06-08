#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <fstream>
#include <sstream>
#include <string>
#include <numeric>
#include <random>
#include <chrono>

// =================================================================
// 1. СТРУКТУРА МІСТА ТА СЕРЕДОВИЩА TSP
// =================================================================

struct City {
    int id;
    double x, y;
};

class TSP_Environment {
public:
    std::vector<City> cities;
    std::vector<std::vector<long long>> distance_matrix;

    void addCity(int id, double x, double y) {
        cities.push_back({id, x, y});
    }

    void buildDistanceMatrix() {
        int n = cities.size();
        distance_matrix.assign(n, std::vector<long long>(n, 0));
        for (int i = 0; i < n; ++i) {
            for (int j = 0; j < n; ++j) {
                if (i == j) distance_matrix[i][j] = 0;
                else {
                    double dx = cities[i].x - cities[j].x;
                    double dy = cities[i].y - cities[j].y;
                    distance_matrix[i][j] = static_cast<long long>(std::floor(std::sqrt(dx * dx + dy * dy) + 0.5));
                }
            }
        }
    }

    bool loadFromFile(const std::string& filename) {
        std::ifstream file(filename);
        if (!file.is_open()) {
            std::cerr << "Помилка: не вдалося відкрити файл " << filename << "\n";
            return false;
        }
        cities.clear();
        std::string line;
        while (std::getline(file, line)) {
            if (line.empty()) continue;
            std::stringstream ss(line);
            int id; double x, y;
            if (ss >> id >> x >> y) addCity(id, x, y);
        }
        file.close();
        buildDistanceMatrix();
        std::cout << "Успішно завантажено міст: " << cities.size() << " з файлу " << filename << "\n";
        return true;
    }

    long long getDistance(int idxA, int idxB) const { return distance_matrix[idxA][idxB]; }
    int getNumCities() const { return cities.size(); }
};

// =================================================================
// 2. ПОДАННЯ ІНДИВІДА
// =================================================================

struct Individual {
    std::vector<int> route;
    long long total_distance;

    void calculateFitness(const TSP_Environment& env) {
        total_distance = 0;
        int n = route.size();
        for (int i = 0; i < n - 1; ++i) total_distance += env.getDistance(route[i], route[i + 1]);
        total_distance += env.getDistance(route[n - 1], route[0]);
    }
};

// =================================================================
// 3. ПРОСУНУТИЙ ГЕНЕТИЧНИЙ АЛГОРИТМ (GPX3 + MUTATION)
// =================================================================

class AdvancedGeneticAlgorithm {
private:
    std::vector<Individual> population;
    int population_size;
    const TSP_Environment& env;
    std::mt19937 rng;

public:
    std::vector<std::pair<int, long long>> convergence_history;

    AdvancedGeneticAlgorithm(int pop_size, const TSP_Environment& environment) 
        : population_size(pop_size), env(environment) {
        rng = std::mt19937(std::chrono::system_clock::now().time_since_epoch().count());
    }

    void initializePopulation() {
        int n = env.getNumCities();
        std::vector<int> base_route(n);
        std::iota(base_route.begin(), base_route.end(), 0);
        for (int i = 0; i < population_size; ++i) {
            Individual ind;
            ind.route = base_route;
            std::shuffle(ind.route.begin(), ind.route.end(), rng);
            ind.calculateFitness(env);
            population.push_back(ind);
        }
    }

    Individual tournamentSelection(int size = 5) {
        std::uniform_int_distribution<int> dist(0, population_size - 1);
        Individual best = population[dist(rng)];
        for (int i = 1; i < size; ++i) {
            Individual competitor = population[dist(rng)];
            if (competitor.total_distance < best.total_distance) best = competitor;
        }
        return best;
    }

    // ПРОСУНУТИЙ ГРАФОВИЙ КРОСОВЕР GPX3
    Individual gpx3Crossover(const Individual& parent1, const Individual& parent2) {
        int n = env.getNumCities();
        Individual child; 
        child.route.reserve(n);

        std::vector<std::vector<int>> graph_union(n);
        
        auto add_edges = [&](const std::vector<int>& route) {
            for (int i = 0; i < n; ++i) {
                int curr = route[i];
                int next = route[(i + 1) % n];
                int prev = route[(i - 1 + n) % n];
                
                if (std::find(graph_union[curr].begin(), graph_union[curr].end(), next) == graph_union[curr].end()) {
                    graph_union[curr].push_back(next);
                }
                if (std::find(graph_union[curr].begin(), graph_union[curr].end(), prev) == graph_union[curr].end()) {
                    graph_union[curr].push_back(prev);
                }
            }
        };

        add_edges(parent1.route);
        add_edges(parent2.route);

        std::vector<bool> visited(n, false);
        int current_city = parent1.route[0];
        child.route.push_back(current_city);
        visited[current_city] = true;

        for (int step = 1; step < n; ++step) {
            for (int i = 0; i < n; ++i) {
                graph_union[i].erase(std::remove(graph_union[i].begin(), graph_union[i].end(), current_city), graph_union[i].end());
            }

            int next_city = -1;
            int min_edges_left = 10; 

            for (int neighbor : graph_union[current_city]) {
                if (!visited[neighbor]) {
                    int edges_left = graph_union[neighbor].size();
                    if (edges_left < min_edges_left) {
                        min_edges_left = edges_left;
                        next_city = neighbor;
                    } else if (edges_left == min_edges_left) {
                        if (next_city == -1 || env.getDistance(current_city, neighbor) < env.getDistance(current_city, next_city)) {
                            next_city = neighbor;
                        }
                    }
                }
            }

            if (next_city == -1) {
                long long best_global_dist = 1e18;
                for (int i = 0; i < n; ++i) {
                    if (!visited[i]) {
                        long long d = env.getDistance(current_city, i);
                        if (d < best_global_dist) {
                            best_global_dist = d;
                            next_city = i;
                        }
                    }
                }
            }

            current_city = next_city;
            child.route.push_back(current_city);
            visited[current_city] = true;
        }

        return child;
    }

    // Мутація інверсією
    void mutate(Individual& ind, double rate = 0.08) {
        std::uniform_real_distribution<double> chance(0, 1);
        if (chance(rng) < rate) {
            std::uniform_int_distribution<int> dist(0, env.getNumCities() - 1);
            int i = dist(rng), j = dist(rng);
            if (i > j) std::swap(i, j);
            std::reverse(ind.route.begin() + i, ind.route.begin() + j + 1);
        }
    }

    void evolve(int current_generation) {
        std::vector<Individual> new_pop;
        
        // Елітизм
        auto best_it = std::min_element(population.begin(), population.end(), 
            [](const Individual& a, const Individual& b) { return a.total_distance < b.total_distance; });
        new_pop.push_back(*best_it);

        // Фіксуємо точку для графіка збіжності
        convergence_history.push_back({current_generation, best_it->total_distance});

        while (new_pop.size() < population_size) {
            Individual p1 = tournamentSelection(), p2 = tournamentSelection();
            Individual child = gpx3Crossover(p1, p2);
            mutate(child);
            child.calculateFitness(env);
            new_pop.push_back(child);
        }
        population = new_pop;
    }

    Individual getBestIndividual() {
        return *std::min_element(population.begin(), population.end(), 
            [](const Individual& a, const Individual& b) { return a.total_distance < b.total_distance; });
    }
};

// =================================================================
// СЕРВІСНІ ФУНКЦІЇ ДЛЯ ЗВІТІВ ТА ГРАФІКИ
// =================================================================

void printBestRoute(const Individual& best, const TSP_Environment& env) {
    std::cout << "\n--- Фінальний маршрут (Послідовність ID міст) ---\n";
    for (int idx : best.route) {
        std::cout << env.cities[idx].id << " -> ";
    }
    std::cout << env.cities[best.route[0]].id << " (Повернення)\n";
}

void exportConvergenceToSVG(const std::string& filename, const std::vector<std::pair<int, long long>>& history) {
    std::ofstream file(filename);
    if (!file.is_open()) return;

    int width = 1000;
    int height = 500;
    int padding = 60;

    long long max_dist = history.front().second;
    long long min_dist = history.back().second;
    int total_gens = history.size();

    if (max_dist == min_dist) max_dist += 10;

    file << "<svg width=\"" << width << "\" height=\"" << height << "\" xmlns=\"http://www.w3.org/2000/svg\">\n";
    file << "<rect width=\"100%\" height=\"100%\" fill=\"#ffffff\"/>\n";
    
    file << "<line x1=\"" << padding << "\" y1=\"" << height - padding << "\" x2=\"" << width - padding << "\" y2=\"" << height - padding << "\" stroke=\"#333\" stroke-width=\"2\"/>\n";
    file << "<line x1=\"" << padding << "\" y1=\"" << padding << "\" x2=\"" << padding << "\" y2=\"" << height - padding << "\" stroke=\"#333\" stroke-width=\"2\"/>\n";

    file << "<text x=\"" << width / 2 << "\" y=\"" << height - 15 << "\" font-size=\"14\" text-anchor=\"middle\" fill=\"#333\">Покоління (Generations)</text>\n";
    file << "<text x=\"15\" y=\"" << height / 2 << "\" font-size=\"14\" text-anchor=\"middle\" transform=\"rotate(-90 15," << height / 2 << ")\" fill=\"#333\">Найкраща відстань (Distance)</text>\n";

    auto getX = [&](int gen) { return padding + (double)gen / total_gens * (width - 2 * padding); };
    auto getY = [&](long long dist) { return height - padding - ((double)(dist - min_dist) / (max_dist - min_dist) * (height - 2 * padding)); };

    file << "<text x=\"" << padding - 10 << "\" y=\"" << padding + 5 << "\" font-size=\"12\" text-anchor=\"end\" fill=\"#666\">" << max_dist << "</text>\n";
    file << "<text x=\"" << padding - 10 << "\" y=\"" << height - padding + 5 << "\" font-size=\"12\" text-anchor=\"end\" fill=\"#666\">" << min_dist << "</text>\n";

    // Лінія графіка буде синьою для просунутого ГА (щоб візуально відрізняти від стандартного)
    file << "<polyline points=\"";
    for (const auto& point : history) { file << getX(point.first) << "," << getY(point.second) << " "; }
    file << "\" fill=\"none\" stroke=\"#007bff\" stroke-width=\"2.5\" />\n";

    file << "</svg>";
    file.close();
    std::cout << "[!] Графік збіжності успішно збережено у файл: " << filename << "\n";
}

void exportRouteToSVG(const std::string& filename, const Individual& best, const TSP_Environment& env) {
    std::ofstream file(filename);
    if (!file.is_open()) return;

    double min_x = 1e9, max_x = -1e9, min_y = 1e9, max_y = -1e9;
    for (const auto& c : env.cities) {
        if (c.x < min_x) min_x = c.x; if (c.x > max_x) max_x = c.x;
        if (c.y < min_y) min_y = c.y; if (c.y > max_y) max_y = c.y;
    }
    
    double padding = 50.0;
    double scale = 800.0 / std::max(max_x - min_x, max_y - min_y);

    file << "<svg width=\"900\" height=\"900\" xmlns=\"http://www.w3.org/2000/svg\">\n";
    file << "<rect width=\"100%\" height=\"100%\" fill=\"#f8f9fa\"/>\n"; 

    // Малюємо лінії (ребра маршруту) — фірмові сині
    file << "<g stroke=\"#007bff\" stroke-width=\"2\" fill=\"none\">\n";
    for (size_t i = 0; i < best.route.size(); ++i) {
        const City& c1 = env.cities[best.route[i]];
        const City& c2 = env.cities[best.route[(i + 1) % best.route.size()]];
        
        double x1 = (c1.x - min_x) * scale + padding;
        double y1 = (max_y - c1.y) * scale + padding;
        double x2 = (c2.x - min_x) * scale + padding;
        double y2 = (max_y - c2.y) * scale + padding;
        
        file << "  <line x1=\"" << x1 << "\" y1=\"" << y1 << "\" x2=\"" << x2 << "\" y2=\"" << y2 << "\" />\n";
    }
    file << "</g>\n";

    // Малюємо міста (вузли) червоним кольором + підписи ID
    file << "<g fill=\"#dc3545\">\n";
    for (const auto& c : env.cities) {
        double x = (c.x - min_x) * scale + padding;
        double y = (max_y - c.y) * scale + padding;
        file << "  <circle cx=\"" << x << "\" cy=\"" << y << "\" r=\"4\" />\n";
        file << "  <text x=\"" << x + 6 << "\" y=\"" << y - 6 << "\" font-size=\"10\" fill=\"#333\">" << c.id << "</text>\n";
    }
    file << "</g>\n";
    file << "</svg>";
    file.close();
    
    std::cout << "[!] Карту маршруту успішно збережено у файл: " << filename << "\n";
}

// =================================================================
// ГОЛОВНА ФУНКЦІЯ
// =================================================================

int main() {
    std::string_view locale = "uk_UA.UTF-8";
    std::setlocale(LC_ALL, locale.data());

    TSP_Environment env;
    std::string filename = "101.txt"; 
    
    if (env.loadFromFile(filename)) {
        int population_size = 100;
        int generations = 1000; // Робимо 1000 поколінь для чесного порівняння зі Стандартним ГА

        AdvancedGeneticAlgorithm ga(population_size, env);
        ga.initializePopulation();
        
        std::cout << "Просунута еволюція запущена (GPX3 + Mutation)...\n";

        auto start_time = std::chrono::high_resolution_clock::now();

        for (int gen = 1; gen <= generations; ++gen) {
            ga.evolve(gen);
            if (gen % 100 == 0) {
                std::cout << "Покоління " << gen << " | Найкраща відстань: " << ga.getBestIndividual().total_distance << "\n";
            }
        }

        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

        Individual best_final = ga.getBestIndividual();
        
        std::cout << "\n============================================\n";
        std::cout << "ЗВІТ: ПРОСУНУТИЙ ГЕНЕТИЧНИЙ АЛГОРИТМ (GPX3)\n";
        std::cout << "============================================\n";
        std::cout << "Файл даних: " << filename << "\n";
        std::cout << "Найкраща відстань: " << best_final.total_distance << "\n";

        if (filename == "52.txt") {
            std::cout << "Точна найкраща відстань: 7542" << "\n";
        }
        if (filename == "101.txt") {
            std::cout << "Точна найкраща відстань: 629" << "\n";
        }

        std::cout << "Час виконання: " << duration.count() << " мс (" << duration.count() / 1000.0 << " сек)\n";
        
        // Порядок міст
        printBestRoute(best_final, env);
        
        // Генерація графіки
        exportConvergenceToSVG("convergence_advanced.svg", ga.convergence_history);
        exportRouteToSVG("route_advanced.svg", best_final, env);
        std::cout << "(Відкрийте файли .svg у будь-якому браузері для перегляду карти та графіка)\n";
    }
    return 0;
}