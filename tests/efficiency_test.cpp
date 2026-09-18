#include "../ga.h"
#include <algorithm>
#include <iostream>
#include <numeric>
#include <sstream>
#include <stdexcept>

using namespace orienteering;

void check(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

int main() try {
    const auto landmarks = load_landmarks(DATA_DIR + "/landmarks.csv");
    Graph graph(load_nodes(DATA_DIR + "/nodes.csv"), load_edges(DATA_DIR + "/edges.csv"));
    const auto gate = load_gate(DATA_DIR + "/seimon.csv");
    const auto gate_node = graph.find_nearest_node(gate.lat, gate.lon);
    std::vector<long long> sources{gate_node};
    for (const auto& lm : landmarks) sources.push_back(lm.nearest_node);
    const PathCache cache(graph, sources);
    const EvaluationTables tables(landmarks, cache, gate_node);
    const int N = static_cast<int>(landmarks.size());
    int cases = 0;
    int neighbors = 0;

    for (int k : {0, 1, 20, 40, N + 1}) for (bool first : {false, true}) {
        SearchOptions options;
        options.candidate_k = k;
        options.first_improvement = first;
        options.quiet = true;
        for (unsigned seed = 1; seed <= 3; ++seed) {
            RNG before_rng(seed), rng(seed);
            const auto before = create_random_chromosome(N, before_rng);
            // 個体1なら交叉が起きず、初期個体の局所探索だけを検証できる。
            const auto result = run_ga(landmarks, cache, gate_node, tables, rng, 1, 1, options);
            check(rng == before_rng, "局所探索が乱数を消費した");
            check(result.best_eval.fitness <= evaluate(before, landmarks, cache, gate_node).fitness,
                  "局所探索で評価が悪化した");
            const auto course = result.best_eval.decoded.selected_indices;
            const int n = static_cast<int>(course.size());
            std::vector<int> unused;
            for (int p = 0; p < N; ++p)
                if (std::find(course.begin(), course.end(), p) == course.end()) unused.push_back(p);
            // 実装の許可表を使わず、その場で未選択地点を距離順に並べて確認する。
            auto allowed = [&](int point, int before, int after) {
                if (k == 0) return true;
                for (int endpoint : {before, after}) {
                    auto nearest = unused;
                    std::sort(nearest.begin(), nearest.end(), [&](int a, int b) {
                        const auto da = tables.path(endpoint, a).length;
                        const auto db = tables.path(endpoint, b).length;
                        return da < db || (da == db && a < b);
                    });
                    const auto rank = std::find(nearest.begin(), nearest.end(), point) - nearest.begin();
                    if (rank < k) return true;
                }
                return false;
            };
            auto neighbor = [&](const std::vector<int>& c) {
                check(evaluate_course(c, tables) >= result.best_eval.fitness,
                      "許可された近傍に改善手が残った");
                ++neighbors;
            };
            for (int p : unused) {
                if (n < MAX_CONTROLS) for (int pos = 0; pos <= n; ++pos) {
                    if (!allowed(p, pos ? course[pos - 1] : N, pos == n ? N : course[pos])) continue;
                    auto c = course; c.insert(c.begin() + pos, p); neighbor(c);
                }
                for (int pos = 0; pos < n; ++pos) {
                    if (!allowed(p, pos ? course[pos - 1] : N, pos + 1 == n ? N : course[pos + 1])) continue;
                    auto c = course; c[pos] = p; neighbor(c);
                }
            }
            for (int i = 0; i < n; ++i) {
                if (n > MIN_CONTROLS) { auto c = course; c.erase(c.begin() + i); neighbor(c); }
                for (int j = i + 1; j < n; ++j) {
                    auto c = course; std::reverse(c.begin() + i, c.begin() + j + 1); neighbor(c);
                }
                for (int j = 0; j < n; ++j) {
                    auto c = course; c.erase(c.begin() + i); c.insert(c.begin() + j, course[i]); neighbor(c);
                }
            }
            if (k > N) {
                auto full_options = options;
                full_options.candidate_k = 0;
                RNG full_rng(seed);
                const auto full = run_ga(landmarks, cache, gate_node, tables, full_rng, 1, 1, full_options);
                check(full.best_chromosome == result.best_chromosome, "Kが全候補以上でも全探索と不一致");
            }
            // 複数世代で重複スキップを有効にしても、染色体・値・履歴・乱数状態を保つ。
            RNG plain_rng(seed), cached_rng(seed);
            const auto plain = run_ga(landmarks, cache, gate_node, tables, plain_rng, 5, 5, options);
            options.dedupe = true;
            const auto cached = run_ga(landmarks, cache, gate_node, tables, cached_rng, 5, 5, options);
            options.dedupe = false;
            check(plain.best_chromosome == cached.best_chromosome &&
                  plain.best_eval.fitness == cached.best_eval.fitness &&
                  plain.best_fitness_history == cached.best_fitness_history && plain_rng == cached_rng,
                  "重複スキップで結果が変わった");
            ++cases;
        }
    }

    // 05で足した「賢い初期解」の検証。
    // 地点の重複がなく、150m未満のペアがなく、推定時間が60分以内で、
    // 地点数が6〜8に収まること。8地点に届かないのは、60分の枠に入る未選択地点が
    // 尽きた場合で、仕様どおり（あとは局所探索が地点数を直す）。
    // さらに同じ種なら毎回同じ個体になること。
    int greedy_cases = 0;
    int greedy_full = 0;
    for (unsigned seed = 1; seed <= 20; ++seed) {
        RNG rng(seed), repeat(seed);
        const auto chromosome = create_greedy_chromosome(N, rng, tables);
        check(chromosome == create_greedy_chromosome(N, repeat, tables), "貪欲初期解が再現しない");
        const auto decoded = decode(chromosome, landmarks, gate_node);
        check(decoded.is_valid, "貪欲初期解が地点数の制約を満たさない");
        const auto& course = decoded.selected_indices;
        const int n_greedy = static_cast<int>(course.size());
        check(MIN_CONTROLS <= n_greedy && n_greedy <= Q_TARGET, "貪欲初期解の地点数が想定外");
        if (n_greedy == Q_TARGET) ++greedy_full;
        auto sorted = course;
        std::sort(sorted.begin(), sorted.end());
        check(std::unique(sorted.begin(), sorted.end()) == sorted.end(), "貪欲初期解に同じ地点がある");
        double distance = 0.0, gain = 0.0;
        size_t from = tables.gate_index();
        for (size_t i = 0; i <= course.size(); ++i) {
            const size_t to = i == course.size() ? tables.gate_index() : course[i];
            const auto& path = tables.path(from, to);
            check(path.reachable, "貪欲初期解に到達できない区間がある");
            distance += path.length;
            gain += path.gain;
            from = to;
        }
        check((distance / WALK_SPEED + gain / CLIMB_SPEED) * 60.0 <= T_TARGET,
              "貪欲初期解の推定時間が目標を超えた");
        check(tables.proximity(course) == 0.0, "貪欲初期解に150m未満のペアがある");
        ++greedy_cases;
    }

    // 3通りの初期化がそれぞれ動き、ランダム指定は既定と同じ結果になること。
    for (unsigned seed = 1; seed <= 3; ++seed) {
        SearchOptions options;
        options.quiet = true;
        RNG base_rng(seed);
        const auto base = run_ga(landmarks, cache, gate_node, tables, base_rng, 6, 3, options);
        options.init = InitMethod::Random;
        RNG same_rng(seed);
        const auto same = run_ga(landmarks, cache, gate_node, tables, same_rng, 6, 3, options);
        check(base.best_chromosome == same.best_chromosome && base_rng == same_rng,
              "--init random が既定と一致しない");
        for (auto method : {InitMethod::GreedyAll, InitMethod::GreedyHalf}) {
            options.init = method;
            RNG rng(seed);
            const auto result = run_ga(landmarks, cache, gate_node, tables, rng, 6, 3, options);
            check(result.best_eval.decoded.is_valid, "貪欲初期解のGAが無効解を返した");
            check(result.best_eval.fitness <= result.initial_best_fitness,
                  "世代交代で初期集団より悪くなった");
            ++greedy_cases;
        }
    }

    std::cout << "PASS: " << cases << " settings/seeds, " << neighbors
              << " neighbors, " << greedy_cases << " greedy-init checks (" << greedy_full << "/20 reached 8)"
              << "; non-worsening, restricted local optimum, cache equivalence, RNG" << std::endl;
} catch (const std::exception& e) {
    // 何が失敗したかを必ず表示する（表示せずに終わると原因が分からないため）。
    std::cout << "FAIL: " << e.what() << std::endl;
    return 1;
}
