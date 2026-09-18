// 同じソースを変更前後でビルドし、丸め前の値を16進表記で照合する。
//
// 注意(09): これは 03 と 05 の「変更前後で1ビットも変わっていない」を確かめる
// 比較専用の道具で、通常のテストではない。tools/verify_speedup.py と
// tools/verify_overhead.py が、変更前の古いソース（run_ga が事前計算表を取らない版）
// と現在のソースの両方に対してビルドするため、#ifdef SPEEDUP_INDEXED の
// 両側が残っている。マクロ無しでビルドできないのはそのためで、
// 現在のソースに対しては必ず -DSPEEDUP_INDEXED を付ける。
// 回帰テストとして常に走らせる厳密一致の検査は tests/local_search_test.cpp にある
// （test.bat と make test から実行される）。
#include "ga.h"
#include <algorithm>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <stdexcept>

using namespace orienteering;

void print_graph(const Graph& graph, const std::vector<long long>& sources) {
    for (long long source : sources) {
        const auto paths = graph.dijkstra_from(source);
        std::vector<long long> targets;
        for (const auto& entry : paths) targets.push_back(entry.first);
        std::sort(targets.begin(), targets.end());
        for (long long target : targets) {
            const auto& p = paths.at(target);
            std::cout << source << ' ' << target << ' ' << p.length << ' ' << p.gain << '\n';
        }
    }
}

int main() {
    std::cout << std::hexfloat;
    const auto landmarks = load_landmarks(DATA_DIR + "/landmarks.csv");
    const Graph graph(load_nodes(DATA_DIR + "/nodes.csv"), load_edges(DATA_DIR + "/edges.csv"));
    const auto gate = load_gate(DATA_DIR + "/seimon.csv");
    const auto gate_node = graph.find_nearest_node(gate.lat, gate.lon);
    std::vector<long long> sources;
    for (const auto& lm : landmarks) sources.push_back(lm.nearest_node);
    sources.push_back(gate_node);
    const PathCache cache(graph, sources);
    print_graph(graph, sources);

    // 入力順とID順が異なる同距離経路、並行辺、孤立点、未知の始点。
    const std::vector<Node> nodes{{90, 0, 0, 0}, {30, 0, 0, 0}, {20, 0, 0, 0},
                                  {40, 0, 0, 0}, {50, 0, 0, 0}, {60, 0, 0, 0}};
    const std::vector<Edge> edges{{90, 30, 1, 0, 9}, {90, 20, 1, 0, 1e16},
                                  {30, 40, 1, 0, 5}, {20, 40, 9, 0, 1},
                                  {20, 40, 1, 0, 7}, {40, 50, 0, 0, 1},
                                  {50, 40, 0, 0, 2}};
    print_graph(Graph(nodes, edges), {90, 20, 30, 40, 50, 60, 999});

#ifdef SPEEDUP_INDEXED
    const EvaluationTables tables(landmarks, cache, gate_node);
#endif
    RNG rng(2026);
    std::vector<int> indices(landmarks.size());
    std::iota(indices.begin(), indices.end(), 0);
    // 順序・地点数を変えた700コースで、表引きと従来の式を厳密比較する。
    for (int n = MIN_CONTROLS; n <= MAX_CONTROLS; ++n) {
        for (int trial = 0; trial < 100; ++trial) {
            std::shuffle(indices.begin(), indices.end(), rng);
            const std::vector<int> course(indices.begin(), indices.begin() + n);
            const auto chromosome = encode_course(course, static_cast<int>(landmarks.size()));
            const auto full = evaluate(chromosome, landmarks, cache, gate_node);
#ifdef SPEEDUP_INDEXED
            const auto indexed = evaluate(chromosome, landmarks, cache, gate_node, &tables);
            if (tables.proximity(course) != f_dist(course, landmarks)
                || evaluate_course(course, tables) != full.fitness
                || indexed.fitness != full.fitness || indexed.total_distance != full.total_distance
                || indexed.total_gain != full.total_gain) {
                throw std::runtime_error("表引きと従来評価の不一致");
            }
#endif
            std::cout << "course " << full.fitness << ' ' << full.total_distance << ' '
                      << full.total_gain << ' ' << full.objectives.f_dist << '\n';
        }
    }
    // 実験条件は20個体・20世代だけに固定する。
    for (unsigned int seed = 1; seed <= 3; ++seed) {
        RNG ga_rng(seed);
#ifdef SPEEDUP_INDEXED
        const auto result = run_ga(landmarks, cache, gate_node, tables, ga_rng, 20, 20);
#else
        const auto result = run_ga(landmarks, cache, gate_node, ga_rng, 20, 20);
#endif
        const auto& ev = result.best_eval;
        std::cout << "seed " << seed << ' ' << ev.fitness << ' ' << ev.total_distance
                  << ' ' << ev.total_gain << '\n';
        for (auto node : ev.decoded.course_nodes) std::cout << node << ' ';
        std::cout << '\n';
        for (double fitness : result.best_fitness_history) std::cout << fitness << ' ';
        std::cout << '\n';
    }
}
