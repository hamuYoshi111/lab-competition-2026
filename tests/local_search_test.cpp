#include "../const.h"
#include "../ga.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <numeric>
#include <set>
#include <stdexcept>

using namespace orienteering;

void check(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

int main() try {
    auto landmarks = load_landmarks(DATA_DIR + "/landmarks.csv");
    Graph graph(load_nodes(DATA_DIR + "/nodes.csv"), load_edges(DATA_DIR + "/edges.csv"));
    const auto gate = load_gate(DATA_DIR + "/seimon.csv");
    const auto gate_node = graph.find_nearest_node(gate.lat, gate.lon);
    std::set<long long> sources{gate_node};
    for (const auto& lm : landmarks) sources.insert(lm.nearest_node);
    PathCache cache(graph, std::vector<long long>(sources.begin(), sources.end()));
    RNG rng(2026);
    const int N = static_cast<int>(landmarks.size());
    int evaluated = 0;
    auto verify = [&](const std::vector<int>& course) {
        auto chrom = encode_course(course, N);
        const auto decoded = decode(chrom, landmarks, gate_node);
        check(decoded.is_valid && decoded.selected_indices == course, "コースの往復変換が不一致");
        std::vector<int> order(chrom.begin() + N, chrom.end());
        std::sort(order.begin(), order.end());
        for (int i = 0; i < MAX_CONTROLS; ++i) check(order[i] == i, "順序パートが順列ではない");
        const auto full = evaluate(chrom, landmarks, cache, gate_node);
        const auto direct = evaluate_course(course, landmarks, cache, gate_node);
        check(full.fitness == direct, "軽量評価と元の評価が不一致");
        ++evaluated;
        return direct;
    };
    // 6～12地点の各サイズで、順序が異なるコースの完全一致を確認する。
    std::vector<int> indices(N);
    std::iota(indices.begin(), indices.end(), 0);
    for (int n = MIN_CONTROLS; n <= MAX_CONTROLS; ++n) {
        for (int trial = 0; trial < 100; ++trial) {
            std::shuffle(indices.begin(), indices.end(), rng);
            verify(std::vector<int>(indices.begin(), indices.begin() + n));
        }
    }
    // 終了した解について、独立に列挙した全5近傍に改善手がないことを確かめる。
    for (int trial = 0; trial < 10; ++trial) {
        auto chrom = create_random_chromosome(N, rng);
        const auto before = evaluate(chrom, landmarks, cache, gate_node).fitness;
        local_search(chrom, landmarks, cache, gate_node);
        const auto course = decode(chrom, landmarks, gate_node).selected_indices;
        const double best = verify(course);
        check(best <= before, "局所探索で評価値が悪化");
        auto neighbor = [&](const std::vector<int>& c) {
            check(verify(c) >= best, "終了した解に改善可能な近傍が残っている");
        };
        const int n = static_cast<int>(course.size());
        for (int p = 0; p < N; ++p) {
            if (std::find(course.begin(), course.end(), p) != course.end()) continue;
            if (n < MAX_CONTROLS) for (int i = 0; i <= n; ++i) {
                auto c = course; c.insert(c.begin() + i, p); neighbor(c);
            }
            for (int i = 0; i < n; ++i) {
                auto c = course; c[i] = p; neighbor(c);
            }
        }
        for (int i = 0; i < n; ++i) {
            if (n > MIN_CONTROLS) {
                auto c = course; c.erase(c.begin() + i); neighbor(c);
            }
            for (int j = i + 1; j < n; ++j) {
                auto c = course; std::reverse(c.begin() + i, c.begin() + j + 1); neighbor(c);
            }
            for (int j = 0; j < n; ++j) {
                auto c = course; c.erase(c.begin() + i); c.insert(c.begin() + j, course[i]); neighbor(c);
            }
        }
    }
    // 地点数違反は修復しないこと、経路がない場合のペナルティ一致も確認する。
    Chromosome invalid(N + MAX_CONTROLS, 0);
    const auto original = invalid;
    local_search(invalid, landmarks, cache, gate_node);
    check(invalid == original, "無効な子が変更された");
    PathCache empty_cache(graph, {});
    std::vector<int> six{0, 1, 2, 3, 4, 5};
    check(evaluate_course(six, landmarks, empty_cache, gate_node) ==
          evaluate(encode_course(six, N), landmarks, empty_cache, gate_node).fitness,
          "到達不能時の評価が不一致");
    check(evaluate_course({}, landmarks, cache, gate_node) == PENALTY, "地点数違反の評価が不正");

    // ============================================================
    // 追加(09): GA が実際に通る「表引きの評価」と、配布元と同じ「参照の評価」が
    // 1ビットも違わないことを確かめる。もとは tests/speedup_test.cpp にあったが、
    // あちらは 03/05 の変更前後を比べる専用の道具で、どのビルド手順からも走らない。
    // そこで生きた回帰テストとしてここへ移し、test.bat と make test の両方から走らせる。
    // ============================================================
    const EvaluationTables tables(landmarks, cache, gate_node);
    int table_checked = 0;
    auto same_eval = [&](const Chromosome& chrom) {
        const auto full    = evaluate(chrom, landmarks, cache, gate_node);          // 参照経路
        const auto indexed = evaluate(chrom, landmarks, cache, gate_node, &tables); // 表引き経路
        check(indexed.is_valid == full.is_valid, "表引きと参照で有効・無効が不一致");
        check(indexed.fitness == full.fitness, "表引きと参照で fitness が不一致");
        check(indexed.total_distance == full.total_distance, "表引きと参照で距離が不一致");
        check(indexed.total_gain == full.total_gain, "表引きと参照で登りが不一致");
        check(indexed.objectives.f_map == full.objectives.f_map, "表引きと参照で f_map が不一致");
        check(indexed.objectives.f_dist == full.objectives.f_dist, "表引きと参照で f_dist が不一致");
        check(indexed.objectives.f_time == full.objectives.f_time, "表引きと参照で f_time が不一致");
        check(indexed.objectives.f_route == full.objectives.f_route, "表引きと参照で f_route が不一致");
        check(indexed.decoded.selected_indices == full.decoded.selected_indices,
              "表引きと参照で巡回順が不一致");
        check(indexed.decoded.course_nodes == full.decoded.course_nodes,
              "表引きと参照でコースが不一致");
        ++table_checked;
        return full;
    };
    RNG table_rng(20260918);
    // 地点数 6〜12 × 150 通り = 1050 件のランダムな巡回順。
    for (int n = MIN_CONTROLS; n <= MAX_CONTROLS; ++n) {
        for (int trial = 0; trial < 150; ++trial) {
            std::shuffle(indices.begin(), indices.end(), table_rng);
            const std::vector<int> course(indices.begin(), indices.begin() + n);
            const auto full = same_eval(encode_course(course, N));
            check(tables.proximity(course) == f_dist(course, landmarks),
                  "表引きの近接罰点が元の式と不一致");
            check(evaluate_course(course, tables) == full.fitness,
                  "表引きの巡回順評価が参照と不一致");
        }
    }
    // 地点数が制約から外れる染色体も混ぜ、無効解の扱いも一致することを見る。
    for (int trial = 0; trial < 200; ++trial)
        same_eval(create_random_chromosome(N, table_rng));
    check(table_checked >= 1000, "厳密一致の検査が1000件に満たない");
    // 到達不能（経路表が空）でも、両経路が同じペナルティを返すこと。
    const EvaluationTables empty_tables(landmarks, empty_cache, gate_node);
    const auto six_chrom = encode_course(six, N);
    check(evaluate(six_chrom, landmarks, empty_cache, gate_node, &empty_tables).fitness ==
          evaluate(six_chrom, landmarks, empty_cache, gate_node).fitness,
          "到達不能時に表引きと参照が不一致");

    // 人工グラフ：並行辺・到達不能・未知の始点・PathCache の起点欠落を踏む。
    // 期待値は実装の取り決め（並行辺は入力順で最初の辺の登りを使う、同距離は
    // ノードID順で先に確定した親を残す）をそのまま書き下したもの。
    {
        const std::vector<Node> art_nodes{{90, 0, 0, 0}, {30, 0, 0, 0}, {20, 0, 0, 0},
                                          {40, 0, 0, 0}, {50, 0, 0, 0}, {60, 0, 0, 0}};
        const std::vector<Edge> art_edges{{90, 30, 1, 0, 9}, {90, 20, 1, 0, 1e16},
                                          {30, 40, 1, 0, 5}, {20, 40, 9, 0, 1},
                                          {20, 40, 1, 0, 7}, {40, 50, 0, 0, 1},
                                          {50, 40, 0, 0, 2}};
        const Graph art(art_nodes, art_edges);
        auto path_is = [&](const std::unordered_map<long long, PathInfo>& paths,
                           long long node, double length, double gain) {
            const auto found = paths.find(node);
            check(found != paths.end(), "人工グラフで到達できるはずの点が無い");
            check(found->second.reachable, "人工グラフの経路が到達不能になった");
            check(found->second.length == length, "人工グラフの距離が想定と違う");
            check(found->second.gain == gain, "人工グラフの登りが想定と違う");
        };
        const auto from90 = art.dijkstra_from(90);
        path_is(from90, 30, 1.0, 9.0);
        path_is(from90, 20, 1.0, 1e16);
        // 90→40 は「30 経由」と「20 経由」がどちらも距離2。先に確定した 20 側が残る。
        path_is(from90, 40, 2.0, 1e16);
        // 登りは終点から親へ順に足す。90→20→40→50 は 1 + 1 + 1e16 = 1e16+2。
        path_is(from90, 50, 2.0, 1e16 + 2.0);
        check(from90.find(60) == from90.end(), "孤立点に到達できてしまった");
        // 並行辺 20→40 は（距離9・登り1）が先、（距離1・登り7）が後。
        // 距離は短い1、登りは入力順で最初の1を使う。
        const auto from20 = art.dijkstra_from(20);
        path_is(from20, 40, 1.0, 1.0);
        path_is(from20, 50, 1.0, 2.0);
        // 未知の始点は「自分自身だけ距離0」で返す。
        const auto from999 = art.dijkstra_from(999);
        check(from999.size() == 1, "未知の始点から他の点に到達できてしまった");
        path_is(from999, 999, 0.0, 0.0);
        // PathCache でも、グラフに無い起点は自分自身だけが距離0。
        const PathCache art_cache(art, {90, 40, 999});
        check(art_cache.get(90, 40).reachable && art_cache.get(90, 40).length == 2.0,
              "人工グラフの経路表が想定と違う");
        check(art_cache.get(999, 999).reachable && art_cache.get(999, 999).length == 0.0,
              "経路表で未知の起点が自分自身に到達できない");
        check(!art_cache.get(999, 90).reachable, "経路表で未知の起点から到達できてしまった");
        check(!art_cache.get(90, 999).reachable, "経路表で未知の終点に到達できてしまった");
    }

    std::cout << "PASS: " << evaluated << " courses; round-trip, exact evaluation, local optimum"
              << "; " << table_checked << " table-vs-reference exact matches; artificial graph"
              << std::endl;
} catch (const std::exception& e) {
    // 追加(09): 何が失敗したかを必ず表示する（表示せずに終わると原因が分からないため）。
    std::cout << "FAIL: " << e.what() << std::endl;
    return 1;
}
