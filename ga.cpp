// 初期個体の生成と GA の世代交代を実装し、局所探索で各個体のコースを改善する。
#include "ga.h"
#include "const.h"

#include <algorithm>
#include <iostream>
#include <limits>
#include <numeric>
#include <stdexcept>

namespace orienteering {

// 近い候補の順序を1回のGA実行で共有する。
struct SearchContext {
    SearchOptions options;
    std::vector<std::vector<int>> nearest;

    SearchContext(const EvaluationTables& tables, const SearchOptions& settings)
        : options(settings)
    {
        if (options.candidate_k < 0) throw std::invalid_argument("候補数は0以上を指定してください");
        if (options.candidate_k == 0) return;
        const int count = static_cast<int>(tables.gate_index());
        nearest.resize(count + 1);
        for (int from = 0; from <= count; ++from) {
            auto& points = nearest[from];
            points.resize(count);
            std::iota(points.begin(), points.end(), 0);
            // 同距離は候補番号で決め、毎回同じ並びにする。正門も含める。
            std::sort(points.begin(), points.end(), [&](int a, int b) {
                const auto& pa = tables.path(from, a);
                const auto& pb = tables.path(from, b);
                const double da = pa.reachable ? pa.length : std::numeric_limits<double>::infinity();
                const double db = pb.reachable ? pb.length : std::numeric_limits<double>::infinity();
                return da < db || (da == db && a < b);
            });
        }
    }
};

// 地点の追加・置換・削除と順序の変更を試し、改善がなくなるまで局所探索する。
static void local_search_with_tables(Chromosome& chromosome, const std::vector<Landmark>& landmarks,
                  const EvaluationTables& tables, long long gate_node, SearchContext& context)
{
    const auto decoded = decode(chromosome, landmarks, gate_node);
    // 制約違反の子は配布GAと同じペナルティで扱う。修復は別の変更になるため行わない。
    if (!decoded.is_valid) return;
    auto course = decoded.selected_indices;
    double current = evaluate_course(course, tables);
    // 作業用の領域は一度確保し、近傍ごとの確保を避ける。
    std::vector<int> candidate, best_course;
    candidate.reserve(MAX_CONTROLS);
    best_course.reserve(MAX_CONTROLS);
    course.reserve(MAX_CONTROLS);
    std::vector<unsigned char> selected(landmarks.size());
    std::vector<std::vector<unsigned char>> insert_allowed, replace_allowed;
    for (;;) {
        best_course = course;
        double best = current;
        // 丸めによる採否の変化を避けるため、評価の加算順序を保つ。
        auto consider = [&](const std::vector<int>& candidate) {
            const double score = evaluate_course(candidate, tables);
            // 同点移動は行わず、列挙順で最初に見つかった最良手を残す。
            if (score < best) {
                best = score;
                best_course = candidate;
            }
        };
        const int n = static_cast<int>(course.size());
        std::fill(selected.begin(), selected.end(), false);
        for (int point : course) selected[point] = true;
        if (context.options.candidate_k > 0) {
            const int gate = static_cast<int>(tables.gate_index());
            // 前後それぞれから近い未選択K地点の和集合を許可する（最大2K地点）。
            auto allow = [&](std::vector<unsigned char>& mask, int before, int after) {
                mask.assign(landmarks.size(), false);
                for (int endpoint : {before, after}) {
                    int count = 0;
                    for (int point : context.nearest[endpoint]) {
                        if (selected[point]) continue;
                        mask[point] = true;
                        if (++count == context.options.candidate_k) break;
                    }
                }
            };
            insert_allowed.resize(n + 1);
            replace_allowed.resize(n);
            for (int pos = 0; pos <= n; ++pos)
                allow(insert_allowed[pos], pos == 0 ? gate : course[pos - 1],
                      pos == n ? gate : course[pos]);
            for (int pos = 0; pos < n; ++pos)
                allow(replace_allowed[pos], pos == 0 ? gate : course[pos - 1],
                      pos + 1 == n ? gate : course[pos + 1]);
        }
        for (int point = 0; point < static_cast<int>(landmarks.size()); ++point) {
            if (selected[point]) continue;
            // 正門の直後から直前まで、すべての挿入位置を調べる。
            if (n < MAX_CONTROLS) {
                for (int pos = 0; pos <= n; ++pos) {
                    if (context.options.candidate_k > 0 && !insert_allowed[pos][point]) continue;
                    candidate = course;
                    candidate.insert(candidate.begin() + pos, point);
                    consider(candidate);
                }
            }
            // 置換：順序上の位置を保ち、地点だけを交換する。
            for (int pos = 0; pos < n; ++pos) {
                if (context.options.candidate_k > 0 && !replace_allowed[pos][point]) continue;
                candidate = course;
                candidate[pos] = point;
                consider(candidate);
            }
        }
        for (int i = 0; i < n; ++i) {
            if (n > MIN_CONTROLS) {
                candidate = course;
                candidate.erase(candidate.begin() + i);
                consider(candidate);
            }
            // 2-opt：正門を固定し、選択地点の連続区間を反転する。
            for (int j = i + 1; j < n; ++j) {
                candidate = course;
                std::reverse(candidate.begin() + i, candidate.begin() + j + 1);
                consider(candidate);
            }
            // Or-opt：1地点を抜き取り、短くなったリストの各位置に挿入する。
            for (int j = 0; j < n; ++j) {
                if (i == j) continue;
                candidate = course;
                candidate.erase(candidate.begin() + i);
                candidate.insert(candidate.begin() + j, course[i]);
                consider(candidate);
            }
        }
        if (!(best < current)) break;
        course = best_course;
        current = best;
    }
    chromosome = encode_course(course, static_cast<int>(landmarks.size()));
}

// 事前計算表を用意し、候補を絞らない全近傍の局所探索を提供する。
void local_search(Chromosome& chromosome, const std::vector<Landmark>& landmarks,
                  const PathCache& path_cache, long long gate_node)
{
    const EvaluationTables tables(landmarks, path_cache, gate_node);
    // 候補を絞らず全近傍を調べる。
    SearchContext context(tables, SearchOptions{});
    local_search_with_tables(chromosome, landmarks, tables, gate_node, context);
}

// ============================================================
// ランダムな染色体の選択数を候補数以下に抑え、配列の範囲外への書き込みを防ぐ。
// ============================================================
Chromosome create_random_chromosome(int N, RNG& rng) {
    // 選ぶ数の上限を候補数までに抑え、配列の範囲外へのアクセスを防ぐ。
    const int max_select = N < MAX_CONTROLS ? N : MAX_CONTROLS;
    std::uniform_int_distribution<int> dist_n(MIN_CONTROLS, max_select);
    int n_select = dist_n(rng);

    Chromosome chrom(N + MAX_CONTROLS, 0);

    // 選択パート：N個の候補からランダムに n_select 個を選ぶ
    std::vector<int> indices(N);
    std::iota(indices.begin(), indices.end(), 0);
    std::shuffle(indices.begin(), indices.end(), rng);
    for (int i = 0; i < n_select; ++i) {
        chrom[indices[i]] = 1;
    }

    // 順序パート：0〜MAX_CONTROLS-1 のランダム順列
    std::vector<int> order(MAX_CONTROLS);
    std::iota(order.begin(), order.end(), 0);
    std::shuffle(order.begin(), order.end(), rng);
    for (int i = 0; i < MAX_CONTROLS; ++i) {
        chrom[N + i] = order[i];
    }
    return chrom;
}

// ============================================================
// トーナメント選択
// ============================================================
const Chromosome& tournament_select(
    const std::vector<Chromosome>& population,
    const std::vector<double>&     fitnesses,
    RNG&                           rng)
{
    std::uniform_int_distribution<int> dist(0, static_cast<int>(population.size()) - 1);
    int    best     = -1;
    double best_fit = std::numeric_limits<double>::max();
    for (int i = 0; i < TOURNAMENT_SIZE; ++i) {
        int idx = dist(rng);
        if (fitnesses[idx] <= best_fit) {
            best_fit = fitnesses[idx];
            best     = idx;
        }
    }
    return population[best];
}

// ============================================================
// 交叉
//   選択パート：一様交叉
//   順序パート：OX（順序交叉／順列を保存）
// ============================================================
std::pair<Chromosome, Chromosome> crossover(
    const Chromosome& parent1,
    const Chromosome& parent2,
    int               N,
    RNG&              rng)
{
    Chromosome c1(N + MAX_CONTROLS);
    Chromosome c2(N + MAX_CONTROLS);

    std::uniform_real_distribution<double> ureal(0.0, 1.0);

    // 選択パート：各ビットを 50% で入れ替え
    for (int i = 0; i < N; ++i) {
        if (ureal(rng) < 0.5) {
            c1[i] = parent1[i];
            c2[i] = parent2[i];
        } else {
            c1[i] = parent2[i];
            c2[i] = parent1[i];
        }
    }

    // 順序パート：OX（順序交叉）
    std::uniform_int_distribution<int> dist_cut(0, MAX_CONTROLS);
    int a = dist_cut(rng);
    int b = dist_cut(rng);
    if (a > b) std::swap(a, b);

    auto ox_fill = [&](Chromosome&       child,
                       const Chromosome& parent_donor,
                       const Chromosome& parent_filler) {
        std::vector<char> used(MAX_CONTROLS, 0);

        for (int i = a; i < b; ++i) {
            int v         = parent_donor[N + i];
            child[N + i]  = v;
            used[v]       = 1;
        }

        int pos = b % MAX_CONTROLS;
        for (int k = 0; k < MAX_CONTROLS; ++k) {
            int v = parent_filler[N + (b + k) % MAX_CONTROLS];
            if (used[v]) continue;
            child[N + pos] = v;
            used[v]        = 1;
            pos = (pos + 1) % MAX_CONTROLS;
        }
    };

    ox_fill(c1, parent1, parent2);
    ox_fill(c2, parent2, parent1);

    return {c1, c2};
}

// ============================================================
// 突然変異
//   選択パート：各ビットを PROB_BIT の確率で反転
//   順序パート：PROB_SWAP の確率で2点をスワップ
// ============================================================
void mutate(Chromosome& chromosome, int N, RNG& rng) {
    std::uniform_real_distribution<double> ureal(0.0, 1.0);

    // ビット反転
    for (int i = 0; i < N; ++i) {
        if (ureal(rng) < PROB_BIT) {
            chromosome[i] = 1 - chromosome[i];
        }
    }

    // 順序パートのスワップ
    if (ureal(rng) < PROB_SWAP) {
        std::uniform_int_distribution<int> dist(N, N + MAX_CONTROLS - 1);
        int a = dist(rng);
        int b = dist(rng);
        while (b == a) b = dist(rng);
        std::swap(chromosome[a], chromosome[b]);
    }
}

// ============================================================
// GA メインループ
// ============================================================
// 初期個体と子個体に局所探索を適用し、事前計算表を共有する。
GAResult run_ga(
    const std::vector<Landmark>& landmarks,
    const PathCache&             path_cache,
    long long                    gate_node,
    const EvaluationTables&      tables,
    RNG&                         rng,
    int                          pop_size,
    int                          n_gen,
    const SearchOptions&         options)
{
    const int N = static_cast<int>(landmarks.size());
    // 候補が 6 か所に満たないデータでは、課題の条件（6〜12 地点）を満たすコースが
    // そもそも作れない。黙って無効解を返すより、理由を言って止める。
    if (N < MIN_CONTROLS) {
        throw std::invalid_argument(
            "候補が少なすぎます。6地点以上のコースを作れません（候補数: " +
            std::to_string(N) + "）");
    }

    // 初期個体群
    SearchContext context(tables, options);
    std::vector<Chromosome> population;
    population.reserve(pop_size);
    for (int i = 0; i < pop_size; ++i) {
        population.push_back(create_random_chromosome(N, rng));
        local_search_with_tables(population.back(), landmarks, tables, gate_node, context);
    }

    // 初期評価
    std::vector<double> fitnesses(pop_size);
    for (int i = 0; i < pop_size; ++i) {
        fitnesses[i] = evaluate(population[i], landmarks, path_cache, gate_node, &tables).fitness;
    }

    std::vector<double> best_history;
    best_history.reserve(n_gen);

    for (int gen = 1; gen <= n_gen; ++gen) {
        std::vector<Chromosome> next_pop;
        next_pop.reserve(pop_size);

        // エリート保存：最良個体を1つそのまま次世代へ
        int elite_idx = static_cast<int>(
            std::min_element(fitnesses.begin(), fitnesses.end()) - fitnesses.begin());
        next_pop.push_back(population[elite_idx]);

        // 残りは選択・交叉・突然変異で生成
        while (static_cast<int>(next_pop.size()) < pop_size) {
            const Chromosome& p1 = tournament_select(population, fitnesses, rng);
            const Chromosome& p2 = tournament_select(population, fitnesses, rng);
            auto children = crossover(p1, p2, N, rng);
            mutate(children.first,  N, rng);
            mutate(children.second, N, rng);
            local_search_with_tables(children.first, landmarks, tables, gate_node, context);
            local_search_with_tables(children.second, landmarks, tables, gate_node, context);
            next_pop.push_back(std::move(children.first));
            if (static_cast<int>(next_pop.size()) < pop_size) {
                next_pop.push_back(std::move(children.second));
            }
        }

        population = std::move(next_pop);

        // 新世代を評価
        for (int i = 0; i < pop_size; ++i) {
            fitnesses[i] = evaluate(population[i], landmarks, path_cache, gate_node, &tables).fitness;
        }

        double best = *std::min_element(fitnesses.begin(), fitnesses.end());
        best_history.push_back(best);

        if (!options.quiet)
            std::cout << "  [世代 " << gen << "]  best_fitness = " << best << std::endl;
    }

    int best_idx = static_cast<int>(
        std::min_element(fitnesses.begin(), fitnesses.end()) - fitnesses.begin());

    GAResult result;
    result.best_chromosome      = population[best_idx];
    result.best_eval            = evaluate(result.best_chromosome, landmarks, path_cache, gate_node, &tables);
    result.best_fitness_history = std::move(best_history);
    return result;
}

} // namespace orienteering
