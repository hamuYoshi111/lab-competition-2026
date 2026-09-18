// 初期個体の生成と GA の世代交代を実装し、局所探索で各個体のコースを改善する。
#include "ga.h"
#include "const.h"

#include <algorithm>
#include <chrono>
#include <iostream>
#include <limits>
#include <map>
#include <numeric>
#include <stdexcept>

namespace orienteering {

// 追加: 探索済みコースと近い候補の順序を1回のGA実行だけで共有する。同じ巡回順なら選択集合も同じになる。
struct SearchContext {
    SearchOptions options;
    std::map<std::vector<int>, std::vector<int>> completed;
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

// 追加: 地点の追加・置換・削除と順序の変更を試し、改善がなくなるまで局所探索する。
static void local_search_with_tables(Chromosome& chromosome, const std::vector<Landmark>& landmarks,
                  const EvaluationTables& tables, long long gate_node, SearchContext& context)
{
    const auto decoded = decode(chromosome, landmarks, gate_node);
    // 制約違反の子は配布GAと同じペナルティで扱う。修復は別の変更になるため行わない。
    if (!decoded.is_valid) return;
    auto course = decoded.selected_indices;
    if (context.options.dedupe) {
        const auto found = context.completed.find(course);
        if (found != context.completed.end()) {
            // 探索を省いても、通常と同じ正規化済み染色体に書き戻す。
            chromosome = encode_course(found->second, static_cast<int>(landmarks.size()));
            return;
        }
    }
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
        // 09 の検討: 近接罰点を差分で更新する案を試したが、足し引きの順が変わって
        // 評価値が最後の1ビットで動くため見送った（詳細は ../docs/09-review-fixes.md）。
        auto consider = [&](const std::vector<int>& candidate) {
            const double score = evaluate_course(candidate, tables);
            // 同点移動は行わず、列挙順で最初に見つかった最良手を残す。
            if (score < best) {
                best = score;
                best_course = candidate;
                return context.options.first_improvement;
            }
            return false;
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
            // 追加：正門の直後から直前まで、すべての挿入位置を調べる。
            if (n < MAX_CONTROLS) {
                for (int pos = 0; pos <= n; ++pos) {
                    if (context.options.candidate_k > 0 && !insert_allowed[pos][point]) continue;
                    candidate = course;
                    candidate.insert(candidate.begin() + pos, point);
                    if (consider(candidate)) goto accept_move;
                }
            }
            // 置換：順序上の位置を保ち、地点だけを交換する。
            for (int pos = 0; pos < n; ++pos) {
                if (context.options.candidate_k > 0 && !replace_allowed[pos][point]) continue;
                candidate = course;
                candidate[pos] = point;
                if (consider(candidate)) goto accept_move;
            }
        }
        for (int i = 0; i < n; ++i) {
            if (n > MIN_CONTROLS) {
                candidate = course;
                candidate.erase(candidate.begin() + i);
                if (consider(candidate)) goto accept_move;
            }
            // 2-opt：正門を固定し、選択地点の連続区間を反転する。
            for (int j = i + 1; j < n; ++j) {
                candidate = course;
                std::reverse(candidate.begin() + i, candidate.begin() + j + 1);
                if (consider(candidate)) goto accept_move;
            }
            // Or-opt：1地点を抜き取り、短くなったリストの各位置に挿入する。
            for (int j = 0; j < n; ++j) {
                if (i == j) continue;
                candidate = course;
                candidate.erase(candidate.begin() + i);
                candidate.insert(candidate.begin() + j, course[i]);
                if (consider(candidate)) goto accept_move;
            }
        }
        // 多重ループを抜け、最初の改善手をすぐに採用する。
accept_move:
        if (!(best < current)) break;
        course = best_course;
        current = best;
    }
    if (context.options.dedupe) {
        context.completed[decoded.selected_indices] = course;
        context.completed[course] = course;
    }
    chromosome = encode_course(course, static_cast<int>(landmarks.size()));
}

// 追加: 事前計算表を用意し、候補を絞らない全近傍の局所探索を提供する。
void local_search(Chromosome& chromosome, const std::vector<Landmark>& landmarks,
                  const PathCache& path_cache, long long gate_node)
{
    const EvaluationTables tables(landmarks, path_cache, gate_node);
    // 公開の局所探索は従来どおり全近傍を調べる。既存の局所最適性テストもこの経路。
    SearchContext context(tables, SearchOptions{});
    local_search_with_tables(chromosome, landmarks, tables, gate_node, context);
}

// ============================================================
// 変更: ランダムな染色体の選択数を候補数以下に抑え、配列の範囲外への書き込みを防ぐ。
// ============================================================
Chromosome create_random_chromosome(int N, RNG& rng) {
    // 06 で足した安全弁：候補が MAX_CONTROLS（12）より少ないデータでは、
    // 「12 地点を選ぶ」と N 個しかない配列の外に書き込んで異常終了していた。
    // 選ぶ数の上限を候補数までに抑える。候補が 12 以上あるときの動きは変わらない。
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
// 追加: 目標に近い初期解を貪欲構築し、上位候補からの乱択で多様性を持たせる。
//
// 配布コードの初期個体は「6〜12地点をでたらめに選び、でたらめな順で回る」なので、
// 目標（8地点・60分・登り50m以内・地点どうし150m以上）からかけ離れたところから
// 始まる。ここでは最初から目標に近いコースを組み立てておき、あとの局所探索と
// 世代交代の仕事を減らすことを狙う。
//
// 手順
//   1. 正門だけの空のコースから始める
//   2. まだ選んでいない地点のうち、
//        ・選択済みのどれとも150m以上離れている（近すぎの罰点が出ない）
//        ・入れたあとの推定時間が60分を超えない
//      ものを候補にする
//   3. 候補ごとに「どこに差し込むと道のりの増え方がいちばん小さいか」を調べる
//   4. 増え方の小さい順に並べ、上位数件から乱数で1つ選んで差し込む
//   5. 8地点になるか、候補が尽きたら終了
//
// 4 で必ず1位を選ぶと全個体が同じコースになってしまうため、少しだけ運を混ぜる。
// ============================================================
namespace {

// 変更(09): 推定所要時間の式は evaluate.h の estimated_minutes に集約した。

struct GreedyMove {
    double increase;  // 差し込みで増える道のり（m）
    int    point;     // 足す地点の候補番号
    int    position;  // 巡回順のどこに差し込むか
    double distance;  // 差し込んだあとのコース全体の道のり
    double gain;      // 同じく累積登り
};

} // namespace

Chromosome create_greedy_chromosome(int N, RNG& rng, const EvaluationTables& tables) {
    const size_t gate = tables.gate_index();
    std::vector<int> course;
    course.reserve(MAX_CONTROLS);
    std::vector<unsigned char> chosen(N, 0);

    // 空のコース（正門 → 正門）の道のりと登り。
    const PathInfo& empty = tables.path(gate, gate);
    double distance = empty.reachable ? empty.length : 0.0;
    double gain     = empty.reachable ? empty.gain   : 0.0;

    std::vector<GreedyMove> moves;
    moves.reserve(N);

    // relax_time を true にすると 60 分の条件を外す。地点数が最低数に届かないときの逃げ道。
    auto collect = [&](bool relax_time, bool relax_proximity) {
        moves.clear();
        const int n = static_cast<int>(course.size());
        for (int point = 0; point < N; ++point) {
            if (chosen[point]) continue;
            if (!relax_proximity) {
                bool too_close = false;
                for (int other : course) {
                    if (tables.proximity_pair(point, other) > 0.0) { too_close = true; break; }
                }
                if (too_close) continue;
            }
            // 差し込む位置ごとに、増える道のりを調べていちばん小さいところを選ぶ。
            GreedyMove best{0.0, point, -1, 0.0, 0.0};
            for (int pos = 0; pos <= n; ++pos) {
                const size_t prev = pos == 0 ? gate : static_cast<size_t>(course[pos - 1]);
                const size_t next = pos == n ? gate : static_cast<size_t>(course[pos]);
                const PathInfo& in  = tables.path(prev, static_cast<size_t>(point));
                const PathInfo& out = tables.path(static_cast<size_t>(point), next);
                const PathInfo& cut = tables.path(prev, next);
                if (!in.reachable || !out.reachable || !cut.reachable) continue;
                const double add_distance = in.length + out.length - cut.length;
                const double add_gain     = in.gain   + out.gain   - cut.gain;
                if (best.position < 0 || add_distance < best.increase) {
                    best = {add_distance, point, pos,
                            distance + add_distance, gain + add_gain};
                }
            }
            if (best.position < 0) continue;
            if (!relax_time && estimated_minutes(best.distance, best.gain) > T_TARGET) continue;
            moves.push_back(best);
        }
        // 同じ増え方なら候補番号の小さい順。乱数以外で結果がぶれないようにする。
        std::sort(moves.begin(), moves.end(), [](const GreedyMove& a, const GreedyMove& b) {
            return a.increase < b.increase || (a.increase == b.increase && a.point < b.point);
        });
    };

    while (static_cast<int>(course.size()) < Q_TARGET) {
        collect(false, false);
        if (moves.empty()) break;
        const int top = std::min(static_cast<int>(moves.size()), GREEDY_TOP_CHOICES);
        std::uniform_int_distribution<int> pick(0, top - 1);
        const GreedyMove& move = moves[pick(rng)];
        course.insert(course.begin() + move.position, move.point);
        chosen[move.point] = 1;
        distance = move.distance;
        gain     = move.gain;
    }

    // 6地点に届かないと配布コードでは無効解になるので、条件を順に緩めて埋める。
    for (int stage = 0; stage < 2 && static_cast<int>(course.size()) < MIN_CONTROLS; ++stage) {
        while (static_cast<int>(course.size()) < MIN_CONTROLS) {
            collect(true, stage == 1);
            if (moves.empty()) break;
            const GreedyMove& move = moves.front();
            course.insert(course.begin() + move.position, move.point);
            chosen[move.point] = 1;
            distance = move.distance;
            gain     = move.gain;
        }
    }

    return encode_course(course, N);
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
// 変更: 初期個体と子個体に局所探索を適用し、事前計算表の共有と処理時間の計測を行う。
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
    const auto initialization_start = std::chrono::steady_clock::now();
    SearchContext context(tables, options);
    std::vector<Chromosome> population;
    population.reserve(pop_size);
    for (int i = 0; i < pop_size; ++i) {
        // GreedyHalf では前半だけ貪欲構築にし、後半はランダムのまま残して散らばりを保つ。
        const bool use_greedy =
            options.init == InitMethod::GreedyAll ||
            (options.init == InitMethod::GreedyHalf && i < pop_size / 2);
        population.push_back(use_greedy ? create_greedy_chromosome(N, rng, tables)
                                        : create_random_chromosome(N, rng));
        if (USE_LOCAL_SEARCH) local_search_with_tables(population.back(), landmarks, tables, gate_node, context);
    }

    // 初期評価
    std::vector<double> fitnesses(pop_size);
    for (int i = 0; i < pop_size; ++i) {
        fitnesses[i] = evaluate(population[i], landmarks, path_cache, gate_node, &tables).fitness;
    }

    std::vector<double> best_history;
    best_history.reserve(n_gen);
    const double initial_best = *std::min_element(fitnesses.begin(), fitnesses.end());

    const auto generations_start = std::chrono::steady_clock::now();
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
            if (USE_LOCAL_SEARCH) {
                local_search_with_tables(children.first, landmarks, tables, gate_node, context);
                local_search_with_tables(children.second, landmarks, tables, gate_node, context);
            }
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

    const auto generations_end = std::chrono::steady_clock::now();
    int best_idx = static_cast<int>(
        std::min_element(fitnesses.begin(), fitnesses.end()) - fitnesses.begin());

    GAResult result;
    result.best_chromosome      = population[best_idx];
    result.best_eval            = evaluate(result.best_chromosome, landmarks, path_cache, gate_node, &tables);
    result.best_fitness_history = std::move(best_history);
    result.initial_best_fitness = initial_best;
    result.initialization_seconds = std::chrono::duration<double>(generations_start - initialization_start).count();
    result.generations_seconds = std::chrono::duration<double>(generations_end - generations_start).count();
    return result;
}

} // namespace orienteering
