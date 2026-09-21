// GA の探索設定・結果と、個体生成・交叉・突然変異・局所探索の関数を宣言する。
#ifndef GA_H
#define GA_H

#include "csv_loader.h"
#include "graph.h"
#include "evaluate.h"
#include "const.h"

#include <random>
#include <utility>
#include <vector>

namespace orienteering {

using RNG = std::mt19937;

// 局所探索の候補数と世代ごとの表示を指定する。
struct SearchOptions {
    int candidate_k = 0;
    bool quiet = false;
};

// 全近傍で最も評価値が下がる手を選び、改善がなくなるまで続ける。
void local_search(Chromosome& chromosome, const std::vector<Landmark>& landmarks,
                  const PathCache& path_cache, long long gate_node);

// ============================================================
// GA 操作（個体生成・選択・交叉・突然変異）
// ============================================================

// ランダムな染色体を生成（初期個体群用）
Chromosome create_random_chromosome(int N, RNG& rng);

// トーナメント選択
const Chromosome& tournament_select(
    const std::vector<Chromosome>& population,
    const std::vector<double>&     fitnesses,
    RNG&                           rng);

// 交叉（選択パート：一様交叉 / 順序パート：OX＝順序交叉）
std::pair<Chromosome, Chromosome> crossover(
    const Chromosome& parent1,
    const Chromosome& parent2,
    int               N,
    RNG&              rng);

// 突然変異（選択パート：ビット反転 / 順序パート：2点スワップ）
void mutate(Chromosome& chromosome, int N, RNG& rng);

// ============================================================
// GA メインループ
// ============================================================

// 最良解と、世代ごとの最良評価値の履歴を返す。
struct GAResult {
    Chromosome           best_chromosome;
    EvalResult           best_eval;
    std::vector<double>  best_fitness_history;
};

// 事前計算表・個体数・世代数・探索設定を呼び出し側から受け取る。
// 個体数・世代数の既定値はここに書かない。実行ファイルの既定は
// const.h（POP_SIZE / N_GEN）だけが持ち、main.cpp がそれを読んで渡す。
GAResult run_ga(
    const std::vector<Landmark>& landmarks,
    const PathCache&             path_cache,
    long long                    gate_node,
    const EvaluationTables&      tables,
    RNG&                         rng,
    int                          pop_size,
    int                          n_gen,
    const SearchOptions&         options = SearchOptions{});

} // namespace orienteering

#endif // GA_H
