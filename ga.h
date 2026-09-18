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

// 追加: 初期個体の作り方。Random が配布コードと同じ「地点も順番も出たとこ勝負」。
enum class InitMethod {
    Random,      // 6〜12地点をランダムに選び、ランダムな順で回る
    GreedyAll,   // 全個体を貪欲構築で作る
    GreedyHalf,  // 半数を貪欲構築、残り半数はランダム（多様性を残す）
};

// 追加: 比較用の探索設定。変更はすべて既定で無効。候補数0は全候補を調べる。
// ここは「ライブラリとしての既定」で、実行ファイルの既定は const.h と main.cpp が決める。
struct SearchOptions {
    bool dedupe = false;
    bool first_improvement = false;
    int candidate_k = 0;
    bool quiet = false;
    InitMethod init = InitMethod::Random;
};

// 追加: 全近傍で最も評価値が下がる手を選び、改善がなくなるまで続ける。
void local_search(Chromosome& chromosome, const std::vector<Landmark>& landmarks,
                  const PathCache& path_cache, long long gate_node);

// ============================================================
// GA 操作（個体生成・選択・交叉・突然変異）
// ============================================================

// ランダムな染色体を生成（初期個体群用）
Chromosome create_random_chromosome(int N, RNG& rng);

// 追加: 目標に近いコースを貪欲に組み立てる（初期個体群用、--init greedy-* で使う）
Chromosome create_greedy_chromosome(int N, RNG& rng, const EvaluationTables& tables);

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

// 変更: 最良解と履歴に加え、処理時間の内訳と初期集団の最良評価値を返す。
struct GAResult {
    Chromosome           best_chromosome;
    EvalResult           best_eval;
    std::vector<double>  best_fitness_history;
    double initialization_seconds = 0.0;
    double generations_seconds = 0.0;
    // 世代交代を始める前（初期集団に局所探索をかけた直後）の最良評価値。
    // 初期解そのものの良し悪しを見るために記録する。
    double initial_best_fitness = 0.0;
};

// 変更: 事前計算表・個体数・世代数・探索設定を呼び出し側から受け取る。
// 変更(09): 個体数・世代数の既定値はここに書かない。実行ファイルの既定は
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
