// 染色体・コース・評価結果の型と、評価用の事前計算表・目的関数を宣言する。
#ifndef EVALUATE_H
#define EVALUATE_H

#include "csv_loader.h"
#include "graph.h"

#include <vector>

namespace orienteering {

// 染色体 = 選択パート（N bit）+ 順序パート（MAX_CONTROLS 個の順列）
using Chromosome = std::vector<int>;

// 4目的関数の値
struct Objectives {
    double f_map;
    double f_dist;
    double f_time;
    double f_route;
};

// デコード結果
struct DecodedCourse {
    std::vector<long long> course_nodes;       // [gate, c1, c2, ..., gate]
    std::vector<int>       selected_indices;   // landmarks のインデックス（巡回順）
    bool                   is_valid;
};

// 評価結果（GAで使う総合情報）
struct EvalResult {
    double         fitness;
    Objectives     objectives;
    bool           is_valid;
    double         total_distance;
    double         total_gain;
    DecodedCourse  decoded;
};

// 候補番号0～N-1と正門Nで引く表。探索中はハッシュ検索や三角関数を使わない。
class EvaluationTables {
public:
    EvaluationTables(const std::vector<Landmark>& landmarks,
                     const PathCache& path_cache, long long gate_node);
    const PathInfo& path(size_t from, size_t to) const { return paths_[from][to]; }
    size_t gate_index() const { return proximity_.size(); }
    double proximity(const std::vector<int>& course) const;
    // 2地点だけの近さの罰点。0なら150m以上離れている。賢い初期解の判定に使う。
    double proximity_pair(size_t a, size_t b) const { return proximity_[a][b]; }
private:
    std::vector<std::vector<PathInfo>> paths_;
    std::vector<std::vector<double>> proximity_;
};

// ============================================================
// デコード関数
// ============================================================
DecodedCourse decode(
    const Chromosome&            chromosome,
    const std::vector<Landmark>& landmarks,
    long long                    gate_node);

// ============================================================
// 4つの目的関数
// ============================================================
double f_map(int n_controls);

// 巡回順から染色体を作る。順序パートは候補番号順の各地点の巡回順位。
Chromosome encode_course(const std::vector<int>& course, int n_landmarks);

// 近傍探索用。デコードを省き、既存の目的関数で直接評価する。
double evaluate_course(const std::vector<int>& course,
    const std::vector<Landmark>& landmarks, const PathCache& path_cache,
    long long gate_node);

double evaluate_course(const std::vector<int>& course, const EvaluationTables& tables);

double f_dist(
    const std::vector<int>&      selected_indices,
    const std::vector<Landmark>& landmarks);

// 推定所要時間（分）。距離と登りから同じ式で求める場所を1か所にまとめる。
// 注意: f_time は配布コードのまま「(距離/速さ)*60 + (登り/速さ)*60」の順で足しており、
// 丸めの回数が1回違う。値をそろえるために式を差し替えると評価値が最後の1ビットで
// 変わるため、f_time の中身はあえてこの関数に寄せていない。
double estimated_minutes(double total_distance, double total_gain);

double f_time(double total_distance, double total_gain);

double f_route(double total_gain);

// ============================================================
// 単目的化（重み付き和）
// ============================================================
double calc_fitness(const Objectives& obj);

// ============================================================
// GA からの評価では事前計算表を受け取り、表なしの呼び出しも維持する。
// ============================================================
EvalResult evaluate(
    const Chromosome&            chromosome,
    const std::vector<Landmark>& landmarks,
    const PathCache&             path_cache,
    long long                    gate_node,
    const EvaluationTables*      tables = nullptr);

} // namespace orienteering

#endif // EVALUATE_H
