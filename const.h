// 課題の制約・評価式の定数と、GA・局所探索・再スタートの既定値を定義する。
#ifndef CONST_H
#define CONST_H

#include <cfloat>
#include <string>

namespace orienteering {

// ============================================================
// ファイル・ディレクトリ
// ============================================================
inline const std::string DATA_DIR   = "input";    // 入力データのフォルダ
inline const std::string OUTPUT_DIR = "output";   // 出力ファイルのフォルダ

// ============================================================
// コントロール数の制約（コンペ課題で指定）
// ============================================================
constexpr int MIN_CONTROLS = 6;
constexpr int MAX_CONTROLS = 12;

// ============================================================
// 目的関数パラメータ（コンペ課題 表1）
// ============================================================
constexpr int    Q_TARGET     = 8;        // 目標コントロール数
constexpr double D_MIN        = 150.0;    // 近すぎる距離の閾値（m）
constexpr double T_TARGET     = 60.0;     // 目標所要時間（分）
constexpr double WALK_SPEED   = 4020.0;   // 平地の歩行速度（m/時）
constexpr double CLIMB_SPEED  = 300.0;    // 登り坂の速度換算値（m/時）
constexpr double ROUTE_TARGET = 50.0;     // 累積登り高低差の許容値（m）
constexpr double PENALTY      = DBL_MAX;   // 無効解へのペナルティ（実値では到達し得ないセンチネル）

// ============================================================
// 重み設定（4指標を重み付き和で集約。合計 1.0）
// ============================================================
constexpr double W_MAP   = 0.50;
constexpr double W_DIST  = 0.10;
constexpr double W_TIME  = 0.25;
constexpr double W_ROUTE = 0.15;

// ============================================================
// GA のハイパーパラメータ
// ============================================================
// 実行時引数（orienteering.exe 種 個体数 世代数 ...）で上書きできる。
constexpr int    POP_SIZE        = 10;    // 各再スタートの個体数
constexpr int    N_GEN           = 20;    // 各再スタートの世代数

// 局所探索の追加・置換候補を、前後それぞれに近い K 地点へ絞る既定値。
// 0 にすると全候補を試す。
// 実行時は --candidate-k K で上書きできる。
constexpr int    DEFAULT_CANDIDATE_K = 40;
// 世代ごとの1行表示を既定で省く。--verbose を付けると表示に戻る。
constexpr bool   DEFAULT_QUIET       = true;
// 独立再スタートの既定回数。1 を指定すると GA を1回だけ走らせる。
// 開始の種 s（省略時は RANDOM_SEED）から s, s+1, …, s+R-1 で R 回走らせ、最良の解を採る。
// 実行時は --restarts R で上書きできる。
constexpr int    DEFAULT_RESTARTS    = 3;
constexpr double PROB_BIT        = 0.02;  // 選択パートのビット反転確率
constexpr double PROB_SWAP       = 0.10;  // 順序パートのスワップ確率
constexpr int    TOURNAMENT_SIZE = 5;     // トーナメント選択のサイズ

constexpr unsigned int RANDOM_SEED = 42;

// ============================================================
// 物理定数
// ============================================================
constexpr double PI                = 3.14159265358979323846;
constexpr double METERS_PER_DEGREE = 111320.0;  // 緯度1度あたりのメートル

} // namespace orienteering

#endif // CONST_H
