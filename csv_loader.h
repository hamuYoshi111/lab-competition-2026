// 入力 CSV の地点・道路ノード・辺・正門のデータ型と読み込み関数を宣言する。
#ifndef CSV_LOADER_H
#define CSV_LOADER_H

#include <string>
#include <vector>

namespace orienteering {

// ============================================================
// CSV から読み込むデータ構造
// ============================================================

struct Landmark {
    int          id;
    std::string  name;
    std::string  feature;
    double       lat;
    double       lon;
    long long    nearest_node;
};

struct Node {
    long long node_id;
    double    lat;
    double    lon;
    double    elevation;
};

struct Edge {
    long long from_node;
    long long to_node;
    double    length_m;
    double    elevation_change;
    double    elevation_gain;
};

// スタート・ゴール地点（正門の座標）
struct Gate {
    double lat;
    double lon;
};

// ============================================================
// CSV ロード関数
// ============================================================

std::vector<Landmark> load_landmarks(const std::string& path);
std::vector<Node>     load_nodes(const std::string& path);
std::vector<Edge>     load_edges(const std::string& path);
Gate                  load_gate(const std::string& path);

// 追加: 小数の読み取りに std::from_chars を使えた環境なら true、
// C 言語の strtod に切り替えた環境（macOS の clang や古い g++）なら false。
// 読み取れる値はどちらでも同じで、速さだけが変わる。動作確認用。
bool uses_float_from_chars();

} // namespace orienteering

#endif // CSV_LOADER_H
