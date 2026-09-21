// 道路グラフと最短経路の距離・累積登りを保持する型、探索・事前計算の関数を定義する。
#ifndef GRAPH_H
#define GRAPH_H

#include "csv_loader.h"

#include <unordered_map>
#include <utility>
#include <vector>

namespace orienteering {

// 1本のエッジ情報
struct EdgeInfo {
    long long to;
    double    length;
    double    elevation_gain;
};

// 最短路の集計結果（距離と累積登り）
struct PathInfo {
    double length;        // 距離（m）
    double gain;          // 累積登り（m）
    bool   reachable;     // 到達可能か
};

// ============================================================
// 道路ネットワーク（有向グラフ）
// ============================================================
class Graph {
public:
    Graph(const std::vector<Node>& nodes, const std::vector<Edge>& edges);

    // src から到達可能な全ノードに対し、（最短距離・その経路上の累積登り）を計算
    // 戻り値：node_id → PathInfo の辞書
    std::unordered_map<long long, PathInfo> dijkstra_from(long long src) const;

    // ゲートの緯度経度に最も近いノードIDを返す
    long long find_nearest_node(double lat, double lon) const;

private:
    friend class PathCache;
    // ノード ID を連番に変換した辺を持ち、配列で探索する。
    struct IndexedEdge {
        size_t to;
        double length;
        double first_gain;
    };
    // 優先度つきキューに入れる (距離, ノード番号)。取り出し順は変えていない。
    using State = std::pair<double, size_t>;

    // 探索用の領域は使い回し、世代番号 epoch で今回の探索で触ったかを判定する。
    // 1ノード分の作業用データをひとまとめにする。ばらばらの配列に分けると
    // 1回の更新でメモリの離れた4か所を触ることになり、キャッシュに乗りにくい。
    struct Visit {
        double       dist;
        double       parent_gain;
        size_t       parent;
        unsigned int stamp;         // 何回目の探索で到達したか
        // 今回の探索で「まだ確定していない目的地」の印。
        // stamp のうしろの余り（詰め物）に入るので、1件あたりの大きさは変わらない。
        unsigned int target_stamp;
    };
    struct Scratch {
        std::vector<Visit> visit;
        std::vector<State> heap;   // std::priority_queue と同じ並べ方
        unsigned int       epoch = 0;
    };

    // ID順に連番を付け、同距離の取り出し順も元のID順に保つ。
    std::vector<long long> node_ids_;
    std::unordered_map<long long, size_t> node_indices_;
    std::vector<std::vector<IndexedEdge>> adj_;
    // 隣接リストは1本の配列にまとめ、始点ごとの区切りを adj_begin_ に持つ（省メモリ・高速）。
    std::vector<IndexedEdge> adj_flat_;
    std::vector<size_t> adj_begin_;
    mutable Scratch scratch_;  // 中身は使い捨ての作業領域なので const 関数から使える
    std::vector<PathInfo> dijkstra_to(long long src, const std::vector<long long>& targets) const;
    // 始点・終点をノード番号で受け取る版。表の事前計算はこちらを使う。
    void dijkstra_indexed(size_t source, const std::vector<size_t>& target_indices,
                          std::vector<PathInfo>& out) const;
    std::vector<Node> node_list_;  // 最近傍検索用
};

// ============================================================
// 最短経路キャッシュ
// 興味のあるノード集合 sources × sources の (距離, 登り) を事前計算
// ============================================================
class PathCache {
public:
    // sourcesの順番を行・列に使う。通常は全候補の後ろに正門を置く。
    PathCache(const Graph& graph, const std::vector<long long>& sources);

    // 表は (行数 × 列数) の1本の配列に持つ。行ごとの vector より読み出しが速い。
    const PathInfo& at(size_t src_index, size_t dst_index) const {
        return cache_[src_index * size_ + dst_index];
    }
    size_t size() const { return size_; }

    // 保存対象はsources同士のみ。対象外や到達不能な区間はペナルティを返す。
    PathInfo get(long long src, long long dst) const;

private:
    std::unordered_map<long long, size_t> source_indices_;
    std::vector<PathInfo> cache_;
    size_t size_ = 0;
};

} // namespace orienteering

#endif // GRAPH_H
