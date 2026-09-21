// 道路グラフを構築し、最短経路の探索と候補地点間の距離・累積登りの事前計算を行う。
#include "graph.h"
#include "const.h"

#include <algorithm>
#include <functional>
#include <limits>
#include <utility>

namespace orienteering {

// ============================================================
// Graph 実装
// ============================================================

// ノードを連番化し、隣接辺を連続配置して探索用の作業領域を確保する。
Graph::Graph(const std::vector<Node>& nodes, const std::vector<Edge>& edges) {
    node_list_ = nodes;
    node_ids_.reserve(nodes.size() + edges.size() * 2);
    for (const auto& n : nodes) node_ids_.push_back(n.node_id);
    for (const auto& e : edges) {
        node_ids_.push_back(e.from_node);
        node_ids_.push_back(e.to_node);
    }
    std::sort(node_ids_.begin(), node_ids_.end());
    node_ids_.erase(std::unique(node_ids_.begin(), node_ids_.end()), node_ids_.end());
    node_indices_.reserve(node_ids_.size() * 2);
    for (size_t i = 0; i < node_ids_.size(); ++i) node_indices_[node_ids_[i]] = i;
    adj_.resize(node_ids_.size());
    for (const auto& e : edges) {
        auto& outgoing = adj_[node_indices_.at(e.from_node)];
        const size_t to = node_indices_.at(e.to_node);
        // 並行辺では最短の辺ではなく、入力順で最初の辺の登りを使う。
        double first_gain = e.elevation_gain;
        for (const auto& previous : outgoing) {
            if (previous.to == to) { first_gain = previous.first_gain; break; }
        }
        outgoing.push_back({to, e.length_m, first_gain});
    }
    // 隣接リストを1本の配列に並べ直す。中身と並び順はそのままで、
    // 探索中に飛び飛びのメモリをたどらずに済むようにするだけ。
    adj_begin_.resize(adj_.size() + 1, 0);
    size_t total = 0;
    for (size_t i = 0; i < adj_.size(); ++i) {
        adj_begin_[i] = total;
        total += adj_[i].size();
    }
    adj_begin_[adj_.size()] = total;
    adj_flat_.reserve(total);
    for (const auto& outgoing : adj_) {
        adj_flat_.insert(adj_flat_.end(), outgoing.begin(), outgoing.end());
    }
    std::vector<std::vector<IndexedEdge>>().swap(adj_);  // 使い終わったので解放する

    // 探索用の作業領域も1回だけ確保しておく。
    const size_t n = node_ids_.size();
    scratch_.visit.assign(n, Visit{0.0, 0.0, n, 0u, 0u});
    scratch_.heap.reserve(total + 1);
}

long long Graph::find_nearest_node(double lat, double lon) const {
    long long best_id = -1;
    double best_dist2 = std::numeric_limits<double>::max();
    for (const auto& n : node_list_) {
        double dlat = n.lat - lat;
        double dlon = n.lon - lon;
        double d2   = dlat * dlat + dlon * dlon;
        if (d2 < best_dist2) {
            best_dist2 = d2;
            best_id    = n.node_id;
        }
    }
    return best_id;
}

// ============================================================
// 連番によるダイクストラ法で作業領域を再利用し、対象の終点がすべて確定したら打ち切る。
//
// 変えていないこと：取り出す順（同じ距離なら同じ並び）、距離の足す向き、
// 親のたどり方、登りの足し算の順。したがって結果は1ビットも変わらない。
// 変えたこと：毎回 2267 要素の配列を作り直して 0 で埋めるのをやめ、
// 「今回の探索で触ったか」を世代番号 epoch で見分ける。山（ヒープ）も使い回す。
// ============================================================
void Graph::dijkstra_indexed(size_t source, const std::vector<size_t>& target_indices,
                             std::vector<PathInfo>& out) const {
    out.assign(target_indices.size(), {PENALTY, PENALTY, false});
    const size_t missing = node_ids_.size();
    if (source >= missing) return;

    Scratch& s = scratch_;
    // epoch が一周する前に、印を全部消してやり直す（実際にはまず起きない）。
    if (s.epoch == std::numeric_limits<unsigned int>::max()) {
        for (auto& v : s.visit) { v.stamp = 0u; v.target_stamp = 0u; }
        s.epoch = 0;
    }
    const unsigned int now = ++s.epoch;

    Visit* const visit = s.visit.data();

    // 早期打ち切りのため、目的地（正門＋候補の最寄りノード）に印を付け、
    // 全部が「確定」した時点で探索をやめる。ダイクストラ法は距離の短い順に
    // 確定させるので、確定済みの距離と親はその後どれだけ探しても変わらない。
    // したがって、ここで止めても答えは1ビットも変わらない。
    size_t remaining = 0;
    for (const size_t index : target_indices) {
        if (index >= missing) continue;
        if (visit[index].target_stamp != now) {
            visit[index].target_stamp = now;
            ++remaining;
        }
    }
    if (remaining == 0) return;  // 行き先が1つも無いので探索そのものが要らない

    visit[source].stamp  = now;
    visit[source].dist   = 0.0;
    visit[source].parent = missing;

    auto& heap = s.heap;
    heap.clear();

    // 山（ヒープ）に入る (距離, ノード番号) は、同じ組が2度入ることがない。
    // 同じノードを積み直すのは距離が厳密に短くなったときだけだからである。
    // したがって「取り出す順」は中の並べ方に関係なく一意に決まり、
    // std::priority_queue と同じ順で出てくる。ここでは取り出しを1回の
    // 下りだけで済ませる作りに替えて、比較と入れ替えの回数を減らしている。
    auto before = [](const State& a, const State& b) {
        return a.first < b.first || (a.first == b.first && a.second < b.second);
    };
    auto heap_push = [&](State value) {
        size_t i = heap.size();
        heap.push_back(value);
        while (i > 0) {
            const size_t parent_pos = (i - 1) / 2;
            if (!before(value, heap[parent_pos])) break;
            heap[i] = heap[parent_pos];
            i = parent_pos;
        }
        heap[i] = value;
    };
    auto heap_pop = [&]() {
        const State last = heap.back();
        heap.pop_back();
        const size_t n = heap.size();
        if (n == 0) return;
        size_t i = 0;
        for (;;) {
            size_t child = 2 * i + 1;
            if (child >= n) break;
            if (child + 1 < n && before(heap[child + 1], heap[child])) ++child;
            if (!before(heap[child], last)) break;
            heap[i] = heap[child];
            i = child;
        }
        heap[i] = last;
    };

    heap_push({0.0, source});
    while (!heap.empty()) {
        const State top = heap.front();
        heap_pop();
        const double d = top.first;
        const size_t u = top.second;
        if (d > visit[u].dist) continue;
        // ここに来た u は「確定」。目的地を全部拾い終わったら、残りは見なくてよい。
        if (visit[u].target_stamp == now) {
            visit[u].target_stamp = 0u;  // 同じノードを二度数えない
            if (--remaining == 0) break;
        }
        for (size_t k = adj_begin_[u], end = adj_begin_[u + 1]; k < end; ++k) {
            const IndexedEdge& e = adj_flat_[k];
            const double nd = d + e.length;
            Visit& next = visit[e.to];
            if (next.stamp != now || nd < next.dist) {
                next.stamp       = now;
                next.dist        = nd;
                next.parent      = u;
                next.parent_gain = e.first_gain;
                heap_push({nd, e.to});
            }
        }
    }
    // 足す向きを変えると丸め誤差が変わるので、必ず終点から親へたどる。
    // キャッシュでは必要な終点だけ集計し、全道路ノードの経路復元を省く。
    for (size_t i = 0; i < target_indices.size(); ++i) {
        const size_t target = target_indices[i];
        if (target >= missing || visit[target].stamp != now) continue;
        double gain = 0.0;
        for (size_t cur = target;
             visit[cur].parent != missing && node_ids_[visit[cur].parent] != -1;
             cur = visit[cur].parent) {
            gain += visit[cur].parent_gain;
        }
        out[i] = {visit[target].dist, gain, true};
    }
}

// ノード ID を連番に変換して、指定された終点への経路を取得する。
std::vector<PathInfo> Graph::dijkstra_to(long long src, const std::vector<long long>& targets) const {
    const auto source = node_indices_.find(src);
    if (source == node_indices_.end()) {
        std::vector<PathInfo> result(targets.size(), {PENALTY, PENALTY, false});
        for (size_t i = 0; i < targets.size(); ++i) {
            if (targets[i] == src) result[i] = {0.0, 0.0, true};
        }
        return result;
    }
    const size_t missing = node_ids_.size();
    std::vector<size_t> target_indices(targets.size(), missing);
    for (size_t i = 0; i < targets.size(); ++i) {
        const auto found = node_indices_.find(targets[i]);
        if (found != node_indices_.end()) target_indices[i] = found->second;
    }
    std::vector<PathInfo> result;
    dijkstra_indexed(source->second, target_indices, result);
    return result;
}

// 配列による探索結果を、配布元と同じノード ID の辞書で返す。
std::unordered_map<long long, PathInfo> Graph::dijkstra_from(long long src) const {
    const auto paths = dijkstra_to(src, node_ids_);
    std::unordered_map<long long, PathInfo> result;
    for (size_t i = 0; i < paths.size(); ++i) {
        if (paths[i].reachable) result[node_ids_[i]] = paths[i];
    }
    // 元の公開関数は、未知の始点も距離0で返す。
    if (node_indices_.find(src) == node_indices_.end()) result[src] = {0.0, 0.0, true};
    return result;
}

// ============================================================
// PathCache 実装
// ============================================================

// 候補間の経路を連続した表に保存し、重複する始点は計算済みの行を再利用する。
PathCache::PathCache(const Graph& graph, const std::vector<long long>& sources) {
    size_ = sources.size();
    cache_.assign(size_ * size_, {PENALTY, PENALTY, false});
    source_indices_.reserve(size_ * 2);
    for (size_t i = 0; i < size_; ++i) source_indices_[sources[i]] = i;

    // 起点・終点のノード番号は 1 回だけ引く（従来は探索のたびに辞書を引いていた）。
    const size_t missing = graph.node_ids_.size();
    std::vector<size_t> target_indices(size_, missing);
    for (size_t i = 0; i < size_; ++i) {
        const auto found = graph.node_indices_.find(sources[i]);
        if (found != graph.node_indices_.end()) target_indices[i] = found->second;
    }

    // 113 か所の候補が指す道路ノードは、重なりを除くと 91 か所しかない。
    // 同じ道路ノードから2度探索しても結果は同じなので、1度だけ計算して行を写す。
    std::unordered_map<size_t, size_t> computed;  // ノード番号 → 計算済みの行
    computed.reserve(size_ * 2);
    std::vector<PathInfo> row;
    for (size_t i = 0; i < size_; ++i) {
        if (target_indices[i] >= missing) {
            // グラフに無い起点は、従来どおり「自分自身だけ距離0」で扱う。
            row = graph.dijkstra_to(sources[i], sources);
            std::copy(row.begin(), row.end(), cache_.begin() + i * size_);
            continue;
        }
        const auto found = computed.find(target_indices[i]);
        if (found != computed.end()) {
            std::copy(cache_.begin() + found->second * size_,
                      cache_.begin() + (found->second + 1) * size_,
                      cache_.begin() + i * size_);
            continue;
        }
        graph.dijkstra_indexed(target_indices[i], target_indices, row);
        std::copy(row.begin(), row.end(), cache_.begin() + i * size_);
        computed[target_indices[i]] = i;
    }
}

// ノード ID から表の添字を引き、対象外は到達不能として返す。
PathInfo PathCache::get(long long src, long long dst) const {
    const auto from = source_indices_.find(src);
    const auto to = source_indices_.find(dst);
    if (from == source_indices_.end() || to == source_indices_.end()) {
        return {PENALTY, PENALTY, false};
    }
    return at(from->second, to->second);
}

} // namespace orienteering
