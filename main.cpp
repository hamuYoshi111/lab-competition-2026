// ============================================================
// コマンドライン引数を解釈し、入力 CSV の読み込み・事前計算・GA 実行を統括する。
// 最良コースを output/best_course.json、評価値の履歴を output/fitness_history.csv に書き出す。
// ============================================================

#include "const.h"
#include "csv_loader.h"
#include "ga.h"
#include "graph.h"
#include "evaluate.h"

#include <chrono>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <random>
#include <string>
#include <vector>

#ifdef _WIN32
#include <windows.h>  // Windows のみ：コンソールを UTF-8 表示にするため
#endif

using namespace orienteering;

namespace {

// CSV 由来の文字列を JSON の文字列として安全に書ける形に直す。
// 逆斜線・二重引用符・制御文字（0x00〜0x1F）を JSON の決まりどおりに置き換える。
// これを通さないと、地点名に二重引用符が1つあるだけで JSON が壊れ、
// 後ろに別の "fitness" を割り込ませることもできてしまう。
std::string json_escape(const std::string& text) {
    std::string out;
    out.reserve(text.size());
    for (unsigned char c : text) {
        switch (c) {
            case '\\': out += "\\\\"; break;
            case '"':  out += "\\\""; break;
            case '\b': out += "\\b";  break;
            case '\f': out += "\\f";  break;
            case '\n': out += "\\n";  break;
            case '\r': out += "\\r";  break;
            case '\t': out += "\\t";  break;
            default:
                if (c < 0x20) {
                    char buf[7];
                    std::snprintf(buf, sizeof(buf), "\\u%04X", static_cast<unsigned int>(c));
                    out += buf;
                } else {
                    out += static_cast<char>(c);
                }
        }
    }
    return out;
}

// 結果を JSON 形式で書き出す（Python版 best_course.json と同じ構造）
void write_best_course_json(
    const std::string&            path,
    const GAResult&               result,
    const std::vector<Landmark>&  landmarks)
{
    const auto& ev = result.best_eval;
    const auto& d  = ev.decoded;
    // 推定所要時間の式は evaluate.h の estimated_minutes に集約した。
    double t_estimated = estimated_minutes(ev.total_distance, ev.total_gain);

    std::ofstream ofs(path);
    if (!ofs.is_open()) {
        throw std::runtime_error("JSON ファイルが書き込めません: " + path);
    }

    // 大きな数値も途中で切れない容量を確保する。
    auto fmt = [](double v, int p) {
        char buf[400];
        std::snprintf(buf, sizeof(buf), "%.*f", p, v);
        return std::string(buf);
    };

    ofs << "{\n";
    ofs << "  \"n_controls\": "         << d.selected_indices.size() << ",\n";
    ofs << "  \"total_distance_m\": "   << fmt(ev.total_distance, 1) << ",\n";
    ofs << "  \"total_gain_m\": "       << fmt(ev.total_gain, 1)     << ",\n";
    ofs << "  \"estimated_time_min\": " << fmt(t_estimated, 1)       << ",\n";
    ofs << "  \"fitness\": "            << fmt(ev.fitness, 6)        << ",\n";

    ofs << "  \"objectives\": {\n";
    ofs << "    \"f_map\": "   << fmt(ev.objectives.f_map,   3) << ",\n";
    ofs << "    \"f_dist\": "  << fmt(ev.objectives.f_dist,  3) << ",\n";
    ofs << "    \"f_time\": "  << fmt(ev.objectives.f_time,  3) << ",\n";
    ofs << "    \"f_route\": " << fmt(ev.objectives.f_route, 3) << "\n";
    ofs << "  },\n";

    ofs << "  \"weights\": {\n";
    ofs << "    \"f_map\": "   << fmt(W_MAP,   2) << ",\n";
    ofs << "    \"f_dist\": "  << fmt(W_DIST,  2) << ",\n";
    ofs << "    \"f_time\": "  << fmt(W_TIME,  2) << ",\n";
    ofs << "    \"f_route\": " << fmt(W_ROUTE, 2) << "\n";
    ofs << "  },\n";

    ofs << "  \"controls\": [\n";
    for (size_t i = 0; i < d.selected_indices.size(); ++i) {
        const auto& lm = landmarks[d.selected_indices[i]];
        ofs << "    {";
        // 文字列は必ず JSON エスケープを通してから書く。
        ofs << "\"name\": \""    << json_escape(lm.name)    << "\", ";
        ofs << "\"feature\": \"" << json_escape(lm.feature) << "\", ";
        ofs << "\"lat\": "       << fmt(lm.lat, 7)          << ", ";
        ofs << "\"lon\": "       << fmt(lm.lon, 7);
        ofs << "}";
        if (i + 1 < d.selected_indices.size()) ofs << ",";
        ofs << "\n";
    }
    ofs << "  ],\n";

    ofs << "  \"course_nodes\": [";
    for (size_t i = 0; i < d.course_nodes.size(); ++i) {
        ofs << d.course_nodes[i];
        if (i + 1 < d.course_nodes.size()) ofs << ", ";
    }
    ofs << "]\n";
    ofs << "}\n";
}

void write_fitness_history_csv(
    const std::string&         path,
    const std::vector<double>& history)
{
    std::ofstream ofs(path);
    if (!ofs.is_open()) {
        throw std::runtime_error("CSV ファイルが書き込めません: " + path);
    }
    ofs << "generation,best_fitness\n";
    ofs << std::fixed << std::setprecision(6);
    for (size_t i = 0; i < history.size(); ++i) {
        ofs << (i + 1) << "," << history[i] << "\n";
    }
}

// ============================================================
// 実行時の設定。既定値は const.h を参照する。
struct Settings {
    unsigned int  seed     = RANDOM_SEED;
    int           pop_size = POP_SIZE;
    int           n_gen    = N_GEN;
    int           restarts = DEFAULT_RESTARTS;
    SearchOptions options;
};

void print_usage(const char* program) {
    std::cerr
        << "使い方: " << program << " [種] [個体数] [世代数] [オプション...]\n"
        << "  位置引数は数字で書き、前から順に 種・個体数・世代数。省略すると既定値を使う。\n"
        << "  既定値: 種=" << RANDOM_SEED << " 個体数=" << POP_SIZE
        << " 世代数=" << N_GEN << " 再スタート=" << DEFAULT_RESTARTS
        << " 候補数=" << DEFAULT_CANDIDATE_K << "\n"
        << "  オプション:\n"
        << "    --restarts R         独立再スタートの回数（1以上）\n"
        << "    --candidate-k K      追加・置換で試す近傍地点の数（0 なら全候補）\n"
        << "    --quiet              世代ごとの表示を止める（既定）\n"
        << "    --verbose            世代ごとの表示に戻す\n"
        << "  例: " << program << " 42 10 20 --restarts 3\n";
}

// 数字だけからなる文字列を符号なし整数として読む。余分な文字があれば失敗。
bool parse_number(const std::string& text, unsigned long long& value) {
    if (text.empty() || text.size() > 20) return false;
    for (char c : text) {
        if (c < '0' || c > '9') return false;
    }
    value = 0;
    for (char c : text) {
        value = value * 10ULL + static_cast<unsigned long long>(c - '0');
    }
    return true;
}

// 解析できたら true。できなければ error に理由を入れて false を返す。
bool parse_arguments(int argc, char* argv[], Settings& settings, std::string& error) {
    settings.options.candidate_k = DEFAULT_CANDIDATE_K;
    settings.options.quiet       = DEFAULT_QUIET;

    // 値をとるオプションのために、次の引数を安全に取り出す。
    int index = 1;
    auto next_value = [&](const std::string& flag, std::string& value) {
        if (index + 1 >= argc) {
            error = "値が足りません: " + flag;
            return false;
        }
        value = argv[++index];
        return true;
    };

    int positional = 0;
    for (; index < argc; ++index) {
        const std::string arg = argv[index];
        if (arg.rfind("--", 0) == 0) {
            std::string value;
            if (arg == "--quiet") {
                settings.options.quiet = true;
            } else if (arg == "--verbose") {
                settings.options.quiet = false;
            } else if (arg == "--candidate-k") {
                if (!next_value(arg, value)) return false;
                unsigned long long number = 0;
                if (!parse_number(value, number) || number > 1000000ULL) {
                    error = "候補数は0以上の整数で指定してください: " + value;
                    return false;
                }
                settings.options.candidate_k = static_cast<int>(number);
            } else if (arg == "--restarts") {
                if (!next_value(arg, value)) return false;
                unsigned long long number = 0;
                if (!parse_number(value, number) || number < 1ULL || number > 1000000ULL) {
                    error = "再スタート回数は1以上の整数で指定してください: " + value;
                    return false;
                }
                settings.restarts = static_cast<int>(number);
            } else {
                error = "不明なオプション: " + arg;
                return false;
            }
            continue;
        }

        // 位置引数（種・個体数・世代数）。数字以外はここで弾く。
        unsigned long long number = 0;
        if (!parse_number(arg, number)) {
            error = "数字で書く引数のはずが読めません: " + arg;
            return false;
        }
        if (positional == 0) {
            if (number > 4294967295ULL) {
                error = "種は 0 以上 4294967295 以下で指定してください: " + arg;
                return false;
            }
            settings.seed = static_cast<unsigned int>(number);
        } else if (positional == 1) {
            if (number < 1ULL || number > 1000000ULL) {
                error = "個体数は1以上の整数で指定してください: " + arg;
                return false;
            }
            settings.pop_size = static_cast<int>(number);
        } else if (positional == 2) {
            if (number < 1ULL || number > 1000000ULL) {
                error = "世代数は1以上の整数で指定してください: " + arg;
                return false;
            }
            settings.n_gen = static_cast<int>(number);
        } else {
            error = "位置引数が多すぎます（種・個体数・世代数の3つまで）: " + arg;
            return false;
        }
        ++positional;
    }
    return true;
}

} // namespace

int main(int argc, char* argv[]) {
    const auto total_start = std::chrono::steady_clock::now();
#ifdef _WIN32
    // Windows コンソールを UTF-8 表示にする（日本語の文字化け防止）。
    // ソースは UTF-8 で記述しているため、出力側もコードページを UTF-8 に揃える。
    SetConsoleOutputCP(CP_UTF8);
#endif
    // 引数の解析は CSV 読み込みより前に1か所で済ませる。
    // 種 s, s+1, …, s+R-1 で GA を R 回走らせ、最良の解を採る。
    Settings settings;
    std::string argument_error;
    if (!parse_arguments(argc, argv, settings, argument_error)) {
        std::cerr << "エラー: " << argument_error << std::endl;
        print_usage(argc > 0 ? argv[0] : "orienteering");
        return 2;
    }
    const int           pop_size = settings.pop_size;
    const int           n_gen    = settings.n_gen;
    const int           restarts = settings.restarts;
    const SearchOptions options  = settings.options;

    try {
        std::cout << "データを読み込み中..." << std::endl;

        auto landmarks = load_landmarks(DATA_DIR + "/landmarks.csv");
        auto nodes     = load_nodes    (DATA_DIR + "/nodes.csv");
        auto edges     = load_edges    (DATA_DIR + "/edges.csv");
        auto gate      = load_gate     (DATA_DIR + "/seimon.csv");

        const int N = static_cast<int>(landmarks.size());
        std::cout << "  候補数: " << N
                  << "件 / ノード: " << nodes.size()
                  << "件 / エッジ: " << edges.size()
                  << "件" << std::endl;

        Graph     graph(nodes, edges);
        long long gate_node = graph.find_nearest_node(gate.lat, gate.lon);

        // 最短経路の事前計算（ゲート + 全ランドマークの nearest_node を起点に）
        std::cout << "  最短経路を事前計算中..." << std::endl;
        // 候補番号をそのまま表の添字にし、最後のN番を正門にする。
        std::vector<long long> sources;
        for (const auto& lm : landmarks) sources.push_back(lm.nearest_node);
        sources.push_back(gate_node);

        PathCache path_cache(graph, sources);
        const EvaluationTables tables(landmarks, path_cache, gate_node);

        std::cout << "\n遺伝的アルゴリズムを実行中..." << std::endl;
        std::cout << "  個体数: " << pop_size
                  << " / 世代数: " << n_gen
                  << " / 再スタート: " << restarts << std::endl;

        // 開始の種を省略すると RANDOM_SEED を使い、再スタートごとに1ずつ増やす。
        const unsigned int base_seed = settings.seed;

        // 独立再スタートは、読み込みと事前計算（path_cache・tables）は1回だけで、
        // GA だけを種 base_seed, base_seed+1, … で走らせ、最良の解を残す。
        // restarts=1 では、指定した種・個体数・世代数・探索設定で GA を1回だけ実行する。
        auto t_start = std::chrono::high_resolution_clock::now();
        GAResult result;
        for (int r = 0; r < restarts; ++r) {
            RNG rng(base_seed + static_cast<unsigned int>(r));
            if (!options.quiet && restarts > 1)
                std::cout << "  [再スタート " << (r + 1) << "/" << restarts
                          << "]  種 = " << (base_seed + static_cast<unsigned int>(r)) << std::endl;
            GAResult trial = run_ga(landmarks, path_cache, gate_node, tables, rng,
                                    pop_size, n_gen, options);
            // 同点なら先の回（小さい種）を残す。再現性のため厳密比較にする。
            if (r == 0 || trial.best_eval.fitness < result.best_eval.fitness)
                result = std::move(trial);
        }
        auto t_end   = std::chrono::high_resolution_clock::now();
        double elapsed = std::chrono::duration<double>(t_end - t_start).count();

        // 全再スタートを通して有効なコースが1つも作れなかった場合は、
        // 無効解のペナルティ値（実在しない巨大な数）を結果として書き出さない。
        // 黙って「成功」に見える出力を残すより、はっきり失敗させる方が安全。
        if (!result.best_eval.is_valid) {
            std::cerr << "エラー: 有効なコースが見つかりませんでした"
                         "（到達可能な候補が 6 か所未満など）" << std::endl;
            return 1;
        }

        // 結果の出力
        std::filesystem::create_directories(OUTPUT_DIR);
        write_best_course_json(OUTPUT_DIR + "/best_course.json", result, landmarks);
        write_fitness_history_csv(OUTPUT_DIR + "/fitness_history.csv", result.best_fitness_history);

        std::cout << "\n  → " << OUTPUT_DIR << "/best_course.json に保存しました" << std::endl;
        std::cout << "  → " << OUTPUT_DIR << "/fitness_history.csv に保存しました" << std::endl;

        const auto& ev = result.best_eval;
        // JSON と画面で同じ式を使う（evaluate.h の estimated_minutes）。
        double t_estimated = estimated_minutes(ev.total_distance, ev.total_gain);

        std::cout << "\n========== 最良コース ==========" << std::endl;

        // 最良個体の巡回順（正門スタート → コントロール → 正門ゴール）
        std::cout << "巡回順（最良個体）:" << std::endl;
        std::cout << "  スタート : 正門" << std::endl;
        const auto& sel = ev.decoded.selected_indices;
        for (size_t i = 0; i < sel.size(); ++i) {
            const auto& lm = landmarks[sel[i]];
            std::cout << "  " << lm.id << " : " << lm.name << std::endl;
        }
        std::cout << "  ゴール   : 正門" << std::endl;

        std::cout << std::fixed;
        std::cout << "実行時間          : " << std::setprecision(2) << elapsed << " 秒" << std::endl;
        std::cout << "最良解の fitness  : " << std::setprecision(4) << ev.fitness << std::endl;
        std::cout << "コントロール数    : " << ev.decoded.selected_indices.size() << " 件" << std::endl;
        std::cout << "推定時間          : " << std::setprecision(1) << t_estimated << " 分" << std::endl;
        std::cout << "累積登り          : " << std::setprecision(1) << ev.total_gain << " m" << std::endl;
        std::cout << std::setprecision(3);
        std::cout << "f_map             : " << ev.objectives.f_map   << std::endl;
        std::cout << "f_dist            : " << ev.objectives.f_dist  << std::endl;
        std::cout << "f_time            : " << ev.objectives.f_time  << std::endl;
        std::cout << "f_route           : " << ev.objectives.f_route << std::endl;
        std::cout << "==========================================" << std::endl;
        const double total_elapsed = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - total_start).count();
        std::cout << "総実行時間 : " << std::setprecision(2) << total_elapsed << " 秒" << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "エラー: " << e.what() << std::endl;
        return 1;
    }
    return 0;
}
