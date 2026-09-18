// 入力 CSV を一括読み込みし、文字列と数値を地点・道路・正門のデータへ変換する。
#include "csv_loader.h"

#include <charconv>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <type_traits>
#include <utility>

namespace orienteering {

namespace {

// ============================================================
// CSV の読み込み（05 で高速化）
//
// もとは「1行ずつ getline → カンマで std::string に切り出し → stod/stoll」
// だった。文字列を1フィールドごとに作り直すのと、stod（strtod）が
// ロケールを見ながら変換するのが重く、edges.csv だけで約 6.5 ミリ秒かかっていた。
//
// 05 では (1) ファイルを丸ごと1回で読み、(2) 中身は切り出さずに
// 「どこからどこまで」を指す string_view で扱い、(3) 数値化には
// std::from_chars を使う。from_chars は「いちばん近い double」に
// 丸める規則が strtod と同じなので、読み取れる値は 1 ビットも変わらない。
//
// 06 で、小数の from_chars が無い環境（macOS の clang、g++ 11 より前）でも
// ビルドできるよう strtod への切り替えを入れた。詳しくは to_double の直前。
// ============================================================

// 追加: ファイル全体を1つの文字列として読む。
std::string read_whole_file(const std::string& path) {
    std::ifstream ifs(path, std::ios::binary);
    if (!ifs.is_open()) {
        throw std::runtime_error("CSV ファイルが開けません: " + path);
    }
    std::string text;
    ifs.seekg(0, std::ios::end);
    const std::streamoff size = ifs.tellg();
    if (size > 0) text.resize(static_cast<size_t>(size));
    ifs.seekg(0, std::ios::beg);
    if (size > 0) ifs.read(&text[0], size);
    text.resize(static_cast<size_t>(ifs.gcount()));
    return text;
}

// 変更: 先頭の UTF-8 BOM を読み飛ばす（ファイルの最初だけに現れる）
size_t skip_bom(const std::string& text) {
    if (text.size() >= 3 &&
        static_cast<unsigned char>(text[0]) == 0xEF &&
        static_cast<unsigned char>(text[1]) == 0xBB &&
        static_cast<unsigned char>(text[2]) == 0xBF) {
        return 3;
    }
    return 0;
}

// 追加: 位置 pos から1行を取り出す。戻り値は行の中身（改行は含まない）。
// pos は次の行の先頭へ進める。もう行がなければ false を返す。
bool next_line(const std::string& text, size_t& pos, std::string_view& line) {
    if (pos >= text.size()) return false;
    const size_t start = pos;
    size_t end = text.find('\n', start);
    if (end == std::string::npos) {
        end = text.size();
        pos = text.size();
    } else {
        pos = end + 1;
    }
    line = std::string_view(text).substr(start, end - start);
    return true;
}

// 変更: 文字列のコピーを作らずに1行をカンマで分割する（フィールド内のカンマは想定しない）。
// 切り出した範囲を指すだけで、文字列のコピーは作らない。
// 返すのは見つかったフィールド数。fields に入りきらない分は数だけ数える。
size_t split_csv_line(std::string_view line, std::string_view* fields, size_t capacity) {
    size_t count = 0;
    size_t start = 0;
    for (;;) {
        const size_t comma = line.find(',', start);
        const size_t end = comma == std::string_view::npos ? line.size() : comma;
        if (count < capacity) fields[count] = line.substr(start, end - start);
        ++count;
        if (comma == std::string_view::npos) break;
        start = comma + 1;
    }
    return count;
}

// 旧実装は行のどこにある '\r' も取り除いていたので、同じ扱いにそろえる。
std::string to_text(std::string_view field) {
    std::string out;
    out.reserve(field.size());
    for (char c : field) {
        if (c != '\r') out += c;
    }
    return out;
}

// 数値化のとき、旧実装（stod/stoll）と同じように前後の余分な文字を無視する。
std::string_view trim_for_number(std::string_view field) {
    size_t begin = 0;
    while (begin < field.size() &&
           (field[begin] == ' ' || field[begin] == '\t')) ++begin;
    size_t end = field.size();
    while (end > begin && (field[end - 1] == '\r' || field[end - 1] == '\n' ||
                           field[end - 1] == ' ' || field[end - 1] == '\t')) --end;
    return field.substr(begin, end - begin);
}

[[noreturn]] void throw_number_error(std::string_view field) {
    throw std::invalid_argument("数値として読めない項目があります: " +
                                std::string(field));
}

// 追加(09): 壊れた行を見つけても読み込みは止めず、標準エラーに1行だけ警告を出す。
// 主催者のデータが少し違う書き方でも動くように寛容なままにしつつ、
// 「黙って別のデータで計算していた」状態にはならないようにする。
void warn_csv(const std::string& path, size_t line_number, const char* reason) {
    std::cerr << "警告: " << path << " の " << line_number << " 行目: "
              << reason << std::endl;
}

// ------------------------------------------------------------
// 小数の変換は環境によって使える道具が違う（06 で対応）
//
// std::from_chars の「小数版」は C++17 の規格にあるが、実際に用意された
// 時期が処理系ごとに大きく違う。整数版はどこでも使えるのに、小数版は
// Apple の clang（macOS 標準のコンパイラ）や古い g++（11 より前）には
// 入っていない。主催者が macOS でビルドすると、整数版だけを見て
// 「使える」と判断するとコンパイルが通らない。
//
// そこで「小数版の from_chars がこの環境に実在するか」をコンパイル時に
// 調べ、無ければ C 言語の strtod に切り替える。strtod は配布コードが
// 使っていた std::stod の中身そのもので、どの環境にもある。
// 読み取れる値は from_chars と同じ「いちばん近い double」なので、
// どちらを通っても結果は1ビットも変わらない。
// ------------------------------------------------------------

// 追加: 小数版 from_chars を呼べるかどうかを判定する（呼べない環境では false）。
template <class T, class = void>
struct HasFloatFromChars : std::false_type {};

template <class T>
struct HasFloatFromChars<
    T, std::void_t<decltype(std::from_chars(static_cast<const char*>(nullptr),
                                            static_cast<const char*>(nullptr),
                                            std::declval<T&>()))>>
    : std::true_type {};

// 検証用の逃げ道：-DORIENTEERING_NO_FLOAT_FROM_CHARS を付けると、
// from_chars が使える環境でも強制的に strtod 側を通す（速度比較に使う）。
// 判定は二重にする。(1) 標準が定める「用意できています」の印
// __cpp_lib_to_chars と、(2) 実際に呼べるかの検査。両方そろったときだけ
// from_chars を使い、片方でも欠ければ strtod に落ちる（安全側）。
#if defined(__cpp_lib_to_chars) && __cpp_lib_to_chars >= 201611L
#define ORIENTEERING_TO_CHARS_ANNOUNCED 1
#else
#define ORIENTEERING_TO_CHARS_ANNOUNCED 0
#endif

#if defined(ORIENTEERING_NO_FLOAT_FROM_CHARS)
constexpr bool kUseFloatFromChars = false;
#else
constexpr bool kUseFloatFromChars =
    ORIENTEERING_TO_CHARS_ANNOUNCED && HasFloatFromChars<double>::value;
#endif

// strtod は「文字列の終わりが '\0'」であることを前提にするので、
// 切り出した範囲をいったん終端つきの小さな箱に写してから渡す。
// テンプレートにしてあるのは、from_chars を使う環境で
// 「使われない関数がある」という警告を出さないため（中身は T に依らない）。
// 変更(09): 数値の後ろに余分な文字が残っていたら trailing を true にする
//（警告を出すためだけに使う。読み取った値と動きは従来どおり）。
template <class T>
double parse_double_with_strtod(std::string_view s, bool& trailing) {
    char stack_buffer[64];
    const char* begin = nullptr;
    std::string heap_buffer;
    if (s.size() < sizeof(stack_buffer)) {
        std::memcpy(stack_buffer, s.data(), s.size());
        stack_buffer[s.size()] = '\0';
        begin = stack_buffer;
    } else {
        heap_buffer.assign(s.data(), s.size());
        begin = heap_buffer.c_str();
    }
    char* end = nullptr;
    const double value = std::strtod(begin, &end);
    if (end == begin) throw_number_error(s);
    if (static_cast<size_t>(end - begin) != s.size()) trailing = true;
    return value;
}

// 追加: 小数変換を処理系に応じて切り替える。使わない側の分岐は
// コンパイル時にそもそも組み立てられない（＝存在しない関数を呼ばずに済む）。
template <class T>
T parse_float(std::string_view s, bool& trailing) {
    if constexpr (kUseFloatFromChars) {
        T value = static_cast<T>(0);
        const auto result = std::from_chars(s.data(), s.data() + s.size(), value);
        if (result.ec != std::errc{} || result.ptr == s.data()) throw_number_error(s);
        if (result.ptr != s.data() + s.size()) trailing = true;
        return value;
    } else {
        return static_cast<T>(parse_double_with_strtod<T>(s, trailing));
    }
}

double to_double(std::string_view field, bool& trailing) {
    return parse_float<double>(trim_for_number(field), trailing);
}

// 追加: 切り出した範囲を from_chars で整数に変換する。
long long to_int64(std::string_view field, bool& trailing) {
    const std::string_view s = trim_for_number(field);
    long long value = 0;
    const auto result = std::from_chars(s.data(), s.data() + s.size(), value);
    if (result.ec != std::errc{} || result.ptr == s.data()) throw_number_error(field);
    if (result.ptr != s.data() + s.size()) trailing = true;
    return value;
}

} // namespace

bool uses_float_from_chars() { return kUseFloatFromChars; }

// 変更: 一括読み込みした CSV の参照範囲から候補地点を組み立てる。
std::vector<Landmark> load_landmarks(const std::string& path) {
    const std::string text = read_whole_file(path);
    std::vector<Landmark> landmarks;

    size_t pos = skip_bom(text);
    std::string_view line;
    next_line(text, pos, line);  // ヘッダ行はスキップ

    std::string_view t[6];
    size_t line_number = 1;  // 1行目はヘッダ
    while (next_line(text, pos, line)) {
        ++line_number;
        if (line.empty()) continue;
        // 変更(09): 読み飛ばす行と、数値の後ろに余分な文字がある行を警告する。
        if (split_csv_line(line, t, 6) < 6) {
            warn_csv(path, line_number, "列が足りないので読み飛ばしました");
            continue;
        }
        Landmark lm;
        bool trailing   = false;
        lm.id           = static_cast<int>(to_int64(t[0], trailing));
        lm.name         = to_text(t[1]);
        lm.feature      = to_text(t[2]);
        lm.lat          = to_double(t[3], trailing);
        lm.lon          = to_double(t[4], trailing);
        lm.nearest_node = to_int64(t[5], trailing);
        if (trailing) warn_csv(path, line_number, "数値の後ろに余分な文字があります（読めたところまでを使いました）");
        landmarks.push_back(std::move(lm));
    }
    return landmarks;
}

// 変更: 一括読み込みした CSV の参照範囲から道路ノードを組み立てる。
std::vector<Node> load_nodes(const std::string& path) {
    const std::string text = read_whole_file(path);
    std::vector<Node> nodes;
    nodes.reserve(text.size() / 40 + 16);  // 1行あたりの目安から先に場所を確保する

    size_t pos = skip_bom(text);
    std::string_view line;
    next_line(text, pos, line);

    std::string_view t[4];
    size_t line_number = 1;  // 1行目はヘッダ
    while (next_line(text, pos, line)) {
        ++line_number;
        if (line.empty()) continue;
        // 変更(09): 読み飛ばす行と、数値の後ろに余分な文字がある行を警告する。
        if (split_csv_line(line, t, 4) < 4) {
            warn_csv(path, line_number, "列が足りないので読み飛ばしました");
            continue;
        }
        Node n;
        bool trailing = false;
        n.node_id   = to_int64(t[0], trailing);
        n.lat       = to_double(t[1], trailing);
        n.lon       = to_double(t[2], trailing);
        n.elevation = to_double(t[3], trailing);
        if (trailing)
            warn_csv(path, line_number, "数値の後ろに余分な文字があります（読めたところまでを使いました）");
        nodes.push_back(n);
    }
    return nodes;
}

// 変更: 一括読み込みした CSV の参照範囲から道路の辺を組み立てる。
std::vector<Edge> load_edges(const std::string& path) {
    const std::string text = read_whole_file(path);
    std::vector<Edge> edges;
    edges.reserve(text.size() / 60 + 16);

    size_t pos = skip_bom(text);
    std::string_view line;
    next_line(text, pos, line);

    std::string_view t[5];
    size_t line_number = 1;  // 1行目はヘッダ
    while (next_line(text, pos, line)) {
        ++line_number;
        if (line.empty()) continue;
        // 変更(09): 読み飛ばす行と、数値の後ろに余分な文字がある行を警告する。
        if (split_csv_line(line, t, 5) < 5) {
            warn_csv(path, line_number, "列が足りないので読み飛ばしました");
            continue;
        }
        Edge e;
        bool trailing      = false;
        e.from_node        = to_int64(t[0], trailing);
        e.to_node          = to_int64(t[1], trailing);
        e.length_m         = to_double(t[2], trailing);
        e.elevation_change = to_double(t[3], trailing);
        e.elevation_gain   = to_double(t[4], trailing);
        if (trailing)
            warn_csv(path, line_number, "数値の後ろに余分な文字があります（読めたところまでを使いました）");
        edges.push_back(e);
    }
    return edges;
}

// 変更: 一括読み込みした CSV の2行目から正門座標を取得する。
Gate load_gate(const std::string& path) {
    const std::string text = read_whole_file(path);

    size_t pos = skip_bom(text);
    std::string_view line;
    next_line(text, pos, line);            // 1行目: ヘッダ（読み飛ばす）
    if (!next_line(text, pos, line)) {     // 2行目: 座標
        throw std::runtime_error("正門の座標が読み込めません: " + path);
    }

    std::string_view t[2];
    if (split_csv_line(line, t, 2) < 2) {
        throw std::runtime_error("正門の座標が読み込めません: " + path);
    }
    Gate g;
    bool trailing = false;
    g.lat = to_double(t[0], trailing);
    g.lon = to_double(t[1], trailing);
    // 変更(09): 数値の後ろに余分な文字があれば警告する（読み込みは続ける）。
    if (trailing)
        warn_csv(path, 2, "数値の後ろに余分な文字があります（読めたところまでを使いました）");
    return g;
}

} // namespace orienteering
