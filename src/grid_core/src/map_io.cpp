#include <cctype>
#include <filesystem>
#include <fstream>
#include <sstream>

#include "grid_core/grid.hpp"

namespace grid_core {

namespace {

std::nullopt_t fail(std::string* error, const std::string& msg) {
    if (error) *error = msg;
    return std::nullopt;
}

std::string trim(const std::string& s) {
    const auto b = s.find_first_not_of(" \t\r\"'");
    const auto e = s.find_last_not_of(" \t\r\"'");
    return b == std::string::npos ? "" : s.substr(b, e - b + 1);
}

// Reads the next whitespace-separated header token of a PGM, skipping
// '#' comments.
bool pgmToken(std::istream& in, std::string& tok) {
    tok.clear();
    char ch;
    while (in.get(ch)) {
        if (ch == '#') {
            std::string ignored;
            std::getline(in, ignored);
        } else if (!std::isspace(static_cast<unsigned char>(ch))) {
            tok.push_back(ch);
            break;
        }
    }
    while (in.get(ch) && !std::isspace(static_cast<unsigned char>(ch))) tok.push_back(ch);
    return !tok.empty();
}

}  // namespace

std::optional<StaticMap> loadMap(const std::string& yaml_path, std::string* error) {
    // The map YAML is a flat "key: value" file, so a full YAML parser is not
    // needed; unknown keys (e.g. mode) are ignored.
    std::ifstream yaml(yaml_path);
    if (!yaml) return fail(error, "cannot open " + yaml_path);
    std::string image;
    double resolution = 0.0, ox = 0.0, oy = 0.0, free_th = 0.196;
    int negate = 0;
    std::string line;
    while (std::getline(yaml, line)) {
        const auto colon = line.find(':');
        if (colon == std::string::npos || trim(line).rfind('#', 0) == 0) continue;
        const std::string key = trim(line.substr(0, colon)), val = trim(line.substr(colon + 1));
        if (key == "image")
            image = val;
        else if (key == "resolution")
            resolution = std::stod(val);
        else if (key == "negate")
            negate = std::stoi(val);
        else if (key == "free_thresh")
            free_th = std::stod(val);
        else if (key == "origin") {
            std::string v = val;
            for (char& ch : v)
                if (ch == '[' || ch == ']' || ch == ',') ch = ' ';
            std::istringstream(v) >> ox >> oy;
        }
    }
    if (image.empty() || resolution <= 0.0)
        return fail(error, yaml_path + ": missing image or resolution");

    const std::filesystem::path img_path =
        std::filesystem::path(yaml_path).parent_path() / std::filesystem::path(image);
    std::ifstream pgm(img_path, std::ios::binary);
    if (!pgm) return fail(error, "cannot open " + img_path.string());
    std::string magic, ws, hs, ms;
    if (!pgmToken(pgm, magic) || (magic != "P5" && magic != "P2") || !pgmToken(pgm, ws) ||
        !pgmToken(pgm, hs) || !pgmToken(pgm, ms))
        return fail(error, img_path.string() + ": not a P2/P5 PGM");
    const int w = std::stoi(ws), h = std::stoi(hs), maxval = std::stoi(ms);
    if (w <= 0 || h <= 0 || maxval <= 0 || maxval > 255)
        return fail(error, img_path.string() + ": unsupported PGM size/depth");

    StaticMap map;
    map.geometry = {w, h, resolution, ox, oy};
    map.occupied.assign(static_cast<std::size_t>(w) * static_cast<std::size_t>(h), 0);
    for (int row = 0; row < h; ++row)
        for (int col = 0; col < w; ++col) {
            int px;
            if (magic == "P5") {
                char ch;
                if (!pgm.get(ch)) return fail(error, "truncated PGM");
                px = static_cast<unsigned char>(ch);
            } else if (!(pgm >> px)) {
                return fail(error, "truncated PGM");
            }
            // map_server semantics: darker = more likely occupied.
            double p = static_cast<double>(maxval - px) / maxval;
            if (negate) p = 1.0 - p;
            // Image rows run top to bottom; grid rows run bottom to top. Only
            // confidently free pixels are free: unknown (between free_thresh
            // and occupied_thresh) is treated as blocked, so occupied_thresh
            // is not needed for a binary map.
            const bool free = p < free_th;
            map.occupied[map.geometry.index({col, h - 1 - row})] = free ? 0 : 1;
        }
    return map;
}

}  // namespace grid_core
