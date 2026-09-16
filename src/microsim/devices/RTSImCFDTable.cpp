/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.dev/sumo
// Copyright (C) 2026 RTSIm contributors.
// This program and the accompanying materials are made available under the
// terms of the Eclipse Public License 2.0 which is available at
// https://www.eclipse.org/legal/epl-2.0/
// This Source Code may also be made available under the following Secondary
// Licenses when the conditions for such availability set forth in the Eclipse
// Public License 2.0 are satisfied: GNU General Public License, version 2
// or later which is available at
// https://www.gnu.org/licenses/old-licenses/gpl-2.0-standalone.html
// SPDX-License-Identifier: EPL-2.0 OR GPL-2.0-or-later
/****************************************************************************/
/// @file    RTSImCFDTable.cpp
/// @brief   Validated CFD coefficient data and bounded gap interpolation.
/****************************************************************************/
#include "RTSImCFDTable.h"

#include <cctype>
#include <cmath>
#include <fstream>
#include <limits>
#include <locale>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace {

bool isSpace(char c) {
    return std::isspace(static_cast<unsigned char>(c)) != 0;
}

std::string trim(const std::string& value) {
    std::size_t first = 0;
    std::size_t last = value.size();
    while (first < last && isSpace(value[first])) {
        ++first;
    }
    while (last > first && isSpace(value[last - 1])) {
        --last;
    }
    return value.substr(first, last - first);
}

// Parse one physical CSV line, including escaped quotes and empty fields.
std::vector<std::string> parseCSV(const std::string& line) {
    std::vector<std::string> fields;
    std::size_t offset = 0;
    while (true) {
        while (offset < line.size() && isSpace(line[offset])) {
            ++offset;
        }
        std::string field;
        if (offset < line.size() && line[offset] == '"') {
            ++offset;
            bool closed = false;
            while (offset < line.size()) {
                const char c = line[offset++];
                if (c != '"') {
                    field += c;
                } else if (offset < line.size() && line[offset] == '"') {
                    field += '"';
                    ++offset;
                } else {
                    closed = true;
                    break;
                }
            }
            if (!closed) {
                throw std::runtime_error("unterminated quoted CSV field");
            }
            while (offset < line.size() && isSpace(line[offset])) {
                ++offset;
            }
            if (offset < line.size() && line[offset] != ',') {
                throw std::runtime_error("unexpected text after quoted CSV field");
            }
        } else {
            while (offset < line.size() && line[offset] != ',') {
                if (line[offset] == '"') {
                    throw std::runtime_error("unexpected quote in unquoted CSV field");
                }
                field += line[offset++];
            }
        }
        fields.push_back(trim(field));
        if (offset == line.size()) {
            break;
        }
        ++offset; // comma; another field exists even when the comma is last
    }
    return fields;
}

int parsePositiveInt(const std::string& value, const std::string& name) {
    // Only decimal integer digits are accepted (not 1.0, 1e0, or a sign).
    if (value.empty()) {
        throw std::runtime_error(name + " must be a positive integer");
    }
    int result = 0;
    for (std::size_t i = 0; i < value.size(); ++i) {
        const char c = value[i];
        if (c < '0' || c > '9' ||
                result > (std::numeric_limits<int>::max() - (c - '0')) / 10) {
            throw std::runtime_error(name + " must be a positive integer");
        }
        result = result * 10 + (c - '0');
    }
    if (result == 0) {
        throw std::runtime_error(name + " must be a positive integer");
    }
    return result;
}

double parseDouble(const std::string& value, const std::string& name) {
    std::istringstream input(value);
    input.imbue(std::locale::classic());
    double result = 0;
    input >> result;
    if (!input || !std::isfinite(result)) {
        throw std::runtime_error(name + " must be finite numeric data");
    }
    input >> std::ws;
    if (!input.eof()) {
        throw std::runtime_error(name + " contains trailing nonnumeric data");
    }
    return result;
}

std::string describe(const std::string& model, int size, int position) {
    std::ostringstream text;
    text << "model='" << model << "', size=" << size << ", position=" << position;
    return text.str();
}

} // namespace

namespace RTSIm {

void CFDTable::load(const std::string& path) {
    std::ifstream input(path.c_str());
    if (!input) {
        throw std::runtime_error("RTSIm CFD table: cannot open '" + path + "'");
    }
    std::map<CurveKey, Curve> curves;
    std::map<std::string, std::size_t> columns;
    const char* required[] = {
        "model_id", "platoon_size", "gap_m", "position", "cd_lower", "cd_upper"
    };
    bool haveHeader = false;
    std::size_t lineNumber = 0;
    std::string line;
    while (std::getline(input, line)) {
        ++lineNumber;
        // Accept the UTF-8 BOM emitted by some spreadsheet exporters.
        if (lineNumber == 1 && line.compare(0, 3, "\xef\xbb\xbf") == 0) {
            line.erase(0, 3);
        }
        if (trim(line).empty()) {
            continue;
        }
        try {
            const std::vector<std::string> fields = parseCSV(line);
            if (!haveHeader) {
                if (fields.size() != 6) {
                    throw std::runtime_error("expected exactly six required CSV columns");
                }
                for (std::size_t i = 0; i < fields.size(); ++i) {
                    if (!columns.insert(std::make_pair(fields[i], i)).second) {
                        throw std::runtime_error("duplicate CSV column '" + fields[i] + "'");
                    }
                }
                for (std::size_t i = 0; i < 6; ++i) {
                    if (columns.find(required[i]) == columns.end()) {
                        throw std::runtime_error(std::string("missing CSV column '") + required[i] + "'");
                    }
                }
                haveHeader = true;
                continue;
            }
            if (fields.size() != 6) {
                throw std::runtime_error("data row must contain exactly six CSV fields");
            }
            const std::string& model = fields[columns.at("model_id")];
            if (model.empty()) {
                throw std::runtime_error("model_id must not be empty");
            }
            const int size = parsePositiveInt(fields[columns.at("platoon_size")], "platoon_size");
            const int position = parsePositiveInt(fields[columns.at("position")], "position");
            if (position > size) {
                throw std::runtime_error("position must not exceed platoon_size");
            }
            const double gap = parseDouble(fields[columns.at("gap_m")], "gap_m");
            if (gap < 0 || (size == 1 && gap != 0)) {
                throw std::runtime_error("gap_m must be nonnegative and must equal zero for solo data");
            }
            const double lower = parseDouble(fields[columns.at("cd_lower")], "cd_lower");
            const double upper = parseDouble(fields[columns.at("cd_upper")], "cd_upper");
            if (lower <= 0 || upper <= 0 || lower > upper) {
                throw std::runtime_error("Cd bounds must be positive with cd_lower <= cd_upper");
            }
            Curve& curve = curves[CurveKey(model, size, position)];
            if (!curve.insert(std::make_pair(gap, Bounds(lower, upper))).second) {
                throw std::runtime_error("duplicate coefficient key for " + describe(model, size, position));
            }
        } catch (const std::runtime_error& error) {
            std::ostringstream message;
            message << "RTSIm CFD table '" << path << "', line " << lineNumber << ": " << error.what();
            throw std::runtime_error(message.str());
        }
    }
    if (input.bad()) {
        throw std::runtime_error("RTSIm CFD table: read failed for '" + path + "'");
    }
    if (!haveHeader || curves.empty()) {
        throw std::runtime_error("RTSIm CFD table '" + path + "' contains no coefficient data");
    }
    myCurves.swap(curves);
}

double CFDTable::lookup(const std::string& model, int size, int position,
                       double gap, const std::string& bound) const {
    if (model.empty() || size < 1 || position < 1 || position > size ||
            !std::isfinite(gap) || gap < 0 || (size == 1 && gap != 0)) {
        throw std::invalid_argument("RTSIm CFD lookup: invalid model, size, position, or gap");
    }
    if (bound != "lower" && bound != "upper") {
        throw std::invalid_argument("RTSIm CFD lookup: bound must be 'lower' or 'upper'");
    }
    const bool useUpper = bound == "upper";
    const std::map<CurveKey, Curve>::const_iterator found = myCurves.find(CurveKey(model, size, position));
    if (found == myCurves.end()) {
        throw std::out_of_range("RTSIm CFD lookup: no data for " + describe(model, size, position));
    }
    const Curve& curve = found->second;
    Curve::const_iterator right = curve.lower_bound(gap);
    if (right != curve.end() && right->first == gap) {
        return useUpper ? right->second.second : right->second.first;
    }
    if (right == curve.begin() || right == curve.end()) {
        std::ostringstream message;
        message << "RTSIm CFD lookup: gap " << gap << " is outside ["
                << curve.begin()->first << ", " << curve.rbegin()->first << "] for "
                << describe(model, size, position);
        throw std::out_of_range(message.str());
    }
    Curve::const_iterator left = right;
    --left;
    const double leftCd = useUpper ? left->second.second : left->second.first;
    const double rightCd = useUpper ? right->second.second : right->second.first;
    const double fraction = (gap - left->first) / (right->first - left->first);
    return leftCd + fraction * (rightCd - leftCd);
}

} // namespace RTSIm
