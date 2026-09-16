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
// Standalone tests; no SUMO or external test library dependencies.
#include "../../src/microsim/devices/RTSImCFDTable.h"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

unsigned assertionsPassed = 0;

const std::string HEADER = "model_id,platoon_size,gap_m,position,cd_lower,cd_upper\n";
const std::string VALID = HEADER +
    "Model1,1,0,1,0.700000,0.950000\n"
    "Model1,2,5,1,0.636723,0.864124\n"
    "Model1,2,10,1,0.694727,0.942844\n"
    "Model1,2,15,1,0.709228,0.962524\n"
    "Model1,2,20,1,0.693409,0.941055\n"
    "Model1,2,5,2,0.536535,0.728154\n"
    "Model1,2,10,2,0.533898,0.724576\n"
    "Model1,3,5,2,0.442938,0.601130\n"
    "Model1,3,10,2,0.474576,0.644068\n"
    "DifferentModel,2,5,1,0.1,0.2\n";

class Fixture {
public:
    explicit Fixture(const std::string& data) {
        static unsigned counter = 0;
        std::ostringstream name;
        name << "rtsim_cfd_test_" << std::chrono::high_resolution_clock::now().time_since_epoch().count()
             << "_" << counter++ << ".csv";
        path = name.str();
        std::ofstream file(path.c_str(), std::ios::binary);
        file << data;
        if (!file) {
            throw std::runtime_error("Cannot write temporary CFD fixture");
        }
    }
    ~Fixture() {
        std::remove(path.c_str());
    }
    std::string path;
};

void check(bool condition, const std::string& description) {
    if (!condition) {
        throw std::runtime_error(description);
    }
    ++assertionsPassed;
}

void close(double actual, double expected, const std::string& description) {
    check(std::isfinite(actual) && std::abs(actual - expected) < 1e-12, description);
}

template<class Exception, class Function>
void rejects(Function function, const std::string& description) {
    try {
        function();
    } catch (const Exception&) {
        ++assertionsPassed;
        return;
    }
    throw std::runtime_error("Expected rejection: " + description);
}

void runTests() {
    Fixture fixture(VALID);
    RTSIm::CFDTable table;
    table.load(fixture.path);
    close(table.lookup("Model1", 1, 1, 0, "lower"), 0.7, "solo lower");
    close(table.lookup("Model1", 1, 1, 0, "upper"), 0.95, "solo upper");
    close(table.lookup("Model1", 2, 1, 5, "lower"), 0.636723, "exact lead coefficient");
    close(table.lookup("Model1", 2, 2, 5, "lower"), 0.536535, "exact trailing coefficient");
    close(table.lookup("Model1", 3, 2, 5, "upper"), 0.601130, "exact middle coefficient");
    close(table.lookup("Model1", 2, 1, 20, "upper"), 0.941055, "maximum gap endpoint");
    close(table.lookup("Model1", 2, 1, 12.5, "lower"), 0.7019775, "increasing interpolation");
    close(table.lookup("Model1", 2, 1, 17.5, "lower"), 0.7013185, "decreasing interpolation");
    close(table.lookup("Model1", 2, 2, 7.5, "upper"), 0.726365, "trailing upper interpolation");
    close(table.lookup("DifferentModel", 2, 1, 5, "lower"), 0.1, "model isolation");
    check(table.lookup("Model1", 2, 1, 15, "upper") > table.lookup("Model1", 1, 1, 0, "upper"),
          "CFD upper bound must not be clamped to solo Cd");

    rejects<std::out_of_range>([&]() { table.lookup("Model1", 2, 1, 4.99, "lower"); }, "lower extrapolation");
    rejects<std::out_of_range>([&]() { table.lookup("Model1", 2, 1, 20.01, "upper"); }, "upper extrapolation");
    rejects<std::out_of_range>([&]() { table.lookup("DifferentModel", 2, 1, 6, "lower"); }, "single-point extrapolation");
    rejects<std::out_of_range>([&]() { table.lookup("Model1", 3, 1, 5, "lower"); }, "missing position curve");
    rejects<std::out_of_range>([&]() { table.lookup("unknown", 1, 1, 0, "lower"); }, "missing model");
    rejects<std::invalid_argument>([&]() { table.lookup("Model1", 1, 1, 1, "lower"); }, "nonzero solo gap");
    rejects<std::invalid_argument>([&]() { table.lookup("Model1", 2, 3, 5, "lower"); }, "position exceeds size");
    rejects<std::invalid_argument>([&]() { table.lookup("Model1", 0, 1, 5, "lower"); }, "zero size");
    rejects<std::invalid_argument>([&]() { table.lookup("Model1", 2, 0, 5, "lower"); }, "zero position");
    rejects<std::invalid_argument>([&]() { table.lookup("", 2, 1, 5, "lower"); }, "empty model");
    rejects<std::invalid_argument>([&]() { table.lookup("Model1", 2, 1, -1, "lower"); }, "negative gap");
    rejects<std::invalid_argument>([&]() { table.lookup("Model1", 2, 1, std::numeric_limits<double>::infinity(), "lower"); }, "infinite gap");
    rejects<std::invalid_argument>([&]() { table.lookup("Model1", 2, 1, std::numeric_limits<double>::quiet_NaN(), "lower"); }, "NaN gap");
    rejects<std::invalid_argument>([&]() { table.lookup("Model1", 2, 1, 5, "mean"); }, "unknown bound");

    const std::vector<std::string> invalid = {
        "", HEADER, "model_id,platoon_size,gap_m,position,cd_lower\n",
        "model_id,platoon_size,gap_m,position,cd_lower,cd_lower\n",
        "model_id,platoon_size,gap_m,position,cd_lower,unexpected\n",
        HEADER + "Model1,2,5,1,0.6\n",
        HEADER + "Model1,2,5,1,0.6,0.9,extra\n",
        HEADER + ",2,5,1,0.6,0.9\n",
        HEADER + "Model1,0,5,1,0.6,0.9\n",
        HEADER + "Model1,-2,5,1,0.6,0.9\n",
        HEADER + "Model1,2.0,5,1,0.6,0.9\n",
        HEADER + "Model1,99999999999999999999999,5,1,0.6,0.9\n",
        HEADER + "Model1,2,5,0,0.6,0.9\n",
        HEADER + "Model1,2,5,3,0.6,0.9\n",
        HEADER + "Model1,2,-5,1,0.6,0.9\n",
        HEADER + "Model1,1,5,1,0.6,0.9\n",
        HEADER + "Model1,2,nan,1,0.6,0.9\n",
        HEADER + "Model1,2,5m,1,0.6,0.9\n",
        HEADER + "Model1,2,5,1,,0.9\n",
        HEADER + "Model1,2,5,1,0,0.9\n",
        HEADER + "Model1,2,5,1,-0.6,0.9\n",
        HEADER + "Model1,2,5,1,0.6,0\n",
        HEADER + "Model1,2,5,1,1,0.9\n",
        HEADER + "Model1,2,5,1,inf,0.9\n",
        HEADER + "Model1,2,5,1,0.6,1e999\n",
        HEADER + "Model1,2,5,1,0.6,0.9\nModel1,2,5.0,1,0.7,1.0\n",
        HEADER + "\"Model1,2,5,1,0.6,0.9\n",
        HEADER + "\"Model1\"x,2,5,1,0.6,0.9\n",
        HEADER + "Mod\"el1,2,5,1,0.6,0.9\n"
    };
    for (std::size_t i = 0; i < invalid.size(); ++i) {
        Fixture broken(invalid[i]);
        rejects<std::runtime_error>([&]() { table.load(broken.path); }, "invalid CSV fixture");
        close(table.lookup("Model1", 1, 1, 0, "lower"), 0.7, "failed load preserves previous data");
    }
    rejects<std::runtime_error>([&]() { table.load(fixture.path + ".missing"); }, "missing file");

    Fixture reordered(std::string("\xef\xbb\xbf") +
        "cd_upper, position, \"model_id\", cd_lower,gap_m,platoon_size\r\n\r\n"
        "0.95,1,\"Model, \"\"quoted\"\"\",0.7,0,1\r\n");
    table.load(reordered.path);
    close(table.lookup("Model, \"quoted\"", 1, 1, 0, "upper"), 0.95, "quoted fields, reordered header, BOM and CRLF");
    rejects<std::out_of_range>([&]() { table.lookup("Model1", 1, 1, 0, "lower"); }, "successful load replaces data");
}

} // namespace

int main(int argc, char** argv) {
    try {
        runTests();
        if (argc > 1) {
            RTSIm::CFDTable original;
            original.load(argv[1]);
            close(original.lookup("Model1", 1, 1, 0, "upper"), 0.95, "source table solo");
            close(original.lookup("Model1", 2, 1, 15, "upper"), 0.962524, "source table nonmonotonic coefficient");
            close(original.lookup("Model1", 3, 2, 5, "lower"), 0.442938, "source table middle coefficient");
        }
        std::cout << "RTSIm CFD table tests passed: " << assertionsPassed << " assertions\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "RTSIm CFD table test failed: " << error.what() << '\n';
        return 1;
    }
}
