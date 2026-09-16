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
/// @file    RTSImCFDTable.h
/// @brief   Validated CFD coefficient data and bounded gap interpolation.
/****************************************************************************/
#pragma once

#include <map>
#include <string>
#include <tuple>
#include <utility>

namespace RTSIm {

/** @brief CFD coefficients indexed by model, platoon size, position, and gap.
 *
 * Positions are one-based (1 is the leading vehicle). Gap is in metres; Cd is
 * dimensionless. CSV columns must be exactly model_id, platoon_size, gap_m,
 * position, cd_lower, cd_upper, in any order. Quoted CSV fields are supported,
 * but multiline quoted fields are not. Blank lines are ignored.
 *
 * This class does not infer membership, choose a representative gap, or fall
 * back to solo data. Those are explicit responsibilities of its caller.
 */
class CFDTable {
public:
    /** @brief Load a CSV file, replacing all previous entries on success.
     * @throws std::runtime_error For unreadable files or invalid data/schema.
     * A failed load leaves the previously loaded table unchanged.
     */
    void load(const std::string& path);

    /** @brief Look up Cd, linearly interpolating within one coefficient curve.
     * @param model Exact, case-sensitive model identifier.
     * @param size Platoon size (positive).
     * @param position One-based position in the platoon (1 through size).
     * @param gap Nonnegative finite gap in metres; solo queries use 0.
     * @param bound Either "lower" or "upper" (case-sensitive).
     * @throws std::invalid_argument For an invalid query.
     * @throws std::out_of_range For missing curves or gaps outside their range.
     * Exact table points return the recorded value. There is no extrapolation,
     * monotonicity assumption, or clamping against a solo coefficient.
     */
    double lookup(const std::string& model, int size, int position,
                  double gap, const std::string& bound) const;

private:
    typedef std::tuple<std::string, int, int> CurveKey;
    typedef std::pair<double, double> Bounds;
    typedef std::map<double, Bounds> Curve;
    std::map<CurveKey, Curve> myCurves;
};

} // namespace RTSIm
