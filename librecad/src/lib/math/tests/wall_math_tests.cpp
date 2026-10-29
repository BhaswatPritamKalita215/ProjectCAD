/****************************************************************************
**
** This file is part of the LibreCAD project, a 2D CAD program
**
** Copyright (C) 2026 ProjectCAD / LibreCAD developers
**
** This program is free software; you can redistribute it and/or
** modify it under the terms of the GNU General Public License
** as published by the Free Software Foundation; either version 2
** of the License, or (at your option) any later version.
**
** This program is distributed in the hope that it will be useful,
** but WITHOUT ANY WARRANTY; without even the implied warranty of
** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
** GNU General Public License for more details.
**
** You should have received a copy of the GNU General Public License
** along with this program; if not, write to the Free Software
** Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
**
**********************************************************************/

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <cmath>
#include <limits>
#include <vector>

#include "lc_wallmath.h"
#include "rs.h"

using Catch::Approx;

namespace {
constexpr double TEST_EPS = 1e-5;

void verifyAllCoordinatesFiniteAndNonZero(const LC_WallMath::WallGeometry& geom) {
    const auto allLines = geom.toLineDataList();
    REQUIRE(!allLines.empty());
    for (const auto& line : allLines) {
        REQUIRE(std::isfinite(line.startpoint.x));
        REQUIRE(std::isfinite(line.startpoint.y));
        REQUIRE(std::isfinite(line.endpoint.x));
        REQUIRE(std::isfinite(line.endpoint.y));
        REQUIRE(line.startpoint.distanceTo(line.endpoint) > RS_TOLERANCE);
    }
}

/**
 * @brief Helper to detect duplicate undirected line segments in a line list.
 * Two lines are considered duplicates if they span the exact same two endpoints
 * in either direction within RS_TOLERANCE.
 */
bool hasDuplicateSegments(const std::vector<RS_LineData>& lines) {
    for (size_t i = 0; i < lines.size(); ++i) {
        for (size_t j = i + 1; j < lines.size(); ++j) {
            const auto& l1 = lines[i];
            const auto& l2 = lines[j];
            const bool sameDir = (l1.startpoint.distanceTo(l2.startpoint) < RS_TOLERANCE &&
                                  l1.endpoint.distanceTo(l2.endpoint) < RS_TOLERANCE);
            const bool oppDir  = (l1.startpoint.distanceTo(l2.endpoint) < RS_TOLERANCE &&
                                  l1.endpoint.distanceTo(l2.startpoint) < RS_TOLERANCE);
            if (sameDir || oppDir) {
                return true;
            }
        }
    }
    return false;
}

void verifyBoundaryContinuity(const std::vector<RS_LineData>& boundary, bool isClosed) {
    REQUIRE(!boundary.empty());
    for (size_t i = 0; i + 1 < boundary.size(); ++i) {
        CHECK(boundary[i].endpoint.distanceTo(boundary[i + 1].startpoint) < TEST_EPS);
    }
    if (isClosed && boundary.size() >= 3) {
        CHECK(boundary.back().endpoint.distanceTo(boundary.front().startpoint) < TEST_EPS);
    }
}

void verifyPerSegmentForwardProgression(const LC_WallMath::WallGeometry& geom,
                                       const std::vector<RS_Vector>& centerlines,
                                       bool isClosed) {
    const size_t N = centerlines.size();
    const size_t M = isClosed ? N : (N - 1);
    for (size_t i = 0; i < M; ++i) {
        const RS_Vector dir = (centerlines[(i + 1) % N] - centerlines[i]).normalized();
        for (const auto& l : geom.leftBoundary) {
            const RS_Vector lDir = (l.endpoint - l.startpoint).normalized();
            if (lDir.dotP(dir) > 0.9) {
                CHECK((l.endpoint - l.startpoint).dotP(dir) > RS_TOLERANCE);
            }
        }
        for (const auto& l : geom.rightBoundary) {
            const RS_Vector rDir = (l.endpoint - l.startpoint).normalized();
            if (rDir.dotP(dir) > 0.9) {
                CHECK((l.endpoint - l.startpoint).dotP(dir) > RS_TOLERANCE);
            }
        }
    }
}

} // anonymous namespace

TEST_CASE("WallMath: Horizontal centerline and boundary coordinates", "[wall][geometry]") {
    const std::vector<RS_Vector> center = {{0.0, 0.0}, {100.0, 0.0}};
    const double thickness = 10.0;

    const auto wall = LC_WallMath::generateWall(center, thickness, LC_WallMath::CapType::Flat);
    REQUIRE(wall.isValid());
    verifyAllCoordinatesFiniteAndNonZero(wall);
    CHECK(!hasDuplicateSegments(wall.toLineDataList()));

    REQUIRE(wall.leftBoundary.size() == 1);
    REQUIRE(wall.rightBoundary.size() == 1);
    REQUIRE(wall.caps.size() == 2);

    // Left boundary: (+Y offset by thickness/2 = 5)
    CHECK(wall.leftBoundary[0].startpoint.x == Approx(0.0).margin(TEST_EPS));
    CHECK(wall.leftBoundary[0].startpoint.y == Approx(5.0).margin(TEST_EPS));
    CHECK(wall.leftBoundary[0].endpoint.x == Approx(100.0).margin(TEST_EPS));
    CHECK(wall.leftBoundary[0].endpoint.y == Approx(5.0).margin(TEST_EPS));

    // Right boundary: (-Y offset by thickness/2 = 5)
    CHECK(wall.rightBoundary[0].startpoint.x == Approx(0.0).margin(TEST_EPS));
    CHECK(wall.rightBoundary[0].startpoint.y == Approx(-5.0).margin(TEST_EPS));
    CHECK(wall.rightBoundary[0].endpoint.x == Approx(100.0).margin(TEST_EPS));
    CHECK(wall.rightBoundary[0].endpoint.y == Approx(-5.0).margin(TEST_EPS));

    // Start cap: from right start (0, -5) to left start (0, 5)
    CHECK(wall.caps[0].startpoint.x == Approx(0.0).margin(TEST_EPS));
    CHECK(wall.caps[0].startpoint.y == Approx(-5.0).margin(TEST_EPS));
    CHECK(wall.caps[0].endpoint.x == Approx(0.0).margin(TEST_EPS));
    CHECK(wall.caps[0].endpoint.y == Approx(5.0).margin(TEST_EPS));

    // End cap: from left end (100, 5) to right end (100, -5)
    CHECK(wall.caps[1].startpoint.x == Approx(100.0).margin(TEST_EPS));
    CHECK(wall.caps[1].startpoint.y == Approx(5.0).margin(TEST_EPS));
    CHECK(wall.caps[1].endpoint.x == Approx(100.0).margin(TEST_EPS));
    CHECK(wall.caps[1].endpoint.y == Approx(-5.0).margin(TEST_EPS));
}

TEST_CASE("WallMath: Vertical and diagonal centerlines", "[wall][geometry]") {
    SECTION("Vertical upward centerline") {
        const std::vector<RS_Vector> center = {{0.0, 0.0}, {0.0, 100.0}};
        const double thickness = 10.0;
        const auto wall = LC_WallMath::generateWall(center, thickness, LC_WallMath::CapType::Flat);
        REQUIRE(wall.isValid());
        verifyAllCoordinatesFiniteAndNonZero(wall);
        CHECK(!hasDuplicateSegments(wall.toLineDataList()));

        // Heading +Y: Left is -X (-5), Right is +X (+5)
        CHECK(wall.leftBoundary[0].startpoint.x == Approx(-5.0).margin(TEST_EPS));
        CHECK(wall.leftBoundary[0].startpoint.y == Approx(0.0).margin(TEST_EPS));
        CHECK(wall.leftBoundary[0].endpoint.x == Approx(-5.0).margin(TEST_EPS));
        CHECK(wall.leftBoundary[0].endpoint.y == Approx(100.0).margin(TEST_EPS));

        CHECK(wall.rightBoundary[0].startpoint.x == Approx(5.0).margin(TEST_EPS));
        CHECK(wall.rightBoundary[0].startpoint.y == Approx(0.0).margin(TEST_EPS));
        CHECK(wall.rightBoundary[0].endpoint.x == Approx(5.0).margin(TEST_EPS));
        CHECK(wall.rightBoundary[0].endpoint.y == Approx(100.0).margin(TEST_EPS));
    }

    SECTION("Vertical downward centerline") {
        const std::vector<RS_Vector> center = {{0.0, 100.0}, {0.0, 0.0}};
        const double thickness = 10.0;
        const auto wall = LC_WallMath::generateWall(center, thickness, LC_WallMath::CapType::Flat);
        REQUIRE(wall.isValid());
        verifyAllCoordinatesFiniteAndNonZero(wall);
        CHECK(!hasDuplicateSegments(wall.toLineDataList()));

        // Heading -Y: Left is +X (+5), Right is -X (-5)
        CHECK(wall.leftBoundary[0].startpoint.x == Approx(5.0).margin(TEST_EPS));
        CHECK(wall.leftBoundary[0].startpoint.y == Approx(100.0).margin(TEST_EPS));
        CHECK(wall.leftBoundary[0].endpoint.x == Approx(5.0).margin(TEST_EPS));
        CHECK(wall.leftBoundary[0].endpoint.y == Approx(0.0).margin(TEST_EPS));

        CHECK(wall.rightBoundary[0].startpoint.x == Approx(-5.0).margin(TEST_EPS));
        CHECK(wall.rightBoundary[0].startpoint.y == Approx(100.0).margin(TEST_EPS));
        CHECK(wall.rightBoundary[0].endpoint.x == Approx(-5.0).margin(TEST_EPS));
        CHECK(wall.rightBoundary[0].endpoint.y == Approx(0.0).margin(TEST_EPS));
    }

    SECTION("Diagonal 45-degree centerline") {
        const std::vector<RS_Vector> center = {{0.0, 0.0}, {100.0, 100.0}};
        const double thickness = 10.0;
        const auto wall = LC_WallMath::generateWall(center, thickness, LC_WallMath::CapType::Flat);
        REQUIRE(wall.isValid());
        verifyAllCoordinatesFiniteAndNonZero(wall);
        CHECK(!hasDuplicateSegments(wall.toLineDataList()));

        // Direction: (1/sqrt(2), 1/sqrt(2)). Normal: (-1/sqrt(2), 1/sqrt(2)). Offset magnitude: 5.0.
        const double offsetCoord = 5.0 / std::sqrt(2.0); // ~3.5355339
        CHECK(wall.leftBoundary[0].startpoint.x == Approx(-offsetCoord).margin(TEST_EPS));
        CHECK(wall.leftBoundary[0].startpoint.y == Approx(offsetCoord).margin(TEST_EPS));
        CHECK(wall.leftBoundary[0].endpoint.x == Approx(100.0 - offsetCoord).margin(TEST_EPS));
        CHECK(wall.leftBoundary[0].endpoint.y == Approx(100.0 + offsetCoord).margin(TEST_EPS));

        CHECK(wall.rightBoundary[0].startpoint.x == Approx(offsetCoord).margin(TEST_EPS));
        CHECK(wall.rightBoundary[0].startpoint.y == Approx(-offsetCoord).margin(TEST_EPS));
        CHECK(wall.rightBoundary[0].endpoint.x == Approx(100.0 + offsetCoord).margin(TEST_EPS));
        CHECK(wall.rightBoundary[0].endpoint.y == Approx(100.0 - offsetCoord).margin(TEST_EPS));
    }
}

TEST_CASE("WallMath: 90-degree corner with correct inner and outer joins", "[wall][geometry]") {
    // Left turn (counter-clockwise): (0, 0) -> (100, 0) -> (100, 100)
    const std::vector<RS_Vector> center = {{0.0, 0.0}, {100.0, 0.0}, {100.0, 100.0}};
    const double thickness = 10.0;

    const auto wall = LC_WallMath::generateWall(center, thickness, LC_WallMath::CapType::Flat);
    REQUIRE(wall.isValid());
    verifyAllCoordinatesFiniteAndNonZero(wall);
    CHECK(!hasDuplicateSegments(wall.toLineDataList()));

    // Left is inner corner, Right is outer corner
    REQUIRE(wall.leftBoundary.size() == 2);
    REQUIRE(wall.rightBoundary.size() == 2);

    // Left Inner Miter Point: (95, 5)
    CHECK(wall.leftBoundary[0].startpoint.x == Approx(0.0).margin(TEST_EPS));
    CHECK(wall.leftBoundary[0].startpoint.y == Approx(5.0).margin(TEST_EPS));
    CHECK(wall.leftBoundary[0].endpoint.x == Approx(95.0).margin(TEST_EPS));
    CHECK(wall.leftBoundary[0].endpoint.y == Approx(5.0).margin(TEST_EPS));

    CHECK(wall.leftBoundary[1].startpoint.x == Approx(95.0).margin(TEST_EPS));
    CHECK(wall.leftBoundary[1].startpoint.y == Approx(5.0).margin(TEST_EPS));
    CHECK(wall.leftBoundary[1].endpoint.x == Approx(95.0).margin(TEST_EPS));
    CHECK(wall.leftBoundary[1].endpoint.y == Approx(100.0).margin(TEST_EPS));

    // Right Outer Miter Point: (105, -5)
    CHECK(wall.rightBoundary[0].startpoint.x == Approx(0.0).margin(TEST_EPS));
    CHECK(wall.rightBoundary[0].startpoint.y == Approx(-5.0).margin(TEST_EPS));
    CHECK(wall.rightBoundary[0].endpoint.x == Approx(105.0).margin(TEST_EPS));
    CHECK(wall.rightBoundary[0].endpoint.y == Approx(-5.0).margin(TEST_EPS));

    CHECK(wall.rightBoundary[1].startpoint.x == Approx(105.0).margin(TEST_EPS));
    CHECK(wall.rightBoundary[1].startpoint.y == Approx(-5.0).margin(TEST_EPS));
    CHECK(wall.rightBoundary[1].endpoint.x == Approx(105.0).margin(TEST_EPS));
    CHECK(wall.rightBoundary[1].endpoint.y == Approx(100.0).margin(TEST_EPS));
}

TEST_CASE("WallMath: Clockwise right-angle corner with inverted inner and outer joins", "[wall][geometry]") {
    // Right turn (clockwise): (0, 0) -> (100, 0) -> (100, -100)
    const std::vector<RS_Vector> center = {{0.0, 0.0}, {100.0, 0.0}, {100.0, -100.0}};
    const double thickness = 10.0;

    const auto wall = LC_WallMath::generateWall(center, thickness, LC_WallMath::CapType::Flat);
    REQUIRE(wall.isValid());
    verifyAllCoordinatesFiniteAndNonZero(wall);
    CHECK(!hasDuplicateSegments(wall.toLineDataList()));

    // For a Right turn: Left is OUTER, Right is INNER
    REQUIRE(wall.leftBoundary.size() == 2);
    REQUIRE(wall.rightBoundary.size() == 2);

    // Left Outer Miter Point: (105, 5)
    CHECK(wall.leftBoundary[0].startpoint.x == Approx(0.0).margin(TEST_EPS));
    CHECK(wall.leftBoundary[0].startpoint.y == Approx(5.0).margin(TEST_EPS));
    CHECK(wall.leftBoundary[0].endpoint.x == Approx(105.0).margin(TEST_EPS));
    CHECK(wall.leftBoundary[0].endpoint.y == Approx(5.0).margin(TEST_EPS));

    CHECK(wall.leftBoundary[1].startpoint.x == Approx(105.0).margin(TEST_EPS));
    CHECK(wall.leftBoundary[1].startpoint.y == Approx(5.0).margin(TEST_EPS));
    CHECK(wall.leftBoundary[1].endpoint.x == Approx(105.0).margin(TEST_EPS));
    CHECK(wall.leftBoundary[1].endpoint.y == Approx(-100.0).margin(TEST_EPS));

    // Right Inner Miter Point: (95, -5)
    CHECK(wall.rightBoundary[0].startpoint.x == Approx(0.0).margin(TEST_EPS));
    CHECK(wall.rightBoundary[0].startpoint.y == Approx(-5.0).margin(TEST_EPS));
    CHECK(wall.rightBoundary[0].endpoint.x == Approx(95.0).margin(TEST_EPS));
    CHECK(wall.rightBoundary[0].endpoint.y == Approx(-5.0).margin(TEST_EPS));

    CHECK(wall.rightBoundary[1].startpoint.x == Approx(95.0).margin(TEST_EPS));
    CHECK(wall.rightBoundary[1].startpoint.y == Approx(-5.0).margin(TEST_EPS));
    CHECK(wall.rightBoundary[1].endpoint.x == Approx(95.0).margin(TEST_EPS));
    CHECK(wall.rightBoundary[1].endpoint.y == Approx(-100.0).margin(TEST_EPS));
}

TEST_CASE("WallMath: S-bend with alternating left and right turns", "[wall][geometry]") {
    // S-bend: (0, 0) -> (100, 0) [Left turn] -> (100, 100) [Right turn] -> (200, 100)
    const std::vector<RS_Vector> center = {{0.0, 0.0}, {100.0, 0.0}, {100.0, 100.0}, {200.0, 100.0}};
    const double thickness = 10.0;

    const auto wall = LC_WallMath::generateWall(center, thickness, LC_WallMath::CapType::Flat);
    REQUIRE(wall.isValid());
    verifyAllCoordinatesFiniteAndNonZero(wall);
    CHECK(!hasDuplicateSegments(wall.toLineDataList()));

    REQUIRE(wall.leftBoundary.size() == 3);
    REQUIRE(wall.rightBoundary.size() == 3);

    // Segment 1 (middle vertical segment: x=95 on left, x=105 on right)
    // Left boundary goes from Corner 0 Inner (95, 5) to Corner 1 Outer (95, 105)
    CHECK(wall.leftBoundary[1].startpoint.x == Approx(95.0).margin(TEST_EPS));
    CHECK(wall.leftBoundary[1].startpoint.y == Approx(5.0).margin(TEST_EPS));
    CHECK(wall.leftBoundary[1].endpoint.x == Approx(95.0).margin(TEST_EPS));
    CHECK(wall.leftBoundary[1].endpoint.y == Approx(105.0).margin(TEST_EPS));

    // Right boundary goes from Corner 0 Outer (105, -5) to Corner 1 Inner (105, 95)
    CHECK(wall.rightBoundary[1].startpoint.x == Approx(105.0).margin(TEST_EPS));
    CHECK(wall.rightBoundary[1].startpoint.y == Approx(-5.0).margin(TEST_EPS));
    CHECK(wall.rightBoundary[1].endpoint.x == Approx(105.0).margin(TEST_EPS));
    CHECK(wall.rightBoundary[1].endpoint.y == Approx(95.0).margin(TEST_EPS));
}

TEST_CASE("WallMath: Non-reversal acute corner with restrictive vs permissive miter limit", "[wall][geometry]") {
    // 143-degree turn (interior angle ~37 deg): (0, 0) -> (100, 0) -> (20, 60)
    // Direction S0: (1, 0). Direction S1: (-80/100, 60/100) = (-0.8, 0.6).
    // cross = (1)(0.6) - (0)(-0.8) = 0.6 > 0 (Left Turn).
    // Left is Inner; Right is Outer.
    // miterRatio ~= 1.58.
    const std::vector<RS_Vector> center = {{0.0, 0.0}, {100.0, 0.0}, {20.0, 60.0}};
    const double thickness = 10.0;

    SECTION("Permissive miter limit (2.0) accepts outer miter") {
        LC_WallMath::WallOptions options;
        options.thickness = thickness;
        options.miterLimit = 2.0; // 1.58 <= 2.0 -> Miter used
        options.capType = LC_WallMath::CapType::Flat;

        const auto wall = LC_WallMath::generateWall(center, options);
        REQUIRE(wall.isValid());
        verifyAllCoordinatesFiniteAndNonZero(wall);
        CHECK(!hasDuplicateSegments(wall.toLineDataList()));

        // Both boundaries mitered: exactly 2 segments each, zero bevels
        CHECK(wall.leftBoundary.size() == 2);
        CHECK(wall.rightBoundary.size() == 2);
    }

    SECTION("Restrictive miter limit (1.0) bevels outer boundary while preserving clean inner intersection") {
        LC_WallMath::WallOptions options;
        options.thickness = thickness;
        options.miterLimit = 1.0; // 1.58 > 1.0 -> Outer boundary beveled
        options.capType = LC_WallMath::CapType::Flat;

        const auto wall = LC_WallMath::generateWall(center, options);
        REQUIRE(wall.isValid());
        verifyAllCoordinatesFiniteAndNonZero(wall);
        CHECK(!hasDuplicateSegments(wall.toLineDataList()));

        // Outer boundary (Right) receives bevel line: 3 segments
        CHECK(wall.rightBoundary.size() == 3);

        // Inner boundary (Left) MUST NOT be beveled: exactly 2 segments meeting at inner intersection
        CHECK(wall.leftBoundary.size() == 2);
        CHECK(wall.leftBoundary[0].endpoint.distanceTo(wall.leftBoundary[1].startpoint) < TEST_EPS);
    }
}

TEST_CASE("WallMath: Near-reversal hairpin is explicitly rejected as invalid", "[wall][geometry]") {
    // Hairpin turnaround: (0, 0) -> (100, 0) -> (5, 5) with turn angle ~177 deg >= 170 deg.
    // Folds wall back into its own corridor, producing self-intersecting geometry.
    // Policy: Rejected as invalid.
    const std::vector<RS_Vector> center = {{0.0, 0.0}, {100.0, 0.0}, {5.0, 5.0}};
    const double thickness = 10.0;

    const auto wall = LC_WallMath::generateWall(center, thickness, LC_WallMath::CapType::Flat);
    CHECK(!wall.isValid());
    CHECK(wall.leftBoundary.empty());
    CHECK(wall.rightBoundary.empty());
}

TEST_CASE("WallMath: Exact 180-degree reversal is explicitly rejected as invalid", "[wall][geometry]") {
    // Exact reversal: (0, 0) -> (100, 0) -> (0, 0) with dot = -1.0.
    // Policy: Rejected as invalid to prevent duplicate coincident lines and ambiguous topology.
    const std::vector<RS_Vector> center = {{0.0, 0.0}, {100.0, 0.0}, {0.0, 0.0}};
    const double thickness = 10.0;

    const auto wall = LC_WallMath::generateWall(center, thickness, LC_WallMath::CapType::Flat);
    CHECK(!wall.isValid());
    CHECK(wall.leftBoundary.empty());
    CHECK(wall.rightBoundary.empty());
}
TEST_CASE("WallMath: Short segment whose length is smaller than wall thickness", "[wall][geometry]") {
    // Segment 1 has length 4.0, while wall thickness is 10.0.
    // Consecutive 90-degree inner turns around segment 4.0 (< 10.0) overlap and must be cleanly rejected.
    const std::vector<RS_Vector> center = {{0.0, 0.0}, {100.0, 0.0}, {100.0, 4.0}, {0.0, 4.0}};
    const double thickness = 10.0;

    const auto wall = LC_WallMath::generateWall(center, thickness, LC_WallMath::CapType::Flat);
    CHECK_FALSE(wall.isValid());
    CHECK(wall.leftBoundary.empty());
    CHECK(wall.rightBoundary.empty());
}

TEST_CASE("WallMath: Collinear consecutive segments", "[wall][geometry]") {
    const std::vector<RS_Vector> center = {{0.0, 0.0}, {50.0, 0.0}, {100.0, 0.0}};
    const double thickness = 10.0;

    const auto wall = LC_WallMath::generateWall(center, thickness, LC_WallMath::CapType::Flat);
    REQUIRE(wall.isValid());
    verifyAllCoordinatesFiniteAndNonZero(wall);
    CHECK(!hasDuplicateSegments(wall.toLineDataList()));

    // Exactly 2 segments on each side, seamlessly meeting at x=50, no bevels
    REQUIRE(wall.leftBoundary.size() == 2);
    REQUIRE(wall.rightBoundary.size() == 2);

    CHECK(wall.leftBoundary[0].endpoint.x == Approx(50.0).margin(TEST_EPS));
    CHECK(wall.leftBoundary[0].endpoint.y == Approx(5.0).margin(TEST_EPS));
    CHECK(wall.leftBoundary[1].startpoint.x == Approx(50.0).margin(TEST_EPS));
    CHECK(wall.leftBoundary[1].startpoint.y == Approx(5.0).margin(TEST_EPS));

    CHECK(wall.rightBoundary[0].endpoint.x == Approx(50.0).margin(TEST_EPS));
    CHECK(wall.rightBoundary[0].endpoint.y == Approx(-5.0).margin(TEST_EPS));
    CHECK(wall.rightBoundary[1].startpoint.x == Approx(50.0).margin(TEST_EPS));
    CHECK(wall.rightBoundary[1].startpoint.y == Approx(-5.0).margin(TEST_EPS));
}

TEST_CASE("WallMath: Duplicate points and zero-length segment filtering", "[wall][geometry]") {
    SECTION("Consecutive duplicates are filtered cleanly") {
        const std::vector<RS_Vector> center = {{0.0, 0.0}, {0.0, 0.0}, {100.0, 0.0}, {100.0, 0.0}};
        const double thickness = 10.0;
        const auto wall = LC_WallMath::generateWall(center, thickness, LC_WallMath::CapType::Flat);
        REQUIRE(wall.isValid());
        REQUIRE(wall.leftBoundary.size() == 1);
        REQUIRE(wall.rightBoundary.size() == 1);
        CHECK(wall.leftBoundary[0].endpoint.x == Approx(100.0).margin(TEST_EPS));
    }

    SECTION("All identical points fail cleanly") {
        const std::vector<RS_Vector> center = {{10.0, 10.0}, {10.0, 10.0}};
        const auto wall = LC_WallMath::generateWall(center, 10.0);
        REQUIRE(!wall.isValid());
        REQUIRE(wall.leftBoundary.empty());
        REQUIRE(wall.rightBoundary.empty());
    }
}

TEST_CASE("WallMath: Invalid thickness and non-finite numerical input", "[wall][geometry]") {
    const std::vector<RS_Vector> validCenter = {{0.0, 0.0}, {100.0, 0.0}};

    SECTION("Zero thickness") {
        const auto wall = LC_WallMath::generateWall(validCenter, 0.0);
        CHECK(!wall.isValid());
    }

    SECTION("Negative thickness") {
        const auto wall = LC_WallMath::generateWall(validCenter, -5.0);
        CHECK(!wall.isValid());
    }

    SECTION("NaN thickness") {
        const auto wall = LC_WallMath::generateWall(validCenter, std::numeric_limits<double>::quiet_NaN());
        CHECK(!wall.isValid());
    }

    SECTION("Infinite coordinate in centerline") {
        const std::vector<RS_Vector> badCenter = {{0.0, 0.0}, {std::numeric_limits<double>::infinity(), 0.0}};
        const auto wall = LC_WallMath::generateWall(badCenter, 10.0);
        CHECK(!wall.isValid());
    }

    SECTION("NaN coordinate in centerline") {
        const std::vector<RS_Vector> badCenter = {{0.0, 0.0}, {100.0, std::numeric_limits<double>::quiet_NaN()}};
        const auto wall = LC_WallMath::generateWall(badCenter, 10.0);
        CHECK(!wall.isValid());
    }
}

TEST_CASE("WallMath: Open-path cap modes", "[wall][geometry]") {
    const std::vector<RS_Vector> center = {{0.0, 0.0}, {100.0, 0.0}};

    SECTION("Flat caps") {
        const auto wall = LC_WallMath::generateWall(center, 10.0, LC_WallMath::CapType::Flat);
        REQUIRE(wall.isValid());
        CHECK(wall.caps.size() == 2);
    }

    SECTION("No caps (None)") {
        const auto wall = LC_WallMath::generateWall(center, 10.0, LC_WallMath::CapType::None);
        REQUIRE(wall.isValid());
        CHECK(wall.caps.empty());
    }
}

TEST_CASE("WallMath: Closed-path geometry", "[wall][geometry]") {
    SECTION("Closed triangle") {
        const std::vector<RS_Vector> triangle = {{0.0, 0.0}, {100.0, 0.0}, {50.0, 80.0}};

        LC_WallMath::WallOptions options;
        options.thickness = 10.0;
        options.isClosed = true;

        const auto wall = LC_WallMath::generateWall(triangle, options);
        REQUIRE(wall.isValid());
        verifyAllCoordinatesFiniteAndNonZero(wall);
        CHECK(!hasDuplicateSegments(wall.toLineDataList()));

        // Closed wall has no start/end caps
        CHECK(wall.caps.empty());

        // 3 segments per boundary loop
        REQUIRE(wall.leftBoundary.size() == 3);
        REQUIRE(wall.rightBoundary.size() == 3);

        // Verify left loop continuity: end of segment i connects to start of segment (i + 1) % 3
        for (size_t i = 0; i < 3; ++i) {
            const auto& cur = wall.leftBoundary[i];
            const auto& next = wall.leftBoundary[(i + 1) % 3];
            CHECK(cur.endpoint.distanceTo(next.startpoint) < TEST_EPS);
        }

        // Verify right loop continuity
        for (size_t i = 0; i < 3; ++i) {
            const auto& cur = wall.rightBoundary[i];
            const auto& next = wall.rightBoundary[(i + 1) % 3];
            CHECK(cur.endpoint.distanceTo(next.startpoint) < TEST_EPS);
        }
    }

    SECTION("Closed rectangle") {
        const std::vector<RS_Vector> rect = {{0.0, 0.0}, {100.0, 0.0}, {100.0, 50.0}, {0.0, 50.0}};

        LC_WallMath::WallOptions options;
        options.thickness = 10.0;
        options.isClosed = true;

        const auto wall = LC_WallMath::generateWall(rect, options);
        REQUIRE(wall.isValid());
        verifyAllCoordinatesFiniteAndNonZero(wall);
        CHECK(!hasDuplicateSegments(wall.toLineDataList()));

        CHECK(wall.caps.empty());
        REQUIRE(wall.leftBoundary.size() == 4);
        REQUIRE(wall.rightBoundary.size() == 4);

        for (size_t i = 0; i < 4; ++i) {
            const auto& curL = wall.leftBoundary[i];
            const auto& nextL = wall.leftBoundary[(i + 1) % 4];
            CHECK(curL.endpoint.distanceTo(nextL.startpoint) < TEST_EPS);

            const auto& curR = wall.rightBoundary[i];
            const auto& nextR = wall.rightBoundary[(i + 1) % 4];
            CHECK(curR.endpoint.distanceTo(nextR.startpoint) < TEST_EPS);
        }
    }
}

TEST_CASE("WallMath: 60-degree turn following segment shorter than thickness", "[wall][geometry]") {
    const double thickness = 10.0;

    SECTION("Clockwise (right turn): Left outer boundary falls back to bevel") {
        // Segment 0 length is 4.0 (< 10.0 thickness)
        // 60-degree right turn: dir = (cos(-60), sin(-60)) = (0.5, -sqrt(3)/2)
        const std::vector<RS_Vector> center = {
            {0.0, 0.0},
            {4.0, 0.0},
            {4.0 + 25.0, -43.301270189}
        };

        const auto wall = LC_WallMath::generateWall(center, thickness, LC_WallMath::CapType::Flat);
        REQUIRE(wall.isValid());
        verifyAllCoordinatesFiniteAndNonZero(wall);
        CHECK(!hasDuplicateSegments(wall.toLineDataList()));

        // Left is outer boundary: falls back to bevel (3 segments: seg 0, bevel, seg 1)
        REQUIRE(wall.leftBoundary.size() == 3);
        CHECK(wall.leftBoundary[0].endpoint.distanceTo(wall.leftBoundary[1].startpoint) < TEST_EPS);
        CHECK(wall.leftBoundary[1].endpoint.distanceTo(wall.leftBoundary[2].startpoint) < TEST_EPS);

        // Right is inner boundary: clean miter join (2 segments)
        REQUIRE(wall.rightBoundary.size() == 2);
        CHECK(wall.rightBoundary[0].endpoint.distanceTo(wall.rightBoundary[1].startpoint) < TEST_EPS);

        CHECK(wall.caps.size() == 2);
    }

    SECTION("Counter-clockwise (left turn): Right outer boundary falls back to bevel") {
        // Segment 0 length is 4.0 (< 10.0 thickness)
        // 60-degree left turn: dir = (0.5, sqrt(3)/2)
        const std::vector<RS_Vector> center = {
            {0.0, 0.0},
            {4.0, 0.0},
            {4.0 + 25.0, 43.301270189}
        };

        const auto wall = LC_WallMath::generateWall(center, thickness, LC_WallMath::CapType::Flat);
        REQUIRE(wall.isValid());
        verifyAllCoordinatesFiniteAndNonZero(wall);
        CHECK(!hasDuplicateSegments(wall.toLineDataList()));

        // Right is outer boundary: falls back to bevel (3 segments)
        REQUIRE(wall.rightBoundary.size() == 3);
        CHECK(wall.rightBoundary[0].endpoint.distanceTo(wall.rightBoundary[1].startpoint) < TEST_EPS);
        CHECK(wall.rightBoundary[1].endpoint.distanceTo(wall.rightBoundary[2].startpoint) < TEST_EPS);

        // Left is inner boundary: clean miter join (2 segments)
        REQUIRE(wall.leftBoundary.size() == 2);
        CHECK(wall.leftBoundary[0].endpoint.distanceTo(wall.leftBoundary[1].startpoint) < TEST_EPS);

        CHECK(wall.caps.size() == 2);
    }
}

TEST_CASE("WallMath: Short segment with miter exceeding segment extent falls back to bevel", "[wall][geometry]") {
    // S-bend: (0, 0) -> (100, 0) [Left turn] -> (100, 8.0) [Right turn] -> (200, 8.0)
    // Middle segment has length 8.0, wall thickness is 10.0.
    // Corner 0 inner shift on Left is 5.0 (< 8.0).
    // Corner 1 outer miter displacement on Left (5.0) exceeds available extent (8.0 - 5.0 = 3.0),
    // cleanly triggering bevel fallback on Corner 1 without inverted boundaries.
    const std::vector<RS_Vector> center = {
        {0.0, 0.0},
        {100.0, 0.0},
        {100.0, 8.0},
        {200.0, 8.0}
    };
    const double thickness = 10.0;

    const auto wall = LC_WallMath::generateWall(center, thickness, LC_WallMath::CapType::Flat);
    REQUIRE(wall.isValid());
    verifyAllCoordinatesFiniteAndNonZero(wall);
    CHECK(!hasDuplicateSegments(wall.toLineDataList()));

    // Left boundary receives bevel at Corner 1 (4 segments total: seg 0, seg 1, bevel, seg 2)
    REQUIRE(wall.leftBoundary.size() == 4);
    verifyBoundaryContinuity(wall.leftBoundary, false);

    // Middle segment (index 1) progresses forward
    CHECK((wall.leftBoundary[1].endpoint.y - wall.leftBoundary[1].startpoint.y) > RS_TOLERANCE);

    // Right boundary continuity verified
    REQUIRE(!wall.rightBoundary.empty());
    verifyBoundaryContinuity(wall.rightBoundary, false);

    CHECK(wall.caps.size() == 2);
}

TEST_CASE("WallMath: Normal length segments produce expected miter joins", "[wall][geometry]") {
    // 60-degree turn with normal segment length (100.0 >= 10.0)
    const std::vector<RS_Vector> center = {
        {0.0, 0.0},
        {100.0, 0.0},
        {100.0 + 50.0, 86.602540378}
    };
    const double thickness = 10.0;

    const auto wall = LC_WallMath::generateWall(center, thickness, LC_WallMath::CapType::Flat);
    REQUIRE(wall.isValid());
    verifyAllCoordinatesFiniteAndNonZero(wall);
    CHECK(!hasDuplicateSegments(wall.toLineDataList()));

    // Both boundaries should be cleanly mitered (exactly 2 segments each, zero bevels)
    REQUIRE(wall.leftBoundary.size() == 2);
    REQUIRE(wall.rightBoundary.size() == 2);

    CHECK(wall.rightBoundary[0].endpoint.x == Approx(100.0 + 5.0 / std::sqrt(3.0)).margin(TEST_EPS));
    CHECK(wall.rightBoundary[0].endpoint.y == Approx(-5.0).margin(TEST_EPS));
    CHECK(wall.rightBoundary[1].startpoint.x == Approx(100.0 + 5.0 / std::sqrt(3.0)).margin(TEST_EPS));
    CHECK(wall.rightBoundary[1].startpoint.y == Approx(-5.0).margin(TEST_EPS));

    // Boundary continuity
    verifyBoundaryContinuity(wall.leftBoundary, false);
    verifyBoundaryContinuity(wall.rightBoundary, false);
}

TEST_CASE("WallMath Regression A: Open intermediate short-segment inversion is cleanly rejected", "[wall][regression]") {
    // Audit failure reproduction:
    // (0,0) -> (100,0) -> (100,3) -> (0,3) -> (0,100), thickness 10.0, open wall.
    // In the defective implementation, intermediate segment 1 (length 3.0) inverted into (95,5) -> (95,-2).
    // Hardened implementation rejects this geometry cleanly.
    const std::vector<RS_Vector> center = {
        {0.0, 0.0},
        {100.0, 0.0},
        {100.0, 3.0},
        {0.0, 3.0},
        {0.0, 100.0}
    };
    const double thickness = 10.0;
    const auto wall = LC_WallMath::generateWall(center, thickness, LC_WallMath::CapType::Flat);
    CHECK_FALSE(wall.isValid());
    CHECK(wall.leftBoundary.empty());
    CHECK(wall.rightBoundary.empty());
}

TEST_CASE("WallMath Regression B: Closed rectangle with short initial segment is cleanly rejected", "[wall][regression]") {
    // Audit failure reproduction:
    // (0,0) -> (4,0) -> (4,100) -> (0,100), thickness 10.0, closed wall.
    // In the defective implementation, initial segment 0 (length 4.0) inverted into (5,5) -> (-1,5).
    // Hardened implementation rejects this geometry cleanly.
    const std::vector<RS_Vector> center = {
        {0.0, 0.0},
        {4.0, 0.0},
        {4.0, 100.0},
        {0.0, 100.0}
    };
    LC_WallMath::WallOptions options;
    options.thickness = 10.0;
    options.isClosed = true;

    const auto wall = LC_WallMath::generateWall(center, options);
    CHECK_FALSE(wall.isValid());
    CHECK(wall.leftBoundary.empty());
    CHECK(wall.rightBoundary.empty());
}

TEST_CASE("WallMath Regression C: Both clockwise and counter-clockwise turns around short segments", "[wall][regression]") {
    const double thickness = 10.0;

    SECTION("Counter-clockwise (left turns) around short segment 4.0 < 10.0 -> rejected") {
        const std::vector<RS_Vector> ccw = {
            {0.0, 0.0},
            {100.0, 0.0},
            {100.0, 4.0},
            {0.0, 4.0}
        };
        const auto wall = LC_WallMath::generateWall(ccw, thickness, LC_WallMath::CapType::Flat);
        CHECK_FALSE(wall.isValid());
    }

    SECTION("Clockwise (right turns) around short segment 4.0 < 10.0 -> rejected") {
        const std::vector<RS_Vector> cw = {
            {0.0, 0.0},
            {100.0, 0.0},
            {100.0, -4.0},
            {0.0, -4.0}
        };
        const auto wall = LC_WallMath::generateWall(cw, thickness, LC_WallMath::CapType::Flat);
        CHECK_FALSE(wall.isValid());
    }

    SECTION("Turns around sufficiently long segment (30.0 >= 10.0) -> valid") {
        const std::vector<RS_Vector> validU = {
            {0.0, 0.0},
            {100.0, 0.0},
            {100.0, 30.0},
            {0.0, 30.0}
        };
        const auto wall = LC_WallMath::generateWall(validU, thickness, LC_WallMath::CapType::Flat);
        REQUIRE(wall.isValid());
        verifyAllCoordinatesFiniteAndNonZero(wall);
        CHECK(!hasDuplicateSegments(wall.toLineDataList()));
        verifyBoundaryContinuity(wall.leftBoundary, false);
        verifyBoundaryContinuity(wall.rightBoundary, false);
    }
}

TEST_CASE("WallMath Regression D: Consecutive outer turns exceeding shared segment length fall back to bevel", "[wall][regression]") {
    // S-bend: (0, 0) -> (100, 0) -> (100, 8.0) -> (200, 8.0), thickness 10.0.
    // Middle segment length is 8.0.
    // Corner 0 is Left inner (shift 5.0 < 8.0).
    // Corner 1 is Left outer (proj 5.0 > available 3.0) -> falls back to bevel.
    const std::vector<RS_Vector> center = {
        {0.0, 0.0},
        {100.0, 0.0},
        {100.0, 8.0},
        {200.0, 8.0}
    };
    const double thickness = 10.0;
    const auto wall = LC_WallMath::generateWall(center, thickness, LC_WallMath::CapType::Flat);
    REQUIRE(wall.isValid());
    verifyAllCoordinatesFiniteAndNonZero(wall);
    CHECK(!hasDuplicateSegments(wall.toLineDataList()));
    verifyBoundaryContinuity(wall.leftBoundary, false);
    verifyBoundaryContinuity(wall.rightBoundary, false);

    // Left boundary receives bevel at Corner 1 (4 segments total: seg 0, seg 1, bevel, seg 2)
    REQUIRE(wall.leftBoundary.size() == 4);
    CHECK((wall.leftBoundary[1].endpoint.y - wall.leftBoundary[1].startpoint.y) > RS_TOLERANCE);
}

TEST_CASE("WallMath Regression E: Short first, intermediate, final, and closure segment bounds", "[wall][regression]") {
    const double thickness = 10.0;

    SECTION("Short first segment in open wall where inner miter exceeds segment length") {
        const std::vector<RS_Vector> center = {{0.0, 0.0}, {3.0, 0.0}, {3.0, 100.0}};
        const auto wall = LC_WallMath::generateWall(center, thickness, LC_WallMath::CapType::Flat);
        CHECK_FALSE(wall.isValid());
    }

    SECTION("Short final segment in open wall where inner miter exceeds segment length") {
        const std::vector<RS_Vector> center = {{0.0, 0.0}, {100.0, 0.0}, {100.0, 3.0}};
        const auto wall = LC_WallMath::generateWall(center, thickness, LC_WallMath::CapType::Flat);
        CHECK_FALSE(wall.isValid());
    }

    SECTION("Short intermediate segment in open wall where inner miter exceeds segment length") {
        const std::vector<RS_Vector> center = {
            {0.0, 0.0},
            {100.0, 0.0},
            {100.0, 3.0},
            {200.0, 3.0}
        };
        const auto wall = LC_WallMath::generateWall(center, thickness, LC_WallMath::CapType::Flat);
        CHECK_FALSE(wall.isValid());
    }

    SECTION("Short closure segment in closed wall where inner miter exceeds segment length") {
        const std::vector<RS_Vector> center = {
            {0.0, 0.0},
            {100.0, 0.0},
            {100.0, 100.0},
            {0.0, 4.0}
        };
        LC_WallMath::WallOptions options;
        options.thickness = 10.0;
        options.isClosed = true;
        const auto wall = LC_WallMath::generateWall(center, options);
        CHECK_FALSE(wall.isValid());
    }
}


