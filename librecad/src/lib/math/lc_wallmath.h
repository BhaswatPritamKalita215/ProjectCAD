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

#ifndef LC_WALLMATH_H
#define LC_WALLMATH_H

#include <vector>
#include "rs_vector.h"
#include "rs_line.h" // for RS_LineData

/**
 * @brief Pure algorithmic helper for architectural double-line wall geometry.
 *
 * Provides value-based calculations to generate two wall boundary outlines
 * (left and right offsets), corner miter/bevel joins, and end caps from an
 * ordered centerline.
 *
 * Geometric Corner & Reversal Policy:
 * - Turn classification: Turn direction is classified via the 2D cross product
 *   (uA.x * uB.y - uA.y * uB.x). Positive indicates a Left turn (Left is Inner,
 *   Right is Outer); negative indicates a Right turn (Left is Outer, Right is Inner).
 * - Outer boundary: When the miter ratio exceeds miterLimit, a bevel (chamfer)
 *   join connects the perpendicular offset endpoints to prevent sharp spikes.
 * - Inner boundary: Preserves the natural inner line-line intersection when finite
 *   and within a defensible safety bound (<= 10 * thickness), avoiding inner bevel
 *   fish-tail loops.
 * - Same-direction collinear segments: Merged seamlessly at common vertices without
 *   redundant bevel segments.
 * - Reversals & hairpins: Paths with turnaround angles >= 170 degrees (interior
 *   angle <= 10 degrees) fold back into their own corridor, producing self-intersecting
 *   and topologically ambiguous boundaries. These are rejected as invalid (returns
 *   empty WallGeometry).
 *
 * This class is completely stateless, performs zero heap allocations of RS_Entity,
 * and has no dependency on UI widgets, action state machines, or document transactions.
 */
class LC_WallMath {
public:
    enum class CapType {
        None,   /**< Open path without end caps */
        Flat    /**< Flat caps connecting left and right boundaries at start and end */
    };

    struct WallOptions {
        double thickness = 10.0;
        /**
         * @brief Maximum miter ratio: distance(corner, miter_tip) / thickness.
         * Applied strictly to the OUTER boundary of a turn. If the ratio exceeds
         * this limit, the sharp outer corner is beveled (chamfered).
         */
        double miterLimit = 2.0;
        CapType capType = CapType::Flat;
        bool isClosed = false;
    };

    struct WallGeometry {
        std::vector<RS_LineData> leftBoundary;
        std::vector<RS_LineData> rightBoundary;
        std::vector<RS_LineData> caps;

        /**
         * @brief Combines all boundary and cap lines into a single list of line data.
         * Appends leftBoundary, rightBoundary, and caps in that order.
         * To form a continuous closed perimeter polygon, traverse:
         * startCap -> leftBoundary -> endCap -> reversed(rightBoundary).
         */
        std::vector<RS_LineData> toLineDataList() const {
            std::vector<RS_LineData> result;
            result.reserve(leftBoundary.size() + rightBoundary.size() + caps.size());
            result.insert(result.end(), leftBoundary.begin(), leftBoundary.end());
            result.insert(result.end(), rightBoundary.begin(), rightBoundary.end());
            result.insert(result.end(), caps.begin(), caps.end());
            return result;
        }

        /**
         * @brief Validates geometric integrity of generated wall boundaries.
         *
         * Checks:
         * 1. Non-empty boundaries (left and right).
         * 2. All coordinates are finite and valid numbers.
         * 3. No zero-length or near-zero-length edges (length > RS_TOLERANCE).
         * 4. Sequential endpoint continuity along each boundary.
         * 5. Closure continuity when applicable.
         * 6. No internal self-intersections or boundary crossovers.
         *
         * Note: Directional progression along corresponding centerline segments
         * is enforced by LC_WallMath::generateWall() during boundary synthesis.
         */
        bool isValid() const;
    };

    /**
     * @brief Generates double-line wall boundaries from an ordered centerline path.
     *
     * @param centerlines Ordered vertices of the centerline.
     * @param options Configuration options (thickness, miter limit, caps, closed).
     * @return WallGeometry containing boundary and cap lines, or an invalid WallGeometry
     *         if input parameters are invalid (e.g. non-positive thickness, non-finite
     *         coordinates, reversals/hairpins >= 170 deg, fewer than 2 distinct points
     *         for open path or 3 for closed path).
     */
    static WallGeometry generateWall(const std::vector<RS_Vector>& centerlines,
                                     const WallOptions& options);

    /**
     * @brief Convenience overload for generating wall with default miter limit.
     */
    static WallGeometry generateWall(const std::vector<RS_Vector>& centerlines,
                                     double thickness,
                                     CapType capType = CapType::Flat,
                                     double miterLimit = 2.0);

    /**
     * @brief Filters consecutive duplicate points (within RS_TOLERANCE) and verifies
     *        coordinate finiteness.
     *
     * @param points Input vertices.
     * @param isClosed Whether the path is intended to be closed (drops duplicate closing vertex).
     * @return Filtered vertices, or empty vector if any coordinate is non-finite or invalid.
     */
    static std::vector<RS_Vector> sanitizeCenterline(const std::vector<RS_Vector>& points,
                                                      bool isClosed);

    /**
     * @brief Computes 2D unit normal pointing to the left of the directed segment (start -> end).
     *
     * Coordinate convention: for start (x1, y1) and end (x2, y2) with length L,
     * normal is (-(y2 - y1)/L, (x2 - x1)/L).
     *
     * @return Unit normal, or RS_Vector(false) if segment length is zero or non-finite.
     */
    static RS_Vector getSegmentNormal(const RS_Vector& start, const RS_Vector& end);

    /**
     * @brief Computes intersection of two infinite 2D lines.
     * Line 1 passes through p1 in direction dir1.
     * Line 2 passes through p2 in direction dir2.
     *
     * @return Intersection point, or RS_Vector(false) if lines are parallel within RS_TOLERANCE.
     */
    static RS_Vector getLineIntersection(const RS_Vector& p1, const RS_Vector& dir1,
                                         const RS_Vector& p2, const RS_Vector& dir2);
};

#endif // LC_WALLMATH_H
