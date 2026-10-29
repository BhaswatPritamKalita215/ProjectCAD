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

#include "lc_wallmath.h"

#include <cmath>
#include <algorithm>
#include "rs.h"

namespace {

// Angular tolerance threshold for collinear segment alignment.
// Derived from: cos(theta) ~= 1 - theta^2 / 2.
// For theta ~= sqrt(2 * RS_TOLERANCE_ANGLE) ~= 1.414e-4 rad (~0.0081 deg),
// 1.0 - dot < RS_TOLERANCE_ANGLE (where RS_TOLERANCE_ANGLE = 1.0e-8).
constexpr double COLLINEAR_DOT_EPSILON = RS_TOLERANCE_ANGLE;

// Threshold for detecting hairpin turnarounds and reversals.
// A turn sharper than 170 degrees (interior angle < 10 degrees) folds the wall back
// into its own corridor, producing self-intersecting geometry and ambiguous boundary topology.
// cos(170 deg) = -0.984807753012208
constexpr double REVERSAL_DOT_THRESHOLD = -0.984807753012208;

// Maximum defensible inner miter distance multiplier: innerDist / thickness.
// Prevents numerical overflow on near-parallel inner intersections.
constexpr double MAX_INNER_MITER_RATIO = 10.0;

struct SegmentInfo {
    RS_Vector start;
    RS_Vector end;
    RS_Vector dir;    // Unit direction
    RS_Vector normal; // Unit normal pointing to the left of travel direction
    double length = 0.0;
};

struct CornerJoinCandidate {
    bool isCollinear = false;
    bool isOuter = false;
    bool isInner = false;
    RS_Vector miter;
    double miterDist = 0.0;
    double shiftA = 0.0; // Inner shift backward into incoming segment A
    double shiftB = 0.0; // Inner shift forward into outgoing segment B
    double projA = 0.0;  // Outer projection forward past incoming segment A
    double projB = 0.0;  // Outer projection backward before outgoing segment B
    bool canMiter = false;
};

struct BoundaryJoin {
    bool hasBevel = false;
    RS_Vector bevelStart;
    RS_Vector bevelEnd;
};

bool isSafeOuterMiter(const RS_Vector& miter,
                      const RS_Vector& cornerVertex,
                      const RS_Vector& pA,
                      const RS_Vector& pB,
                      const SegmentInfo& segA,
                      const SegmentInfo& segB,
                      double thickness,
                      double miterLimit,
                      double availExtentA,
                      double availExtentB) {
    if (!miter.isValid() || !std::isfinite(miter.x) || !std::isfinite(miter.y)) {
        return false;
    }
    const double miterDist = cornerVertex.distanceTo(miter);
    const double miterRatio = miterDist / thickness;
    if (miterRatio > miterLimit) {
        return false;
    }

    const double projA = (miter - pA).dotP(segA.dir);
    const double projB = (pB - miter).dotP(segB.dir);

    if (projA < -RS_TOLERANCE || projB < -RS_TOLERANCE) {
        return false;
    }

    if (projA > availExtentA || (segA.length < thickness && miterDist > segA.length)) {
        return false;
    }
    if (projB > availExtentB || (segB.length < thickness && miterDist > segB.length)) {
        return false;
    }

    return true;
}

bool isSafeInnerMiter(const RS_Vector& miter,
                      const RS_Vector& cornerVertex,
                      const RS_Vector& pA,
                      const RS_Vector& pB,
                      const SegmentInfo& segA,
                      const SegmentInfo& segB,
                      double thickness,
                      double availExtentA,
                      double availExtentB) {
    if (!miter.isValid() || !std::isfinite(miter.x) || !std::isfinite(miter.y)) {
        return false;
    }
    const double innerDist = cornerVertex.distanceTo(miter);
    if (innerDist > MAX_INNER_MITER_RATIO * thickness) {
        return false;
    }
    const double shiftA = (pA - miter).dotP(segA.dir);
    const double shiftB = (miter - pB).dotP(segB.dir);

    if (shiftA < -RS_TOLERANCE || shiftB < -RS_TOLERANCE) {
        return false;
    }

    if (shiftA >= availExtentA - RS_TOLERANCE || shiftB >= availExtentB - RS_TOLERANCE) {
        return false;
    }

    return true;
}

} // anonymous namespace

bool LC_WallMath::WallGeometry::isValid() const {
    if (leftBoundary.empty() || rightBoundary.empty()) {
        return false;
    }

    // 1. Check all coordinates are finite, valid, and segments are non-degenerate
    auto checkLines = [](const std::vector<RS_LineData>& lines) -> bool {
        for (const auto& l : lines) {
            if (!l.startpoint.isValid() || !l.endpoint.isValid() ||
                !std::isfinite(l.startpoint.x) || !std::isfinite(l.startpoint.y) ||
                !std::isfinite(l.endpoint.x) || !std::isfinite(l.endpoint.y)) {
                return false;
            }
            if (l.startpoint.distanceTo(l.endpoint) <= RS_TOLERANCE) {
                return false;
            }
        }
        return true;
    };

    if (!checkLines(leftBoundary) || !checkLines(rightBoundary) || !checkLines(caps)) {
        return false;
    }

    // 2. Sequential endpoint continuity along each boundary
    for (size_t i = 0; i + 1 < leftBoundary.size(); ++i) {
        if (leftBoundary[i].endpoint.distanceTo(leftBoundary[i + 1].startpoint) > RS_TOLERANCE) {
            return false;
        }
    }
    for (size_t i = 0; i + 1 < rightBoundary.size(); ++i) {
        if (rightBoundary[i].endpoint.distanceTo(rightBoundary[i + 1].startpoint) > RS_TOLERANCE) {
            return false;
        }
    }

    // 3. For closed boundary loops, verify closure continuity
    if (caps.empty()) {
        const bool leftCloses = leftBoundary.back().endpoint.distanceTo(leftBoundary.front().startpoint) <= RS_TOLERANCE;
        const bool rightCloses = rightBoundary.back().endpoint.distanceTo(rightBoundary.front().startpoint) <= RS_TOLERANCE;
        if (leftCloses != rightCloses) {
            return false;
        }
    }

    // 4. Proper segment intersection helper to detect self-intersections and crossovers
    auto segmentsIntersect = [](const RS_Vector& p1, const RS_Vector& p2,
                                const RS_Vector& p3, const RS_Vector& p4) -> bool {
        const double minX12 = std::min(p1.x, p2.x) - RS_TOLERANCE;
        const double maxX12 = std::max(p1.x, p2.x) + RS_TOLERANCE;
        const double minX34 = std::min(p3.x, p4.x) - RS_TOLERANCE;
        const double maxX34 = std::max(p3.x, p4.x) + RS_TOLERANCE;
        if (maxX12 < minX34 || maxX34 < minX12) return false;

        const double minY12 = std::min(p1.y, p2.y) - RS_TOLERANCE;
        const double maxY12 = std::max(p1.y, p2.y) + RS_TOLERANCE;
        const double minY34 = std::min(p3.y, p4.y) - RS_TOLERANCE;
        const double maxY34 = std::max(p3.y, p4.y) + RS_TOLERANCE;
        if (maxY12 < minY34 || maxY34 < minY12) return false;

        auto orient = [](const RS_Vector& a, const RS_Vector& b, const RS_Vector& c) -> double {
            return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
        };

        const double o1 = orient(p1, p2, p3);
        const double o2 = orient(p1, p2, p4);
        const double o3 = orient(p3, p4, p1);
        const double o4 = orient(p3, p4, p2);

        if (((o1 > RS_TOLERANCE && o2 < -RS_TOLERANCE) || (o1 < -RS_TOLERANCE && o2 > RS_TOLERANCE)) &&
            ((o3 > RS_TOLERANCE && o4 < -RS_TOLERANCE) || (o3 < -RS_TOLERANCE && o4 > RS_TOLERANCE))) {
            return true;
        }
        return false;
    };

    // 5. Check boundary self-intersection and crossover
    if (caps.empty()) {
        // For closed paths, neither boundary loop may self-intersect
        for (size_t i = 0; i < leftBoundary.size(); ++i) {
            for (size_t j = i + 2; j < leftBoundary.size(); ++j) {
                if (i == 0 && j == leftBoundary.size() - 1) {
                    continue;
                }
                if (segmentsIntersect(leftBoundary[i].startpoint, leftBoundary[i].endpoint,
                                      leftBoundary[j].startpoint, leftBoundary[j].endpoint)) {
                    return false;
                }
            }
        }

        for (size_t i = 0; i < rightBoundary.size(); ++i) {
            for (size_t j = i + 2; j < rightBoundary.size(); ++j) {
                if (i == 0 && j == rightBoundary.size() - 1) {
                    continue;
                }
                if (segmentsIntersect(rightBoundary[i].startpoint, rightBoundary[i].endpoint,
                                      rightBoundary[j].startpoint, rightBoundary[j].endpoint)) {
                    return false;
                }
            }
        }

        // Inner and outer boundary loops must not cross each other
        for (const auto& l : leftBoundary) {
            for (const auto& r : rightBoundary) {
                if (segmentsIntersect(l.startpoint, l.endpoint, r.startpoint, r.endpoint)) {
                    return false;
                }
            }
        }
    } else {
        // For open paths, check that corresponding segment boundaries do not cross over
        const size_t nSegs = std::min(leftBoundary.size(), rightBoundary.size());
        for (size_t i = 0; i < nSegs; ++i) {
            if (segmentsIntersect(leftBoundary[i].startpoint, leftBoundary[i].endpoint,
                                  rightBoundary[i].startpoint, rightBoundary[i].endpoint)) {
                return false;
            }
        }
    }

    return true;
}

std::vector<RS_Vector> LC_WallMath::sanitizeCenterline(const std::vector<RS_Vector>& points,
                                                       bool isClosed) {
    std::vector<RS_Vector> result;
    result.reserve(points.size());

    for (const auto& pt : points) {
        if (!pt.isValid() || !std::isfinite(pt.x) || !std::isfinite(pt.y)) {
            return {};
        }
        if (result.empty() || result.back().distanceTo(pt) > RS_TOLERANCE) {
            result.push_back(pt);
        }
    }

    if (isClosed) {
        if (result.size() >= 2 && result.front().distanceTo(result.back()) <= RS_TOLERANCE) {
            result.pop_back();
        }
        if (result.size() < 3) {
            return {};
        }
    } else {
        if (result.size() < 2) {
            return {};
        }
    }

    return result;
}

RS_Vector LC_WallMath::getSegmentNormal(const RS_Vector& start, const RS_Vector& end) {
    if (!start.isValid() || !end.isValid()) {
        return RS_Vector(false);
    }
    const RS_Vector v = end - start;
    const double len = v.magnitude();
    if (!std::isfinite(len) || len <= RS_TOLERANCE) {
        return RS_Vector(false);
    }
    // Perpendicular vector to the left of travel direction: (-dy/L, dx/L)
    return RS_Vector(-v.y / len, v.x / len, 0.0);
}

RS_Vector LC_WallMath::getLineIntersection(const RS_Vector& p1, const RS_Vector& dir1,
                                          const RS_Vector& p2, const RS_Vector& dir2) {
    const double det = dir1.x * dir2.y - dir1.y * dir2.x;
    if (std::abs(det) <= RS_TOLERANCE) {
        return RS_Vector(false); // Lines are parallel or collinear
    }

    const double t = ((p2.x - p1.x) * dir2.y - (p2.y - p1.y) * dir2.x) / det;
    const RS_Vector result = p1 + dir1 * t;

    if (!std::isfinite(result.x) || !std::isfinite(result.y)) {
        return RS_Vector(false);
    }
    return result;
}

LC_WallMath::WallGeometry LC_WallMath::generateWall(const std::vector<RS_Vector>& centerlines,
                                                   const WallOptions& options) {
    if (!std::isfinite(options.thickness) || options.thickness <= RS_TOLERANCE) {
        return WallGeometry{};
    }

    double miterLimit = options.miterLimit;
    if (!std::isfinite(miterLimit) || miterLimit < 1.0) {
        miterLimit = 2.0;
    }

    const double d = options.thickness / 2.0;
    const std::vector<RS_Vector> pts = sanitizeCenterline(centerlines, options.isClosed);
    if (pts.empty()) {
        return WallGeometry{};
    }

    const size_t N = pts.size();
    const size_t M = options.isClosed ? N : (N - 1);
    if (M == 0) {
        return WallGeometry{};
    }

    std::vector<SegmentInfo> segs(M);
    for (size_t i = 0; i < M; ++i) {
        segs[i].start = pts[i];
        segs[i].end = pts[(i + 1) % N];
        const RS_Vector v = segs[i].end - segs[i].start;
        segs[i].length = v.magnitude();
        if (segs[i].length <= RS_TOLERANCE || !std::isfinite(segs[i].length)) {
            return WallGeometry{};
        }
        segs[i].dir = v / segs[i].length;
        segs[i].normal = RS_Vector(-segs[i].dir.y, segs[i].dir.x, 0.0);
    }

    // Number of corner joins to evaluate
    const size_t K = options.isClosed ? M : (M - 1);

    // Reversal / hairpin detection: reject self-overlapping corridors
    for (size_t k = 0; k < K; ++k) {
        const size_t A = k;
        const size_t B = (k + 1) % M;
        const double dot = segs[A].dir.dotP(segs[B].dir);
        if (dot <= REVERSAL_DOT_THRESHOLD) {
            return WallGeometry{};
        }
    }

    WallGeometry result;

    // Helper lambda to construct a boundary (Left or Right)
    auto buildBoundary = [&](bool isLeft) -> bool {
        const double offsetDist = isLeft ? d : -d;
        std::vector<RS_Vector> rawStart(M), rawEnd(M);
        for (size_t i = 0; i < M; ++i) {
            rawStart[i] = segs[i].start + segs[i].normal * offsetDist;
            rawEnd[i]   = segs[i].end   + segs[i].normal * offsetDist;
        }

        std::vector<CornerJoinCandidate> cand(K);

        // Pass 1: Local Corner Join Evaluation
        for (size_t k = 0; k < K; ++k) {
            const size_t A = k;
            const size_t B = (k + 1) % M;
            const RS_Vector cornerVertex = segs[A].end;
            const double dot = segs[A].dir.dotP(segs[B].dir);
            const double cross = segs[A].dir.x * segs[B].dir.y - segs[A].dir.y * segs[B].dir.x;

            if ((1.0 - dot) < COLLINEAR_DOT_EPSILON) {
                cand[k].isCollinear = true;
                cand[k].canMiter = true;
                continue;
            }

            const bool isLeftTurn = (cross > 0.0);
            cand[k].isOuter = isLeft ? !isLeftTurn : isLeftTurn;
            cand[k].isInner = isLeft ? isLeftTurn : !isLeftTurn;

            const RS_Vector miter = getLineIntersection(rawEnd[A], segs[A].dir, rawStart[B], segs[B].dir);
            if (!miter.isValid() || !std::isfinite(miter.x) || !std::isfinite(miter.y)) {
                if (cand[k].isInner) {
                    return false; // Invalid inner miter cannot be repaired
                }
                cand[k].canMiter = false;
                continue;
            }

            cand[k].miter = miter;
            cand[k].miterDist = cornerVertex.distanceTo(miter);

            if (cand[k].isOuter) {
                cand[k].projA = (miter - rawEnd[A]).dotP(segs[A].dir);
                cand[k].projB = (rawStart[B] - miter).dotP(segs[B].dir);
                const double miterRatio = cand[k].miterDist / options.thickness;
                if (miterRatio > miterLimit || cand[k].projA < -RS_TOLERANCE || cand[k].projB < -RS_TOLERANCE) {
                    cand[k].canMiter = false;
                } else {
                    cand[k].canMiter = true;
                }
            } else { // Inner corner
                cand[k].shiftA = (rawEnd[A] - miter).dotP(segs[A].dir);
                cand[k].shiftB = (miter - rawStart[B]).dotP(segs[B].dir);
                if (cand[k].miterDist > MAX_INNER_MITER_RATIO * options.thickness ||
                    cand[k].shiftA < -RS_TOLERANCE || cand[k].shiftB < -RS_TOLERANCE ||
                    cand[k].shiftA >= segs[A].length - RS_TOLERANCE ||
                    cand[k].shiftB >= segs[B].length - RS_TOLERANCE) {
                    return false; // Inner shift exceeds segment or safety bound -> reject
                }
                cand[k].canMiter = true;
            }
        }

        // Pass 2: Global Segment Extent Budgeting across all segments
        for (size_t i = 0; i < M; ++i) {
            int k_in = -1;
            int k_out = -1;
            if (options.isClosed) {
                k_in = static_cast<int>((i + M - 1) % M);
                k_out = static_cast<int>(i);
            } else {
                if (i > 0) k_in = static_cast<int>(i - 1);
                if (i < M - 1) k_out = static_cast<int>(i);
            }

            // Both neighboring joins are Inner turns
            if (k_in >= 0 && k_out >= 0 && cand[k_in].isInner && cand[k_out].isInner) {
                const double totalInnerShift = cand[k_in].shiftB + cand[k_out].shiftA;
                if (totalInnerShift >= segs[i].length - RS_TOLERANCE) {
                    return false; // Inner turns collide and reverse the segment -> reject cleanly
                }
            }

            // Both neighboring joins are Outer turns
            if (k_in >= 0 && k_out >= 0 && cand[k_in].isOuter && cand[k_out].isOuter) {
                if (cand[k_in].canMiter && cand[k_out].canMiter) {
                    const double totalOuterProj = cand[k_in].projB + cand[k_out].projA;
                    if (totalOuterProj > segs[i].length ||
                        cand[k_in].projB > segs[i].length ||
                        cand[k_out].projA > segs[i].length ||
                        (segs[i].length < options.thickness &&
                         (cand[k_in].miterDist > segs[i].length || cand[k_out].miterDist > segs[i].length))) {
                        // Combined or individual projections exceed shared segment extent -> fallback to bevel
                        cand[k_in].canMiter = false;
                        cand[k_out].canMiter = false;
                    }
                }
            }

            // Incoming is Outer, Outgoing is Inner
            if (k_in >= 0 && cand[k_in].isOuter && cand[k_in].canMiter) {
                const double availOnSeg = (k_out >= 0 && cand[k_out].isInner)
                                              ? (segs[i].length - cand[k_out].shiftA)
                                              : segs[i].length;
                if (cand[k_in].projB > availOnSeg ||
                    (segs[i].length < options.thickness && cand[k_in].miterDist > segs[i].length)) {
                    cand[k_in].canMiter = false;
                }
            }

            // Incoming is Inner, Outgoing is Outer
            if (k_out >= 0 && cand[k_out].isOuter && cand[k_out].canMiter) {
                const double availOnSeg = (k_in >= 0 && cand[k_in].isInner)
                                              ? (segs[i].length - cand[k_in].shiftB)
                                              : segs[i].length;
                if (cand[k_out].projA > availOnSeg ||
                    (segs[i].length < options.thickness && cand[k_out].miterDist > segs[i].length)) {
                    cand[k_out].canMiter = false;
                }
            }

            // Open path terminal outer miter extent checks
            if (!options.isClosed) {
                if (i == 0 && k_out >= 0 && cand[k_out].isOuter && cand[k_out].canMiter) {
                    if (cand[k_out].projA > segs[0].length ||
                        (segs[0].length < options.thickness && cand[k_out].miterDist > segs[0].length)) {
                        cand[k_out].canMiter = false;
                    }
                }
                if (i == M - 1 && k_in >= 0 && cand[k_in].isOuter && cand[k_in].canMiter) {
                    if (cand[k_in].projB > segs[M - 1].length ||
                        (segs[M - 1].length < options.thickness && cand[k_in].miterDist > segs[M - 1].length)) {
                        cand[k_in].canMiter = false;
                    }
                }
            }
        }

        // Pass 3: Finalize Boundary Positions and Emitted Edges
        std::vector<RS_Vector> b_start(M), b_end(M);
        std::vector<BoundaryJoin> joins(K);

        for (size_t k = 0; k < K; ++k) {
            const size_t A = k;
            const size_t B = (k + 1) % M;

            if (cand[k].isCollinear) {
                b_end[A] = rawEnd[A];
                b_start[B] = rawStart[B];
            } else if (cand[k].canMiter) {
                b_end[A] = cand[k].miter;
                b_start[B] = cand[k].miter;
            } else if (cand[k].isOuter) {
                // Bevel fallback on outer turn
                b_end[A] = rawEnd[A];
                b_start[B] = rawStart[B];
                joins[k].hasBevel = true;
                joins[k].bevelStart = rawEnd[A];
                joins[k].bevelEnd = rawStart[B];
            } else {
                // Inner turn cannot be repaired without fish-tail self-intersection -> reject
                return false;
            }
        }

        if (!options.isClosed) {
            b_start[0] = rawStart[0];
            b_end[M - 1] = rawEnd[M - 1];
        }

        // Enforce per-segment progression invariant along centerline direction
        for (size_t i = 0; i < M; ++i) {
            const double prog = (b_end[i] - b_start[i]).dotP(segs[i].dir);
            if (prog <= RS_TOLERANCE || b_start[i].distanceTo(b_end[i]) <= RS_TOLERANCE) {
                return false;
            }
        }

        // Validate bevel edges
        for (size_t k = 0; k < K; ++k) {
            if (joins[k].hasBevel) {
                if (joins[k].bevelStart.distanceTo(joins[k].bevelEnd) <= RS_TOLERANCE) {
                    return false;
                }
            }
        }

        // Assemble into output boundary
        auto& boundaryVec = isLeft ? result.leftBoundary : result.rightBoundary;
        boundaryVec.clear();
        for (size_t i = 0; i < M; ++i) {
            boundaryVec.emplace_back(b_start[i], b_end[i]);
            if (i < K && joins[i].hasBevel) {
                boundaryVec.emplace_back(joins[i].bevelStart, joins[i].bevelEnd);
            }
        }

        return true;
    };

    if (!buildBoundary(true)) {
        return WallGeometry{};
    }
    if (!buildBoundary(false)) {
        return WallGeometry{};
    }

    // Open path start and end flat caps
    if (!options.isClosed && options.capType == CapType::Flat) {
        const RS_Vector& rStart = result.rightBoundary.front().startpoint;
        const RS_Vector& lStart = result.leftBoundary.front().startpoint;
        if (rStart.distanceTo(lStart) > RS_TOLERANCE) {
            result.caps.emplace_back(rStart, lStart);
        } else {
            return WallGeometry{};
        }

        const RS_Vector& lEnd = result.leftBoundary.back().endpoint;
        const RS_Vector& rEnd = result.rightBoundary.back().endpoint;
        if (lEnd.distanceTo(rEnd) > RS_TOLERANCE) {
            result.caps.emplace_back(lEnd, rEnd);
        } else {
            return WallGeometry{};
        }
    }

    // Final comprehensive geometry validation
    if (!result.isValid()) {
        return WallGeometry{};
    }

    return result;
}

LC_WallMath::WallGeometry LC_WallMath::generateWall(const std::vector<RS_Vector>& centerlines,
                                                   double thickness,
                                                   CapType capType,
                                                   double miterLimit) {
    WallOptions opt;
    opt.thickness = thickness;
    opt.capType = capType;
    opt.miterLimit = miterLimit;
    opt.isClosed = false;
    return generateWall(centerlines, opt);
}
