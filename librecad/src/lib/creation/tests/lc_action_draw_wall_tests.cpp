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
#include <memory>
#include <QKeyEvent>

#include "lc_action_draw_wall.h"
#include "lc_actiontestsupport.h"
#include "rs_commands.h"
#include "rs_graphic.h"
#include "rs_line.h"
#include "rs_preview.h"

using Catch::Approx;

namespace {

class WallActionProbe final : public LC_ActionDrawWall {
public:
    explicit WallActionProbe(LC_ActionContext* actionContext)
        : LC_ActionDrawWall(actionContext) {}

    using LC_ActionDrawWall::doProcessCommand;
    using LC_ActionDrawWall::onCoordinateEvent;
    using LC_ActionDrawWall::doPreparePreviewEntities;
    using LC_ActionDrawWall::doCheckMayDrawPreview;
    using LC_ActionDrawWall::doBack;
    using LC_ActionDrawWall::completeWall;
    using LC_ActionDrawWall::undoPoint;
    using LC_ActionDrawWall::m_preview;
};

struct WallActionFixture {
    const bool m_qtReady{lc::test::application() != nullptr};
    RS_Graphic m_graphic;
    lc::test::TestGraphicView m_view;
    LC_ActionContext m_context;
    std::shared_ptr<WallActionProbe> m_action;

    WallActionFixture() {
        m_graphic.initForNewDocument();
        m_view.setDocument(&m_graphic);
        m_context.setDocumentAndView(&m_graphic, &m_view);
        m_action = std::make_shared<WallActionProbe>(&m_context);
        m_action->init(LC_ActionDrawWall::SetStartPoint);
    }

    ~WallActionFixture() {
        m_view.beginClose();
    }

    size_t countAlive() const {
        size_t count = 0;
        for (const auto* e : m_graphic) {
            if (e != nullptr && e->isAlive()) {
                ++count;
            }
        }
        return count;
    }
};

} // anonymous namespace

TEST_CASE("Wall command registration and aliases", "[wall][action]") {
    WallActionFixture fixture;
    CHECK(RS_COMMANDS->cmdToAction("wall") == RS2::ActionDrawWall);
    CHECK(RS_COMMANDS->cmdToAction("wa") == RS2::ActionDrawWall);
}

TEST_CASE("Wall preview generation and live updates", "[wall][action]") {
    WallActionFixture fixture;
    auto& action = *fixture.m_action;

    // Initially at SetStartPoint: no preview
    CHECK(action.getStatus() == LC_ActionDrawWall::SetStartPoint);
    CHECK(!action.doCheckMayDrawPreview(nullptr, action.getStatus()));

    // Place first point (0, 0)
    action.onCoordinateEvent(action.getStatus(), false, RS_Vector{0.0, 0.0});
    CHECK(action.getStatus() == LC_ActionDrawWall::SetNextPoint);
    CHECK(action.getCenterlinePoints().size() == 1);
    CHECK(action.doCheckMayDrawPreview(nullptr, action.getStatus()));

    // Query preview with candidate second point (100, 0)
    QList<RS_Entity*> previewList;
    RS_Vector snap{100.0, 0.0};
    action.doPreparePreviewEntities(nullptr, snap, previewList, action.getStatus());

    REQUIRE(action.isLastPreviewValid());
    // Straight open wall preview should contain 2 boundary segments + 2 flat caps = 4 entities
    REQUIRE(previewList.size() == 4);

    for (const auto* e : previewList) {
        REQUIRE(e != nullptr);
        CHECK(e->rtti() == RS2::EntityLine);
    }

    // Clean up preview heap allocations (in real app, RS_Preview adopts them)
    qDeleteAll(previewList);
}

TEST_CASE("Wall Backspace removes last placed point and resets state", "[wall][action]") {
    WallActionFixture fixture;
    auto& action = *fixture.m_action;

    // Place three points: (0, 0), (100, 0), (100, 100)
    action.onCoordinateEvent(action.getStatus(), false, RS_Vector{0.0, 0.0});
    action.onCoordinateEvent(action.getStatus(), false, RS_Vector{100.0, 0.0});
    action.onCoordinateEvent(action.getStatus(), false, RS_Vector{100.0, 100.0});
    REQUIRE(action.getCenterlinePoints().size() == 3);
    CHECK(action.getStatus() == LC_ActionDrawWall::SetNextPoint);

    // 1st Backspace: drops (100, 100) -> 2 points remain
    QKeyEvent bs1(QEvent::KeyPress, Qt::Key_Backspace, Qt::NoModifier);
    action.keyPressEvent(&bs1);
    CHECK(action.getCenterlinePoints().size() == 2);
    CHECK(action.getStatus() == LC_ActionDrawWall::SetNextPoint);
    CHECK(action.getCenterlinePoints().back().distanceTo(RS_Vector{100.0, 0.0}) < 1e-6);

    // 2nd Backspace: drops (100, 0) -> 1 point remains
    action.keyPressEvent(&bs1);
    CHECK(action.getCenterlinePoints().size() == 1);
    CHECK(action.getStatus() == LC_ActionDrawWall::SetNextPoint);
    CHECK(action.getCenterlinePoints().back().distanceTo(RS_Vector{0.0, 0.0}) < 1e-6);

    // 3rd Backspace: drops (0, 0) -> 0 points, resets to SetStartPoint
    action.keyPressEvent(&bs1);
    CHECK(action.getCenterlinePoints().empty());
    CHECK(action.getStatus() == LC_ActionDrawWall::SetStartPoint);
}

TEST_CASE("Wall invalid hairpin and reversal rejection", "[wall][action]") {
    WallActionFixture fixture;
    auto& action = *fixture.m_action;

    action.onCoordinateEvent(action.getStatus(), false, RS_Vector{0.0, 0.0});
    action.onCoordinateEvent(action.getStatus(), false, RS_Vector{100.0, 0.0});
    REQUIRE(action.getCenterlinePoints().size() == 2);

    SECTION("Hairpin turnaround (177 deg >= 170 deg) is rejected from centerline") {
        action.onCoordinateEvent(action.getStatus(), false, RS_Vector{5.0, 5.0});
        // Centerline point must NOT be added
        CHECK(action.getCenterlinePoints().size() == 2);
        CHECK(action.getCenterlinePoints().back().distanceTo(RS_Vector{100.0, 0.0}) < 1e-6);

        // Preview candidate with (5, 5) must be flagged invalid
        QList<RS_Entity*> previewList;
        RS_Vector badSnap{5.0, 5.0};
        action.doPreparePreviewEntities(nullptr, badSnap, previewList, action.getStatus());
        CHECK(!action.isLastPreviewValid());
        qDeleteAll(previewList);
    }

    SECTION("Exact 180-degree reversal is rejected from centerline") {
        action.onCoordinateEvent(action.getStatus(), false, RS_Vector{0.0, 0.0});
        CHECK(action.getCenterlinePoints().size() == 2);
        CHECK(action.getCenterlinePoints().back().distanceTo(RS_Vector{100.0, 0.0}) < 1e-6);
    }
}

TEST_CASE("Wall cancellation leaves document unchanged", "[wall][action]") {
    WallActionFixture fixture;
    auto& action = *fixture.m_action;

    // Document initially empty
    REQUIRE(fixture.m_graphic.count() == 0);

    // Place points
    action.onCoordinateEvent(action.getStatus(), false, RS_Vector{0.0, 0.0});
    action.onCoordinateEvent(action.getStatus(), false, RS_Vector{100.0, 0.0});
    REQUIRE(action.getCenterlinePoints().size() == 2);

    // Right-click / Escape cancellation
    action.doBack(nullptr, action.getStatus());

    // Action state is reset to SetStartPoint with empty centerline
    CHECK(action.getStatus() == LC_ActionDrawWall::SetStartPoint);
    CHECK(action.getCenterlinePoints().empty());

    // Document entity count must be strictly 0
    CHECK(fixture.m_graphic.count() == 0);
}

TEST_CASE("Wall confirmation creates expected open wall boundary and cap entities", "[wall][action]") {
    WallActionFixture fixture;
    auto& action = *fixture.m_action;

    action.setThickness(10.0);

    // Draw straight horizontal wall (0, 0) -> (100, 0)
    action.onCoordinateEvent(action.getStatus(), false, RS_Vector{0.0, 0.0});
    action.onCoordinateEvent(action.getStatus(), false, RS_Vector{100.0, 0.0});

    // Confirm open wall
    action.completeWall(false);

    // Exactly 4 entities created: 2 boundaries (left + right) + 2 flat end caps
    CHECK(fixture.m_graphic.count() == 4);

    // State reset for next wall
    CHECK(action.getStatus() == LC_ActionDrawWall::SetStartPoint);
    CHECK(action.getCenterlinePoints().empty());
}

TEST_CASE("Wall confirmation creates expected closed wall without end caps", "[wall][action]") {
    WallActionFixture fixture;
    auto& action = *fixture.m_action;

    action.setThickness(10.0);

    // Draw triangle wall: (0, 0) -> (100, 0) -> (50, 80)
    action.onCoordinateEvent(action.getStatus(), false, RS_Vector{0.0, 0.0});
    action.onCoordinateEvent(action.getStatus(), false, RS_Vector{100.0, 0.0});
    action.onCoordinateEvent(action.getStatus(), false, RS_Vector{50.0, 80.0});

    // Close wall
    action.completeWall(true);

    // Exactly 6 entities created: 3 left boundary + 3 right boundary, 0 end caps
    CHECK(fixture.m_graphic.count() == 6);

    CHECK(action.getStatus() == LC_ActionDrawWall::SetStartPoint);
    CHECK(action.getCenterlinePoints().empty());
}

TEST_CASE("One-step undo reverses entire completed wall", "[wall][action]") {
    WallActionFixture fixture;
    auto& action = *fixture.m_action;

    action.onCoordinateEvent(action.getStatus(), false, RS_Vector{0.0, 0.0});
    action.onCoordinateEvent(action.getStatus(), false, RS_Vector{100.0, 0.0});
    action.completeWall(false);

    bool undoAvail = false;
    bool redoAvail = false;
    fixture.m_graphic.collectUndoState(undoAvail, redoAvail);
    REQUIRE(undoAvail);
    REQUIRE(!redoAvail);
    REQUIRE(fixture.m_graphic.count() == 4);
    REQUIRE(fixture.countAlive() == 4);

    // Single undo operation must reverse all 4 entities
    REQUIRE(fixture.m_graphic.undo());
    fixture.m_graphic.collectUndoState(undoAvail, redoAvail);
    CHECK(!undoAvail);
    CHECK(redoAvail);
    CHECK(fixture.countAlive() == 0);
    CHECK(!fixture.m_graphic.undo());

    // Redo restores all 4 entities
    REQUIRE(fixture.m_graphic.redo());
    fixture.m_graphic.collectUndoState(undoAvail, redoAvail);
    CHECK(undoAvail);
    CHECK(!redoAvail);
    CHECK(fixture.countAlive() == 4);
    CHECK(!fixture.m_graphic.redo());
}

TEST_CASE("Wall thickness updates via command line and setter", "[wall][action]") {
    WallActionFixture fixture;
    auto& action = *fixture.m_action;

    // Test setter
    action.setThickness(20.0);
    CHECK(action.getThickness() == Approx(20.0));

    // Test command "thickness 15.0"
    action.doProcessCommand(action.getStatus(), "thickness 15.0");
    CHECK(action.getThickness() == Approx(15.0));

    // Test short command "t 30.0"
    action.doProcessCommand(action.getStatus(), "t 30.0");
    CHECK(action.getThickness() == Approx(30.0));

    // Test short command "w 25.0"
    action.doProcessCommand(action.getStatus(), "w 25.0");
    CHECK(action.getThickness() == Approx(25.0));

    // Test direct numeric expression "50"
    action.doProcessCommand(action.getStatus(), "50");
    CHECK(action.getThickness() == Approx(50.0));
}

TEST_CASE("Repeated activation, preview clearing, and destruction safety", "[wall][action]") {
    WallActionFixture fixture;
    auto& action = *fixture.m_action;

    for (int cycle = 0; cycle < 5; ++cycle) {
        action.init(LC_ActionDrawWall::SetStartPoint);
        action.onCoordinateEvent(action.getStatus(), false, RS_Vector{0.0, 0.0});
        action.onCoordinateEvent(action.getStatus(), false, RS_Vector{100.0, 0.0});

        QList<RS_Entity*> previewList;
        RS_Vector snap{100.0, 100.0};
        action.doPreparePreviewEntities(nullptr, snap, previewList, action.getStatus());
        qDeleteAll(previewList);

        action.doBack(nullptr, action.getStatus());
        CHECK(action.getCenterlinePoints().empty());
    }

    CHECK(fixture.m_graphic.count() == 0);
}

TEST_CASE("Wall closed completion failure preserves centerline for editing without document modification", "[wall][action]") {
    WallActionFixture fixture;
    auto& action = *fixture.m_action;

    action.setThickness(10.0);

    // Initial document state: 0 entities, no undo/redo available
    REQUIRE(fixture.m_graphic.count() == 0);
    bool undoAvail = false;
    bool redoAvail = false;
    fixture.m_graphic.collectUndoState(undoAvail, redoAvail);
    REQUIRE(!undoAvail);
    REQUIRE(!redoAvail);

    // 1. Build an in-progress wall with 4 points:
    // P0: (0, 0), P1: (100, 0), P2: (100, 50), P3: (5, 0)
    // As an open wall, these 4 points are valid.
    // However, closing this wall adds segment P3 (5, 0) -> P0 (0, 0),
    // which creates an exact 180-degree reversal with outgoing P0 -> P1 (100, 0).
    action.onCoordinateEvent(action.getStatus(), false, RS_Vector{0.0, 0.0});
    action.onCoordinateEvent(action.getStatus(), false, RS_Vector{100.0, 0.0});
    action.onCoordinateEvent(action.getStatus(), false, RS_Vector{100.0, 50.0});
    action.onCoordinateEvent(action.getStatus(), false, RS_Vector{5.0, 0.0});
    REQUIRE(action.getCenterlinePoints().size() == 4);
    REQUIRE(action.getStatus() == LC_ActionDrawWall::SetNextPoint);

    // Attempt to complete as closed wall -> must fail validation
    action.completeWall(true);

    // Invariants verified after failed closed completion:
    // 1. Action remains active in SetNextPoint status
    CHECK(action.getStatus() == LC_ActionDrawWall::SetNextPoint);
    // 2. In-progress centerline points are preserved (not discarded!)
    REQUIRE(action.getCenterlinePoints().size() == 4);
    CHECK(action.getCenterlinePoints()[0].distanceTo(RS_Vector{0.0, 0.0}) < 1e-6);
    CHECK(action.getCenterlinePoints()[1].distanceTo(RS_Vector{100.0, 0.0}) < 1e-6);
    CHECK(action.getCenterlinePoints()[2].distanceTo(RS_Vector{100.0, 50.0}) < 1e-6);
    CHECK(action.getCenterlinePoints()[3].distanceTo(RS_Vector{5.0, 0.0}) < 1e-6);
    // 3. Document entity count remains unchanged (0)
    CHECK(fixture.m_graphic.count() == 0);
    CHECK(fixture.countAlive() == 0);
    // 4. No empty undo transaction created (undo stack is clean)
    fixture.m_graphic.collectUndoState(undoAvail, redoAvail);
    CHECK(!undoAvail);
    CHECK(!redoAvail);

    // 5. Centerline points remain available for editing:
    // User presses Backspace to drop the problematic P3 (5, 0)
    QKeyEvent bs(QEvent::KeyPress, Qt::Key_Backspace, Qt::NoModifier);
    action.keyPressEvent(&bs);
    REQUIRE(action.getCenterlinePoints().size() == 3);
    CHECK(action.getCenterlinePoints().back().distanceTo(RS_Vector{100.0, 50.0}) < 1e-6);
    CHECK(action.getStatus() == LC_ActionDrawWall::SetNextPoint);

    // User provides corrected point P3' (0, 50) completing a proper rectangle
    action.onCoordinateEvent(action.getStatus(), false, RS_Vector{0.0, 50.0});
    REQUIRE(action.getCenterlinePoints().size() == 4);

    // 6. Complete wall as closed again -> now succeeds!
    action.completeWall(true);

    // Action state resets to SetStartPoint with empty centerline for next wall
    CHECK(action.getStatus() == LC_ActionDrawWall::SetStartPoint);
    CHECK(action.getCenterlinePoints().empty());

    // Document contains exactly 8 boundary entities (4 segments * 2 boundaries, 0 end caps)
    CHECK(fixture.m_graphic.count() == 8);
    CHECK(fixture.countAlive() == 8);

    // Single undo reverses the entire completed wall
    fixture.m_graphic.collectUndoState(undoAvail, redoAvail);
    CHECK(undoAvail);
    CHECK(!redoAvail);
    REQUIRE(fixture.m_graphic.undo());
    fixture.m_graphic.collectUndoState(undoAvail, redoAvail);
    CHECK(!undoAvail);
    CHECK(redoAvail);
    CHECK(fixture.countAlive() == 0);

    // Redo restores all 8 entities
    REQUIRE(fixture.m_graphic.redo());
    fixture.m_graphic.collectUndoState(undoAvail, redoAvail);
    CHECK(undoAvail);
    CHECK(!redoAvail);
    CHECK(fixture.countAlive() == 8);
}

TEST_CASE("Wall action regression: Open intermediate short-segment point is rejected by onCoordinateEvent", "[wall][action]") {
    WallActionFixture fixture;
    auto& action = *fixture.m_action;
    action.setThickness(10.0);

    // Build centerline: (0,0) -> (100,0)
    action.onCoordinateEvent(action.getStatus(), false, RS_Vector{0.0, 0.0});
    action.onCoordinateEvent(action.getStatus(), false, RS_Vector{100.0, 0.0});
    REQUIRE(action.getCenterlinePoints().size() == 2);
    REQUIRE(action.getStatus() == LC_ActionDrawWall::SetNextPoint);

    // Attempting to place (100, 3) turns 90 degrees into a segment of length 3,
    // which requires an inner shift of 5.0 > 3.0.
    // onCoordinateEvent must reject adding this point, preserving the 2 valid points.
    action.onCoordinateEvent(action.getStatus(), false, RS_Vector{100.0, 3.0});
    CHECK(action.getCenterlinePoints().size() == 2);
    CHECK(action.getStatus() == LC_ActionDrawWall::SetNextPoint);
    CHECK(fixture.m_graphic.count() == 0);

    // Placing a valid point (100, 30) with sufficient length (30 >= 5) is accepted
    action.onCoordinateEvent(action.getStatus(), false, RS_Vector{100.0, 30.0});
    CHECK(action.getCenterlinePoints().size() == 3);
    CHECK(action.getStatus() == LC_ActionDrawWall::SetNextPoint);
}

TEST_CASE("Wall action regression: Short initial segment inner turn is rejected by onCoordinateEvent", "[wall][action]") {
    WallActionFixture fixture;
    auto& action = *fixture.m_action;
    action.setThickness(10.0);

    // Build centerline: (0,0) -> (4,0)
    action.onCoordinateEvent(action.getStatus(), false, RS_Vector{0.0, 0.0});
    action.onCoordinateEvent(action.getStatus(), false, RS_Vector{4.0, 0.0});
    REQUIRE(action.getCenterlinePoints().size() == 2);
    REQUIRE(action.getStatus() == LC_ActionDrawWall::SetNextPoint);

    // Attempting to place (4, 100) requires a 5.0 inner shift along (0,0)->(4,0) which exceeds length 4:
    // onCoordinateEvent must reject adding this point, preserving centerline at 2 points.
    action.onCoordinateEvent(action.getStatus(), false, RS_Vector{4.0, 100.0});
    CHECK(action.getCenterlinePoints().size() == 2);
    CHECK(action.getStatus() == LC_ActionDrawWall::SetNextPoint);
    CHECK(fixture.m_graphic.count() == 0);

    // Backspace removes last placed point and allows clean restart
    QKeyEvent bs(QEvent::KeyPress, Qt::Key_Backspace, Qt::NoModifier);
    action.keyPressEvent(&bs);
    CHECK(action.getCenterlinePoints().size() == 1);
    CHECK(action.getStatus() == LC_ActionDrawWall::SetNextPoint);
}


