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

#include "lc_action_draw_wall.h"

#include <cmath>
#include <QKeyEvent>
#include "rs_commands.h"
#include "rs_graphic.h"
#include "rs_graphicview.h"
#include "rs_line.h"
#include "rs_math.h"
#include "rs_preview.h"
#include "rs_settings.h"

LC_ActionDrawWall::LC_ActionDrawWall(LC_ActionContext* actionContext)
    : LC_AbstractActionWithPreview("ActionDrawWall", actionContext, RS2::ActionDrawWall) {
    m_options.thickness = 10.0;
    m_options.miterLimit = 2.0;
    m_options.capType = LC_WallMath::CapType::Flat;
    m_options.isClosed = false;
}

void LC_ActionDrawWall::init(const int status) {
    LC_AbstractActionWithPreview::init(status);
    m_centerlinePoints.clear();
    m_closing = false;
    m_lastPreviewValid = true;
    updateActionPrompt();
}

void LC_ActionDrawWall::updateActionPrompt() {
    switch (getStatus()) {
        case SetStartPoint:
            updatePromptTRBack(tr("Specify first wall point or enter thickness"));
            break;
        case SetNextPoint:
            if (m_centerlinePoints.size() >= 2) {
                updatePromptTRBack(tr("Specify next wall point or [Close/Done/Undo]"));
            } else {
                updatePromptTRBack(tr("Specify next wall point or [Undo]"));
            }
            break;
        default:
            break;
    }
}

QStringList LC_ActionDrawWall::getAvailableCommands() {
    return {"close", "c", "done", "d", "undo", "u", "thickness", "t", "width", "w"};
}

bool LC_ActionDrawWall::doProcessCommand(const int status, const QString& command) {
    const QString cmd = command.trimmed();
    if (cmd.isEmpty()) {
        if (status == SetNextPoint && m_centerlinePoints.size() >= 2) {
            completeWall(false);
            return true;
        }
        return false;
    }

    // 1. Close command
    if (checkCommand("close", cmd) || cmd.compare("c", Qt::CaseInsensitive) == 0) {
        completeWall(true);
        return true;
    }

    // 2. Done / confirm command
    if (checkCommand("done", cmd) || cmd.compare("d", Qt::CaseInsensitive) == 0 ||
        checkCommand("enter", cmd)) {
        completeWall(false);
        return true;
    }

    // 3. Undo / back command
    if (checkCommand("undo", cmd) || cmd.compare("u", Qt::CaseInsensitive) == 0 ||
        checkCommand("back", cmd)) {
        undoPoint();
        return true;
    }

    // 4. Thickness input via prefix keyword: "thickness <val>", "width <val>", "t <val>", "w <val>"
    QString valStr;
    bool isThicknessCmd = false;
    if (cmd.startsWith("thickness ", Qt::CaseInsensitive)) {
        valStr = cmd.mid(10).trimmed();
        isThicknessCmd = true;
    } else if (cmd.startsWith("width ", Qt::CaseInsensitive)) {
        valStr = cmd.mid(6).trimmed();
        isThicknessCmd = true;
    } else if (cmd.startsWith("t ", Qt::CaseInsensitive)) {
        valStr = cmd.mid(2).trimmed();
        isThicknessCmd = true;
    } else if (cmd.startsWith("w ", Qt::CaseInsensitive)) {
        valStr = cmd.mid(2).trimmed();
        isThicknessCmd = true;
    }

    if (isThicknessCmd) {
        bool ok = false;
        const double val = RS_Math::eval(valStr, &ok);
        if (ok && val > RS_TOLERANCE) {
            setThickness(val);
            commandMessage(tr("Wall thickness set to %1").arg(val));
        } else {
            commandMessage(tr("Invalid thickness expression"));
        }
        return true;
    }

    // 5. Direct numeric expression for thickness (e.g. "200", "15.5", "100/2")
    bool ok = false;
    const double val = RS_Math::eval(cmd, &ok);
    if (ok && val > RS_TOLERANCE) {
        setThickness(val);
        commandMessage(tr("Wall thickness set to %1").arg(val));
        return true;
    }

    return false;
}

void LC_ActionDrawWall::onCoordinateEvent(const int status, [[maybe_unused]] const bool isZero,
                                        const RS_Vector& coord) {
    if (!coord.isValid() || !std::isfinite(coord.x) || !std::isfinite(coord.y)) {
        return;
    }

    switch (status) {
        case SetStartPoint: {
            m_centerlinePoints.clear();
            m_centerlinePoints.push_back(coord);
            moveRelativeZero(coord);
            addSnappedPointToVisualSnap(coord);
            setStatus(SetNextPoint);
            updateActionPrompt();
            break;
        }
        case SetNextPoint: {
            if (m_centerlinePoints.empty()) {
                setStatus(SetStartPoint);
                updateActionPrompt();
                return;
            }

            // Reject zero-length duplicate clicks
            if (m_centerlinePoints.back().distanceTo(coord) <= RS_TOLERANCE) {
                return;
            }

            // Validate candidate geometry before accepting point
            std::vector<RS_Vector> candidate = m_centerlinePoints;
            candidate.push_back(coord);
            const auto candidateWall = LC_WallMath::generateWall(candidate, m_options);
            if (!candidateWall.isValid()) {
                commandMessage(tr("Cannot add point: reversal or hairpin turn angle >= 170 degrees"));
                return;
            }

            m_centerlinePoints.push_back(coord);
            moveRelativeZero(coord);
            addSnappedPointToVisualSnap(coord);
            updateActionPrompt();
            break;
        }
        default:
            break;
    }
}

void LC_ActionDrawWall::keyPressEvent(QKeyEvent* e) {
    if (e == nullptr) {
        return;
    }
    switch (e->key()) {
        case Qt::Key_Backspace: {
            undoPoint();
            e->accept();
            break;
        }
        case Qt::Key_Return:
        case Qt::Key_Enter: {
            if (m_centerlinePoints.size() >= 2) {
                completeWall(false);
            }
            e->accept();
            break;
        }
        case Qt::Key_Escape: {
            doBack(nullptr, getStatus());
            e->accept();
            break;
        }
        default:
            LC_AbstractActionWithPreview::keyPressEvent(e);
            break;
    }
}

void LC_ActionDrawWall::setThickness(const double thickness) {
    if (std::isfinite(thickness) && thickness > RS_TOLERANCE) {
        m_options.thickness = thickness;
        drawPreviewForLastPoint();
        redraw();
    }
}

double LC_ActionDrawWall::getThickness() const {
    return m_options.thickness;
}

void LC_ActionDrawWall::setMiterLimit(const double miterLimit) {
    if (std::isfinite(miterLimit) && miterLimit >= 1.0) {
        m_options.miterLimit = miterLimit;
        drawPreviewForLastPoint();
        redraw();
    }
}

double LC_ActionDrawWall::getMiterLimit() const {
    return m_options.miterLimit;
}

void LC_ActionDrawWall::setCapType(const LC_WallMath::CapType capType) {
    m_options.capType = capType;
    drawPreviewForLastPoint();
    redraw();
}

LC_WallMath::CapType LC_ActionDrawWall::getCapType() const {
    return m_options.capType;
}

const std::vector<RS_Vector>& LC_ActionDrawWall::getCenterlinePoints() const {
    return m_centerlinePoints;
}

void LC_ActionDrawWall::completeWall(const bool close) {
    const size_t minPoints = close ? 3 : 2;
    if (m_centerlinePoints.size() < minPoints) {
        commandMessage(tr("At least %1 points required to complete wall").arg(minPoints));
        return;
    }

    auto opt = m_options;
    opt.isClosed = close;
    const auto wall = LC_WallMath::generateWall(m_centerlinePoints, opt);
    if (!wall.isValid()) {
        if (close) {
            commandMessage(tr("Cannot close wall: closing segment forms an invalid reversal or hairpin turn"));
        } else {
            commandMessage(tr("Cannot complete wall: invalid wall geometry"));
        }
        return;
    }

    m_closing = close;
    trigger();
}

void LC_ActionDrawWall::undoPoint() {
    if (m_centerlinePoints.size() > 1) {
        m_centerlinePoints.pop_back();
        moveRelativeZero(m_centerlinePoints.back());
        deletePreview();
        drawPreviewForLastPoint();
        redraw();
        updateActionPrompt();
    } else if (m_centerlinePoints.size() == 1) {
        m_centerlinePoints.clear();
        deletePreview();
        setStatus(SetStartPoint);
        redraw();
        updateActionPrompt();
    }
}

bool LC_ActionDrawWall::mayClose() const {
    return m_centerlinePoints.size() >= 3;
}

bool LC_ActionDrawWall::mayUndo() const {
    return !m_centerlinePoints.empty();
}

bool LC_ActionDrawWall::isLastPreviewValid() const {
    return m_lastPreviewValid;
}

void LC_ActionDrawWall::doOnLeftMouseButtonRelease([[maybe_unused]] const LC_MouseEvent* e,
                                                  const int status, const RS_Vector& snapPoint) {
    onCoordinateEvent(status, false, snapPoint);
}

void LC_ActionDrawWall::doBack([[maybe_unused]] const LC_MouseEvent* e, const int status) {
    deletePreview();
    if (status == SetNextPoint) {
        m_centerlinePoints.clear();
        setStatus(SetStartPoint);
        updateActionPrompt();
        redraw();
    } else {
        finishAction();
    }
}

bool LC_ActionDrawWall::doCheckMayDrawPreview([[maybe_unused]] const LC_MouseEvent* e, const int status) {
    return status == SetNextPoint && !m_centerlinePoints.empty();
}

void LC_ActionDrawWall::doPreparePreviewEntities([[maybe_unused]] const LC_MouseEvent* e, RS_Vector& snap,
                                                QList<RS_Entity*>& list, const int status) {
    if (status != SetNextPoint || m_centerlinePoints.empty()) {
        m_lastPreviewValid = false;
        return;
    }

    if (!snap.isValid() || !std::isfinite(snap.x) || !std::isfinite(snap.y)) {
        m_lastPreviewValid = false;
        return;
    }

    if (snap.distanceTo(m_centerlinePoints.back()) <= RS_TOLERANCE) {
        m_lastPreviewValid = false;
        return;
    }

    std::vector<RS_Vector> candidate = m_centerlinePoints;
    candidate.push_back(snap);

    const auto wall = LC_WallMath::generateWall(candidate, m_options);
    if (!wall.isValid()) {
        m_lastPreviewValid = false;
        createRefLine(m_centerlinePoints.back(), snap, list);
        return;
    }

    m_lastPreviewValid = true;

    // Fresh heap allocations adopted and deleted by RS_Preview
    for (const auto& lineData : wall.leftBoundary) {
        list.append(new RS_Line(m_preview.get(), lineData));
    }
    for (const auto& lineData : wall.rightBoundary) {
        list.append(new RS_Line(m_preview.get(), lineData));
    }
    for (const auto& lineData : wall.caps) {
        list.append(new RS_Line(m_preview.get(), lineData));
    }
}

bool LC_ActionDrawWall::doTriggerEntitiesPrepare(LC_DocumentModificationBatch& ctx) {
    auto opt = m_options;
    opt.isClosed = m_closing;

    const auto wall = LC_WallMath::generateWall(m_centerlinePoints, opt);
    if (!wall.isValid()) {
        commandMessage(tr("Cannot create wall: invalid wall geometry"));
        return false;
    }

    // Allocate document-owned entities for transaction batch
    for (const auto& lineData : wall.leftBoundary) {
        ctx += new RS_Line(m_document, lineData);
    }
    for (const auto& lineData : wall.rightBoundary) {
        ctx += new RS_Line(m_document, lineData);
    }
    for (const auto& lineData : wall.caps) {
        ctx += new RS_Line(m_document, lineData);
    }

    return true;
}

void LC_ActionDrawWall::doTriggerCompletion(const bool success) {
    if (success) {
        LC_AbstractActionWithPreview::doTriggerCompletion(success);
    } else {
        m_closing = false;
        clearAlternativeActionMode();
        redraw();
    }
}

void LC_ActionDrawWall::doAfterTrigger() {
    m_centerlinePoints.clear();
    m_closing = false;
    deletePreview();
    setStatus(SetStartPoint);
    updateActionPrompt();
    redraw();
}

void LC_ActionDrawWall::doSaveOptions() {
    save("Thickness", m_options.thickness);
    save("MiterLimit", m_options.miterLimit);
}

void LC_ActionDrawWall::doLoadOptions() {
    m_options.thickness = loadDouble("Thickness", 10.0);
    m_options.miterLimit = loadDouble("MiterLimit", 2.0);
}
