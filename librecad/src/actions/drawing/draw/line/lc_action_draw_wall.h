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

#ifndef LC_ACTION_DRAW_WALL_H
#define LC_ACTION_DRAW_WALL_H

#include <vector>
#include "lc_abstractactionwithpreview.h"
#include "lc_wallmath.h"
#include "rs_vector.h"

/**
 * @brief Interactive drawing action for architectural double-line walls.
 *
 * Integrates LC_WallMath helper with LibreCAD's action, preview, command,
 * and undo/redo infrastructure.
 *
 * Centerline points are placed interactively while live wall boundaries
 * and end caps are rendered in preview. The completed wall is committed in
 * a single undoable document modification transaction.
 */
class LC_ActionDrawWall : public LC_AbstractActionWithPreview {
    Q_OBJECT
public:
    enum Status {
        SetStartPoint = InitialActionStatus,
        SetNextPoint
    };

    explicit LC_ActionDrawWall(LC_ActionContext* actionContext);
    ~LC_ActionDrawWall() override = default;

    void init(int status) override;
    void updateActionPrompt() override;
    QStringList getAvailableCommands() override;
    bool doProcessCommand(int status, const QString& command) override;
    void onCoordinateEvent(int status, bool isZero, const RS_Vector& coord) override;
    void keyPressEvent(QKeyEvent* e) override;

    // Wall parameters
    void setThickness(double thickness);
    double getThickness() const;
    void setMiterLimit(double miterLimit);
    double getMiterLimit() const;
    void setCapType(LC_WallMath::CapType capType);
    LC_WallMath::CapType getCapType() const;

    // Centerline access & interaction controls
    const std::vector<RS_Vector>& getCenterlinePoints() const;
    void completeWall(bool close);
    void undoPoint();
    bool mayClose() const;
    bool mayUndo() const;
    bool isLastPreviewValid() const;

protected:
    void doOnLeftMouseButtonRelease(const LC_MouseEvent* e, int status, const RS_Vector& snapPoint) override;
    void doBack(const LC_MouseEvent* e, int status) override;
    bool doCheckMayDrawPreview(const LC_MouseEvent* e, int status) override;
    void doPreparePreviewEntities(const LC_MouseEvent* e, RS_Vector& snap, QList<RS_Entity*>& list, int status) override;
    bool doTriggerEntitiesPrepare(LC_DocumentModificationBatch& ctx) override;
    void doTriggerCompletion(bool success) override;
    void doAfterTrigger() override;
    void doSaveOptions() override;
    void doLoadOptions() override;

private:
    std::vector<RS_Vector> m_centerlinePoints;
    LC_WallMath::WallOptions m_options;
    bool m_closing = false;
    bool m_lastPreviewValid = true;
};

#endif // LC_ACTION_DRAW_WALL_H
