/******************************************************************************
 *
 *    This file is part of openDarkEngine project
 *    Copyright (C) 2005-2009 openDarkEngine team
 *
 *    This program is free software; you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation; either version 2 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with this program; if not, write to the Free Software
 *    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 *
 *		$Id$
 *
 *****************************************************************************/

#ifndef __SIMCOMMON_H
#define __SIMCOMMON_H

#include <cstring>
#include "DarknessMath.h"
#include "worldquery/ObjectState.h"

namespace Darkness {

// ============================================================================
// SimTransform — base transform snapshot for simulated objects
// ============================================================================
//
// Shared by DoorSystem, TweqSystem, and any future system that animates
// objects at runtime. Stores the object's initial position, rotation, and
// scale as captured from P$Position / ObjectPlacement at init time.
// Systems compose animation offsets on top of this base transform.

struct SimTransform {
    Vector3 position = {0.0f, 0.0f, 0.0f};
    Matrix4 rotation = Matrix4(1.0f);  // column-major GLM (avoids Euler round-trip)
    Vector3 scale    = {1.0f, 1.0f, 1.0f};
};

// ============================================================================
// applyModelMatrix — write a composed 4x4 transform to ObjectState
// ============================================================================
//
// Centralizes the GLM→ObjectState matrix copy used by all sim systems.
inline void applyModelMatrix(ObjectStateMap &states, int32_t objID,
                             const Matrix4 &fullGlm, const Vector3 &pos,
                             const Vector3 &scale) {
    ObjectState &os = states.get(objID);
    std::memcpy(os.modelMatrix, glm::value_ptr(fullGlm), 16 * sizeof(float));
    os.hasMatrix = true;
    os.position = pos;
    os.scale = scale;
}

// ============================================================================
// SimListener — abstract per-frame simulation callback
// ============================================================================

/** Abstract Simulation listener - a class that does something related to
 * simulation extends this
 */
class SimListener {
public:
    SimListener();

    virtual ~SimListener();

    /** Called when the simulation is started by sim service. Sets sim time to
     * zero. sets mSimRunning to true. */
    virtual void simStarted();

    /** Called when the simulation is ended by sim service. Sets mSimRunning to
     * false. */
    virtual void simEnded();

    /** Called when the simulation is paused. Sets mPaused */
    virtual void simPaused();

    /** Called when the simulation is un-paused. Unsets mPaused */
    virtual void simUnPaused();

    /** Called every time time flow change happens. */
    virtual void simFlowChange(float newFlow);

    /** simulation time step happened
     * @param simTime the new sim time
     * @param delta the time increment that happened
     */
    virtual void simStep(float simTime, float delta);

protected:
    float mSimTime;
    float mSimTimeFlow;
    bool mSimPaused;
    bool mSimRunning;
};

} // namespace Darkness

#endif /* __SIMCOMMON_H */
