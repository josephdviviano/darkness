/******************************************************************************
 *
 *    This file is part of the darkness project
 *    Copyright (C) 2024-2026 darkness contributors
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
 *****************************************************************************/

// FunctionalLoopClient.h — Lambda-based LoopClient adapter
//
// Wraps a std::function<void(float)> as a LoopClient so that existing free
// functions and lambdas (which capture local state by reference) can plug
// into the LoopService priority dispatch without needing dedicated classes.
//
// Usage:
//   FunctionalLoopClient input(LOOPCLIENT_ID_INPUT, "Input",
//       LOOPMODE_INPUT | LOOPMODE_RENDER, LOOPCLIENT_PRIORITY_INPUT,
//       [&](float dt) { handleEvents(state, ...); });
//   loopSvc->addLoopClient(&input);

#pragma once

#include <functional>
#include <string>
#include "loop/LoopCommon.h"
#include "ServiceCommon.h"

namespace Darkness {

/// A LoopClient that delegates loopStep() to a std::function.
/// The function receives delta time in seconds (float).
class FunctionalLoopClient : public LoopClient {
public:
    FunctionalLoopClient(LoopClientID id, const std::string &name,
                         LoopModeMask mask, LoopClientPriority priority,
                         std::function<void(float)> fn)
        : mFn(std::move(fn))
    {
        mLoopClientDef.id = id;
        mLoopClientDef.name = name;
        mLoopClientDef.mask = mask;
        mLoopClientDef.priority = priority;
    }

    // Allow moving but not copying (captures may hold references)
    FunctionalLoopClient(FunctionalLoopClient &&) = default;
    FunctionalLoopClient &operator=(FunctionalLoopClient &&) = default;
    FunctionalLoopClient(const FunctionalLoopClient &) = delete;
    FunctionalLoopClient &operator=(const FunctionalLoopClient &) = delete;

protected:
    void loopStep(float deltaTime) override {
        if (mFn) mFn(deltaTime);
    }

private:
    std::function<void(float)> mFn;
};

} // namespace Darkness
