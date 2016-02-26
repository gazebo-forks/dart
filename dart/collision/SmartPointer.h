/*
 * Copyright (c) 2016, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Jeongseok Lee <jslee02@gmail.com>
 *
 * Georgia Tech Graphics Lab and Humanoid Robotics Lab
 *
 * Directed by Prof. C. Karen Liu and Prof. Mike Stilman
 * <karenliu@cc.gatech.edu> <mstilman@cc.gatech.edu>
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef DART_COLLISION_SMARTPOINTER_H_
#define DART_COLLISION_SMARTPOINTER_H_

#include "dart/config.h"
#include "dart/common/SmartPointer.h"

namespace dart {
namespace collision {

DART_COMMON_MAKE_SHARED_WEAK(CollisionDetector)

DART_COMMON_MAKE_SHARED_WEAK(Engine)
DART_COMMON_MAKE_SHARED_WEAK(FCLEngine)
DART_COMMON_MAKE_SHARED_WEAK(FCLMeshEngine)
DART_COMMON_MAKE_SHARED_WEAK(DARTEngine)
#ifdef HAVE_BULLET_COLLISION
  DART_COMMON_MAKE_SHARED_WEAK(BulletEngine)
#endif

DART_COMMON_MAKE_SHARED_WEAK(CollisionObject)
DART_COMMON_MAKE_SHARED_WEAK(CollisionObjectData)
DART_COMMON_MAKE_SHARED_WEAK(FreeCollisionObject)

DART_COMMON_MAKE_SHARED_WEAK(CollisionGroup)
DART_COMMON_MAKE_SHARED_WEAK(CollisionGroupData)


}  // namespace collision
}  // namespace dart

#endif  // DART_COLLISION_SMARTPOINTER_H_
