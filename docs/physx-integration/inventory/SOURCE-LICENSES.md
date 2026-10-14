# 源码摘录来源与许可

只对分析文档中的原文摘录使用以下来源声明；每个片段均在JSON中标明原文件和行号。相同文件多次引用已去重。

来源：`physx/physx/include/PxArticulationFlag.h`, `physx/physx/include/PxArticulationJointReducedCoordinate.h`, `physx/physx/include/PxArticulationMimicJoint.h`, `physx/physx/include/PxArticulationReducedCoordinate.h`, `physx/physx/include/PxArticulationTendon.h`, `physx/physx/include/PxArticulationTendonData.h`, `physx/physx/include/PxConeLimitedConstraint.h`, `physx/physx/include/PxConstraint.h`, `physx/physx/include/PxConstraintDesc.h`, `physx/physx/include/PxContactModifyCallback.h`, `physx/physx/include/PxDeformableAttachment.h`, `physx/physx/include/PxDeformableBody.h`, `physx/physx/include/PxDeformableBodyFlag.h`, `physx/physx/include/PxDeformableElementFilter.h`, `physx/physx/include/PxDeformableMaterial.h`, `physx/physx/include/PxDeformableSurface.h`, `physx/physx/include/PxDeformableSurfaceFlag.h`, `physx/physx/include/PxDeformableSurfaceMaterial.h`, `physx/physx/include/PxDeformableVolume.h`, `physx/physx/include/PxDeformableVolumeFlag.h`, `physx/physx/include/PxDeformableVolumeMaterial.h`, `physx/physx/include/PxFEMParameter.h`, `physx/physx/include/PxFiltering.h`, `physx/physx/include/PxMaterial.h`, `physx/physx/include/PxPBDMaterial.h`, `physx/physx/include/PxPBDParticleSystem.h`, `physx/physx/include/PxParticleBuffer.h`, `physx/physx/include/PxParticleGpu.h`, `physx/physx/include/PxParticleSystem.h`, `physx/physx/include/PxParticleSystemFlag.h`, `physx/physx/include/PxPhysics.h`, `physx/physx/include/PxRigidBody.h`, `physx/physx/include/PxRigidDynamic.h`, `physx/physx/include/PxSceneDesc.h`, `physx/physx/include/PxShape.h`, `physx/physx/include/extensions/PxConstraintExt.h`, `physx/physx/include/extensions/PxD6Joint.h`, `physx/physx/include/extensions/PxDistanceJoint.h`, `physx/physx/include/extensions/PxGearJoint.h`, `physx/physx/include/extensions/PxJoint.h`, `physx/physx/include/extensions/PxJointLimit.h`, `physx/physx/include/extensions/PxParticleClothCooker.h`, `physx/physx/include/extensions/PxPrismaticJoint.h`, `physx/physx/include/extensions/PxRackAndPinionJoint.h`, `physx/physx/include/extensions/PxRevoluteJoint.h`, `physx/physx/include/extensions/PxSphericalJoint.h`, `physx/physx/include/solver/PxSolverDefs.h`, `physx/physx/include/vehicle2/physxConstraints/PxVehiclePhysXConstraintParams.h`, `physx/physx/include/vehicle2/physxConstraints/PxVehiclePhysXConstraintStates.h`, `physx/physx/snippets/snippetcustomjoint/PulleyJoint.cpp`, `physx/physx/snippets/snippetcustomjoint/PulleyJoint.h`, `physx/physx/source/gpusimulationcontroller/src/CUDA/FEMCloth.cu`, `physx/physx/source/gpusimulationcontroller/src/CUDA/FEMClothConstraintPrep.cu`, `physx/physx/source/gpusimulationcontroller/src/CUDA/FEMClothExternalSolve.cu`, `physx/physx/source/gpusimulationcontroller/src/CUDA/anisotropy.cu`, `physx/physx/source/gpusimulationcontroller/src/CUDA/attachments.cuh`, `physx/physx/source/gpusimulationcontroller/src/CUDA/particleSystem.cuh`, `physx/physx/source/gpusimulationcontroller/src/CUDA/particlesystem.cu`, `physx/physx/source/gpusimulationcontroller/src/CUDA/softBody.cu`, `physx/physx/source/gpusimulationcontroller/src/CUDA/softBodyGM.cu`, `physx/physx/source/gpusimulationcontroller/src/PxgParticleSystemCore.cpp`, `physx/physx/source/lowlevel/software/include/PxsMaterialCombiner.h`, `physx/physx/source/lowleveldynamics/include/DyArticulationJointCore.h`, `physx/physx/source/lowleveldynamics/include/DyArticulationTendon.h`, `physx/physx/source/lowleveldynamics/src/DyArticulationMimicJoint.cpp`, `physx/physx/source/lowleveldynamics/src/DyConstraintSetup.cpp`, `physx/physx/source/lowleveldynamics/src/DyContactPrep.cpp`, `physx/physx/source/lowleveldynamics/src/DyFeatherstoneArticulation.cpp`, `physx/physx/source/lowleveldynamics/src/DySolverConstraintTypes.h`, `physx/physx/source/physx/src/NpArticulationJointReducedCoordinate.cpp`, `physx/physx/source/physx/src/NpArticulationMimicJoint.cpp`, `physx/physx/source/physx/src/NpArticulationReducedCoordinate.cpp`, `physx/physx/source/physx/src/NpArticulationTendon.cpp`, `physx/physx/source/physx/src/NpConstraint.cpp`, `physx/physx/source/physx/src/NpDeformableAttachment.cpp`, `physx/physx/source/physx/src/NpFactory.cpp`, `physx/physx/source/physx/src/NpPBDMaterial.cpp`, `physx/physx/source/physx/src/NpPhysics.cpp`, `physx/physx/source/physx/src/NpShape.cpp`, `physx/physx/source/physxextensions/src/ExtConstraintHelper.h`, `physx/physx/source/physxextensions/src/ExtD6Joint.cpp`, `physx/physx/source/physxextensions/src/ExtD6Joint.h`, `physx/physx/source/physxextensions/src/ExtDistanceJoint.cpp`, `physx/physx/source/physxextensions/src/ExtFixedJoint.cpp`, `physx/physx/source/physxextensions/src/ExtGearJoint.cpp`, `physx/physx/source/physxextensions/src/ExtJoint.h`, `physx/physx/source/physxextensions/src/ExtParticleClothCooker.cpp`, `physx/physx/source/physxextensions/src/ExtPrismaticJoint.cpp`, `physx/physx/source/physxextensions/src/ExtRackAndPinionJoint.cpp`, `physx/physx/source/physxextensions/src/ExtRevoluteJoint.cpp`, `physx/physx/source/physxextensions/src/ExtSphericalJoint.cpp`, `physx/physx/source/physxvehicle/src/physxConstraints/VhPhysXConstraintFunctions.cpp`, `physx/physx/source/physxvehicle/src/physxConstraints/VhPhysXConstraintHelpers.cpp`, `physx/physx/source/simulationcontroller/include/ScArticulationTendonCore.h`, `physx/physx/source/simulationcontroller/src/ScConstraintCore.cpp`

```text
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of NVIDIA CORPORATION nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
// OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Copyright (c) 2008-2025 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
```

来源：`physx/physx/source/gpusimulationcontroller/src/PxgSimulationController.cpp`

```text
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of NVIDIA CORPORATION nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
// OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Copyright (c) 2008-2025 NVIDIA Corporation. All rights reserved.
```

来源：`bullet/src/BulletDynamics/ConstraintSolver/btFixedConstraint.h`, `bullet/src/BulletDynamics/Featherstone/btMultiBodyConstraintSolver.cpp`, `bullet/src/BulletDynamics/Featherstone/btMultiBodyConstraintSolver.h`, `bullet/src/BulletDynamics/Featherstone/btMultiBodyLink.h`, `bullet/src/BulletDynamics/Featherstone/btMultiBodySolverConstraint.h`

```text
/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2013 Erwin Coumans  http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/
```

来源：`bullet/src/BulletDynamics/Featherstone/btMultiBodyFixedConstraint.h`, `bullet/src/BulletDynamics/Featherstone/btMultiBodyGearConstraint.h`, `bullet/src/BulletDynamics/Featherstone/btMultiBodyJointMotor.h`, `bullet/src/BulletDynamics/Featherstone/btMultiBodyPoint2Point.h`, `bullet/src/BulletDynamics/Featherstone/btMultiBodySliderConstraint.h`

```text
/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2013 Erwin Coumans  http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

///This file was written by Erwin Coumans
```

来源：`bullet/src/BulletDynamics/ConstraintSolver/btHingeConstraint.h`

```text
/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  https://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

/* Hinge Constraint by Dirk Gregorius. Limits added by Marcus Hennix at Starbreeze Studios */
```

来源：`bullet/src/BulletDynamics/Featherstone/btMultiBodyConstraint.h`, `bullet/src/BulletDynamics/Featherstone/btMultiBodyJointLimitConstraint.h`

```text
/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2013 Erwin Coumans  http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/
```

来源：`bullet/src/BulletDynamics/ConstraintSolver/btSliderConstraint.h`

```text
/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  https://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

/*
Added by Roman Ponomarev (rponom@gmail.com)
April 04, 2008

TODO:
 - add clamping od accumulated impulse to improve stability
 - add conversion for ODE constraint solver
*/
```

来源：`bullet/src/BulletCollision/CollisionDispatch/btCollisionObject.h`, `bullet/src/BulletDynamics/ConstraintSolver/btPoint2PointConstraint.h`, `bullet/src/BulletDynamics/ConstraintSolver/btSolverConstraint.h`, `bullet/src/BulletDynamics/Dynamics/btRigidBody.h`, `bullet/src/BulletSoftBody/btSoftBodySolvers.h`

```text
/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  https://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/
```

来源：`bullet/src/BulletDynamics/ConstraintSolver/btConeTwistConstraint.h`

```text
/*
Bullet Continuous Collision Detection and Physics Library
btConeTwistConstraint is Copyright (c) 2007 Starbreeze Studios

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.

Written by: Marcus Hennix
*/

/*
Overview:

btConeTwistConstraint can be used to simulate ragdoll joints (upper arm, leg etc).
It is a fixed translation, 3 degree-of-freedom (DOF) rotational "joint".
It divides the 3 rotational DOFs into swing (movement within a cone) and twist.
Swing is divided into swing1 and swing2 which can have different limits, giving an elliptical shape.
(Note: the cone's base isn't flat, so this ellipse is "embedded" on the surface of a sphere.)

In the contraint's frame of reference:
twist is along the x-axis,
and swing 1 and 2 are along the z and y axes respectively.
*/
```

来源：`bullet/src/BulletDynamics/ConstraintSolver/btTypedConstraint.h`

```text
/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2010 Erwin Coumans  https://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/
```

来源：`bullet/src/BulletDynamics/ConstraintSolver/btGeneric6DofSpring2Constraint.h`

```text
/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  https://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

/*
2014 May: btGeneric6DofSpring2Constraint is created from the original (2.82.2712) btGeneric6DofConstraint by Gabor Puhr and Tamas Umenhoffer
Pros:
- Much more accurate and stable in a lot of situation. (Especially when a sleeping chain of RBs connected with 6dof2 is pulled)
- Stable and accurate spring with minimal energy loss that works with all of the solvers. (latter is not true for the original 6dof spring)
- Servo motor functionality
- Much more accurate bouncing. 0 really means zero bouncing (not true for the original 6odf) and there is only a minimal energy loss when the value is 1 (because of the solvers' precision)
- Rotation order for the Euler system can be set. (One axis' freedom is still limited to pi/2)

Cons:
- It is slower than the original 6dof. There is no exact ratio, but half speed is a good estimation.
- At bouncing the correct velocity is calculated, but not the correct position. (it is because of the solver can correct position or velocity, but not both.)
*/

/// 2009 March: btGeneric6DofConstraint refactored by Roman Ponomarev
/// Added support for generic constraint solver through getInfo1/getInfo2 methods

/*
2007-09-09
btGeneric6DofConstraint Refactored by Francisco Le?n
email: projectileman@yahoo.com
http://gimpact.sf.net
*/
```

来源：`bullet/src/BulletDynamics/ConstraintSolver/btGearConstraint.h`

```text
/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2012 Advanced Micro Devices, Inc.  http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/
```

来源：`bullet/src/BulletDynamics/Featherstone/btMultiBody.h`

```text
/*
 * PURPOSE:
 *   Class representing an articulated rigid body. Stores the body's
 *   current state, allows forces and torques to be set, handles
 *   timestepping and implements Featherstone's algorithm.
 *
 * COPYRIGHT:
 *   Copyright (C) Stephen Thompson, <stephen@solarflare.org.uk>, 2011-2013
 *   Portions written By Erwin Coumans: connection to LCP solver, various multibody constraints, replacing Eigen math library by Bullet LinearMath and a dedicated 6x6 matrix inverse (solveImatrix)
 *   Portions written By Jakub Stepien: support for multi-DOF constraints, introduction of spatial algebra and several other improvements

 This software is provided 'as-is', without any express or implied warranty.
 In no event will the authors be held liable for any damages arising from the use of this software.
 Permission is granted to anyone to use this software for any purpose,
 including commercial applications, and to alter it and redistribute it freely,
 subject to the following restrictions:

 1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
 2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
 3. This notice may not be removed or altered from any source distribution.

 */
```

来源：`bullet/src/BulletDynamics/Featherstone/btMultiBodySphericalJointLimit.h`, `bullet/src/BulletDynamics/Featherstone/btMultiBodySphericalJointMotor.h`

```text
/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2018 Erwin Coumans  http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

///This file was written by Erwin Coumans
```

来源：`bullet/src/BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.h`

```text
/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  https://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/
```

来源：`bullet/src/BulletDynamics/Vehicle/btRaycastVehicle.h`

```text
/*
 * Copyright (c) 2005 Erwin Coumans https://bulletphysics.org
 *
 * Permission to use, copy, modify, distribute and sell this software
 * and its documentation for any purpose is hereby granted without fee,
 * provided that the above copyright notice appear in all copies.
 * Erwin Coumans makes no representations about the suitability
 * of this software for any purpose.
 * It is provided "as is" without express or implied warranty.
*/
```

来源：`bullet/src/BulletSoftBody/btSoftBody.cpp`, `bullet/src/BulletSoftBody/btSoftBody.h`

```text
/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  https://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/
///btSoftBody implementation by Nathanael Presson
```

来源：`bullet/src/BulletSoftBody/btDeformableContactConstraint.h`, `bullet/src/BulletSoftBody/btDeformableCorotatedForce.h`, `bullet/src/BulletSoftBody/btDeformableLagrangianForce.h`, `bullet/src/BulletSoftBody/btDeformableMultiBodyDynamicsWorld.h`

```text
/*
 Written by Xuchen Han <xuchenhan2015@u.northwestern.edu>

 Bullet Continuous Collision Detection and Physics Library
 Copyright (c) 2019 Google Inc. http://bulletphysics.org
 This software is provided 'as-is', without any express or implied warranty.
 In no event will the authors be held liable for any damages arising from the use of this software.
 Permission is granted to anyone to use this software for any purpose,
 including commercial applications, and to alter it and redistribute it freely,
 subject to the following restrictions:
 1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
 2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
 3. This notice may not be removed or altered from any source distribution.
 */
```

来源：`bullet/src/BulletSoftBody/btDeformableNeoHookeanForce.h`

```text
/*
Written by Xuchen Han <xuchenhan2015@u.northwestern.edu>

Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2019 Google Inc. http://bulletphysics.org
This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:
1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/
```

来源：`bullet/src/BulletDynamics/MLCPSolvers/btMLCPSolver.h`

```text
/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2013 Erwin Coumans  http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/
///original version written by Erwin Coumans, October 2013
```

来源：`bullet/src/BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.cpp`

```text
/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  https://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

//#define COMPUTE_IMPULSE_DENOM 1
```

来源：`bullet/src/BulletSoftBody/btDeformableMultiBodyDynamicsWorld.cpp`

```text
/*
 Written by Xuchen Han <xuchenhan2015@u.northwestern.edu>

 Bullet Continuous Collision Detection and Physics Library
 Copyright (c) 2019 Google Inc. http://bulletphysics.org
 This software is provided 'as-is', without any express or implied warranty.
 In no event will the authors be held liable for any damages arising from the use of this software.
 Permission is granted to anyone to use this software for any purpose,
 including commercial applications, and to alter it and redistribute it freely,
 subject to the following restrictions:
 1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
 2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
 3. This notice may not be removed or altered from any source distribution.
 */

/* ====== Overview of the Deformable Algorithm ====== */

/*
A single step of the deformable body simulation contains the following main components:
Call internalStepSimulation multiple times, to achieve 240Hz (4 steps of 60Hz).
1. Deformable maintaintenance of rest lengths and volume preservation. Forces only depend on position: Update velocity to a temporary state v_{n+1}^* = v_n + explicit_force * dt / mass, where explicit forces include gravity and elastic forces.
2. Detect discrete collisions between rigid and deformable bodies at position x_{n+1}^* = x_n + dt * v_{n+1}^*.

3a. Solve all constraints, including LCP. Contact, position correction due to numerical drift, friction, and anchors for deformable.

3b. 5 Newton steps (multiple step). Conjugent Gradient solves linear system. Deformable Damping: Then velocities of deformable bodies v_{n+1} are solved in
        M(v_{n+1} - v_{n+1}^*) = damping_force * dt / mass,
   by a conjugate gradient solver, where the damping force is implicit and depends on v_{n+1}.
   Make sure contact constraints are not violated in step b by performing velocity projections as in the paper by Baraff and Witkin https://www.cs.cmu.edu/~baraff/papers/sig98.pdf. Dynamic frictions are treated as a force and added to the rhs of the CG solve, whereas static frictions are treated as constraints similar to contact.
4. Position is updated via x_{n+1} = x_n + dt * v_{n+1}.


The algorithm also closely resembles the one in http://physbam.stanford.edu/~fedkiw/papers/stanford2008-03.pdf
 */
```
