/* New Bullet extension, 2026. Distributed under the Bullet zlib license. */
#ifndef BT_MULTIBODY_DISTANCE_CONSTRAINT_H
#define BT_MULTIBODY_DISTANCE_CONSTRAINT_H
#include "btMultiBodyConstraint.h"
#include "BulletDynamics/ConstraintSolver/btDistanceConstraint.h"
class btMultiBodyDistanceConstraint : public btMultiBodyConstraint, public btDistanceConstraintState
{
public:
	BT_DECLARE_ALIGNED_ALLOCATOR();
	// Either endpoint may be a link, rigid body, or world. At least one must be a link.
	btMultiBodyDistanceConstraint(const btDistanceConstraintEndpoint& a, const btDistanceConstraintEndpoint& b,
		const btVector3& pivotA, const btVector3& pivotB);
	virtual void finalizeMultiDof();
	virtual int getIslandIdA() const;
	virtual int getIslandIdB() const;
	virtual void createConstraintRows(btMultiBodyConstraintArray& rows, btMultiBodyJacobianData& data, const btContactSolverInfo& info);
	virtual void onExtensionRowSolved(int index, btScalar impulse, btScalar dt);
	virtual void debugDraw(btIDebugDraw* drawer) { debugDrawDistance(drawer); }
};
#endif
