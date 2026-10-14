/* New Bullet constraint extension interfaces. Distributed under the Bullet zlib license. */
#ifndef BT_MULTIBODY_ROW_DESCRIPTOR_CONSTRAINT_H
#define BT_MULTIBODY_ROW_DESCRIPTOR_CONSTRAINT_H
#include "btMultiBodyConstraint.h"
#include "BulletDynamics/ConstraintSolver/btConstraintExtension.h"

// A hard generalized-row bridge for MB/world and different MB trees only.
// Same-tree contributions require the response work in step 4 and are rejected.
class btMultiBodyRowDescriptorConstraint : public btMultiBodyConstraint
{
	btConstraintObjectRef m_objectA, m_objectB;
	btConstraintValueArray<btConstraintRowDescriptor> m_rows;
	uint64_t m_step;
	btScalar m_timeStep;
	bool m_prepared;
	bool m_hasSnapshot;
	btConstraintStatus m_status;
public:
	BT_DECLARE_ALIGNED_ALLOCATOR();
	btMultiBodyRowDescriptorConstraint(btMultiBody* a, int linkA, uint64_t idA, btMultiBody* b = 0, int linkB = -1, uint64_t idB = 0);
	btConstraintStatus prepare(const btConstraintFrameSnapshot& frame, const btAlignedObjectArray<btConstraintRowDescriptor>& rows);
	const btConstraintStatus& getStatus() const { return m_status; }
	void invalidate();
	virtual void finalizeMultiDof();
	virtual int getIslandIdA() const;
	virtual int getIslandIdB() const;
	virtual void createConstraintRows(btMultiBodyConstraintArray& rows, btMultiBodyJacobianData& data, const btContactSolverInfo& info);
	virtual void debugDraw(btIDebugDraw*) {}
};
#endif
