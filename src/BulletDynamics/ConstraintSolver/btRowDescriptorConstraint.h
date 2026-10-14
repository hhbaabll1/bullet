/* New Bullet constraint extension interfaces. Distributed under the Bullet zlib license. */
#ifndef BT_ROW_DESCRIPTOR_CONSTRAINT_H
#define BT_ROW_DESCRIPTOR_CONSTRAINT_H
#include "btConstraintExtension.h"
#include "btTypedConstraint.h"

// Opt-in bridge for native symmetric hard scalar rows. Not a new physical joint.
// The appended type ID dispatches to no existing concrete-joint debug cast.
class btRowDescriptorConstraint : public btTypedConstraint
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
	btRowDescriptorConstraint(btRigidBody& a, uint64_t idA, btRigidBody& b, uint64_t idB);
	btRowDescriptorConstraint(btRigidBody& a, uint64_t idA);
	// Call before each simulation step. The rows are copied and remain immutable to getInfo*.
	btConstraintStatus prepare(const btConstraintFrameSnapshot& frame, const btAlignedObjectArray<btConstraintRowDescriptor>& rows);
	const btConstraintStatus& getStatus() const { return m_status; }
	void invalidate();
	virtual void getInfo1(btConstraintInfo1* info);
	virtual void getInfo2(btConstraintInfo2* info);
	// Use descriptor parameters. Legacy integer parameter setters are explicitly unsupported.
	virtual void setParam(int num, btScalar value, int axis = -1);
	virtual btScalar getParam(int num, int axis = -1) const;
	// Legacy .bullet saves base metadata with the new type ID only. Its importer
	// cannot recreate extension rows; use the explicit description graph codec.
	virtual int calculateSerializeBufferSize() const { return btTypedConstraint::calculateSerializeBufferSize(); }
	virtual const char* serialize(void* data, btSerializer* serializer) const { return btTypedConstraint::serialize(data, serializer); }
};
#endif
