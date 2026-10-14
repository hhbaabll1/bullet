/* New Bullet constraint extension interfaces. Distributed under the Bullet zlib license. */
#ifndef BT_CONSTRAINT_ELEMENT_OPERATOR_H
#define BT_CONSTRAINT_ELEMENT_OPERATOR_H

#include "BulletDynamics/ConstraintSolver/btConstraintExtension.h"

enum btConstraintElementPointKind
{
	BT_CONSTRAINT_ELEMENT_NODE,
	BT_CONSTRAINT_ELEMENT_TRIANGLE,
	BT_CONSTRAINT_ELEMENT_TETRAHEDRON,
	BT_CONSTRAINT_ELEMENT_RIGID_POINT,
	BT_CONSTRAINT_ELEMENT_WORLD_POINT
};

// Element indices refer to the backend's simulation mesh, never implicitly to
// its collision mesh. The backend validates counts, topology and generations.
struct btConstraintElementPoint
{
	BT_DECLARE_ALIGNED_ALLOCATOR();
	btConstraintObjectRef m_object;
	btConstraintElementPointKind m_kind;
	int m_element;
	unsigned int m_topologyGeneration;
	btScalar m_weights[4];
	btVector3 m_localPoint;
	btConstraintElementPoint() : m_kind(BT_CONSTRAINT_ELEMENT_WORLD_POINT), m_element(-1),
		m_topologyGeneration(0), m_localPoint(0, 0, 0)
	{ for (int i = 0; i < 4; ++i) m_weights[i] = 0; }

	btConstraintStatus validate() const
	{
		if (m_object.m_domain < BT_CONSTRAINT_WORLD || m_object.m_domain > BT_CONSTRAINT_VOLUME ||
			((m_object.m_id == 0) != (m_object.m_domain == BT_CONSTRAINT_WORLD)) ||
			(m_object.m_domain == BT_CONSTRAINT_MULTIBODY_LINK ? m_object.m_link < -1 : m_object.m_link != -1))
			return btConstraintStatus(BT_CONSTRAINT_INVALID_PARAMETER, "invalid element object identity");
		for (int i = 0; i < 3; ++i) if (!btConstraintScalarFinite(m_localPoint[i]))
			return btConstraintStatus(BT_CONSTRAINT_INVALID_PARAMETER, "invalid local point");
		int count = 0;
		switch (m_kind)
		{
		case BT_CONSTRAINT_ELEMENT_WORLD_POINT:
			if (m_object.m_domain != BT_CONSTRAINT_WORLD || m_element != -1) return btConstraintStatus(BT_CONSTRAINT_INVALID_PARAMETER, "world point identity mismatch");
			break;
		case BT_CONSTRAINT_ELEMENT_RIGID_POINT:
			if ((m_object.m_domain != BT_CONSTRAINT_RIGID_BODY && m_object.m_domain != BT_CONSTRAINT_MULTIBODY_LINK) || m_element != -1) return btConstraintStatus(BT_CONSTRAINT_INVALID_PARAMETER, "rigid point identity mismatch");
			break;
		case BT_CONSTRAINT_ELEMENT_NODE:
			if (m_object.m_domain < BT_CONSTRAINT_PARTICLE || m_element < 0) return btConstraintStatus(BT_CONSTRAINT_INVALID_PARAMETER, "invalid node");
			break;
		case BT_CONSTRAINT_ELEMENT_TRIANGLE:
			if (m_object.m_domain != BT_CONSTRAINT_SURFACE || m_element < 0) return btConstraintStatus(BT_CONSTRAINT_INVALID_PARAMETER, "invalid surface triangle");
			count = 3; break;
		case BT_CONSTRAINT_ELEMENT_TETRAHEDRON:
			if (m_object.m_domain != BT_CONSTRAINT_VOLUME || m_element < 0) return btConstraintStatus(BT_CONSTRAINT_INVALID_PARAMETER, "invalid volume tetrahedron");
			count = 4; break;
		default: return btConstraintStatus(BT_CONSTRAINT_INVALID_PARAMETER, "unknown element kind");
		}
		btScalar sum = 0;
		for (int i = 0; i < 4; ++i)
		{
			if (!btConstraintScalarFinite(m_weights[i]) || (i < count ? m_weights[i] < 0 : m_weights[i] != 0))
				return btConstraintStatus(BT_CONSTRAINT_INVALID_PARAMETER, "invalid barycentric weights", i);
			sum += m_weights[i];
		}
		if (count && btFabs(sum - 1) > btScalar(1e-5)) return btConstraintStatus(BT_CONSTRAINT_INVALID_PARAMETER, "barycentric weights must sum to one");
		return btConstraintStatus();
	}
};

struct btConstraintAttachmentParameters
{
	uint64_t m_ownerId;
	btConstraintElementPoint m_points[2];
	btScalar m_compliance, m_damping;
	bool m_enabled, m_collisionEnabled;
	btConstraintAttachmentParameters() : m_ownerId(0), m_compliance(0), m_damping(0), m_enabled(true), m_collisionEnabled(true) {}
};

struct btConstraintElementDelta
{
	BT_DECLARE_ALIGNED_ALLOCATOR();
	btConstraintElementPoint m_point;
	btVector3 m_positionDelta, m_impulse;
	uint64_t m_ownerId;
	btConstraintElementDelta() : m_positionDelta(0, 0, 0), m_impulse(0, 0, 0), m_ownerId(0) {}
};

class btConstraintElementContext
{
public:
	virtual ~btConstraintElementContext() {}
	virtual btConstraintStatus readPoint(const btConstraintElementPoint& point, btVector3& position, btVector3& velocity) const = 0;
	// Validate every endpoint first, then apply the complete batch. No partial
	// write on failure. The backend supplies barycentric and RB/MB reactions.
	virtual btConstraintStatus applyDeltas(const btAlignedObjectArray<btConstraintElementDelta>& deltas) = 0;
};

// PBD/FEM/attachment operators have their own consumer interface. They are not
// required to pretend that density/volume/energy is one two-body scalar row.
class btConstraintElementOperator
{
public:
	virtual ~btConstraintElementOperator() {}
	virtual btConstraintCategoryId getCategory() const = 0;
	virtual btConstraintStatus validate(const btConstraintElementContext& context) const = 0;
	virtual btConstraintStatus prepare(const btConstraintFrameSnapshot& frame, const btConstraintElementContext& context) = 0;
	virtual btConstraintStatus project(btConstraintElementContext& context) = 0;
	virtual btConstraintStatus getResidual(btScalar& residual) const = 0;
	virtual void invalidate() = 0;
};

#endif
