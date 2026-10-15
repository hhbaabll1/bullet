/* Bullet extension, 2026. Distributed under the Bullet zlib license.
   Distance row geometry is adapted from PhysX 5.6.1 ExtDistanceJoint.cpp;
   see btDistanceConstraint.cpp for the retained source license. */
#ifndef BT_DISTANCE_CONSTRAINT_H
#define BT_DISTANCE_CONSTRAINT_H
#include "btTypedConstraint.h"
#include "btConstraintExtension.h"

class btMultiBody;
class btMultiBodyDynamicsWorld;
class btDiscreteDynamicsWorld;
class btIDebugDraw;

enum btDistanceConstraintFlags
{
	BT_DISTANCE_MIN_ENABLED = 1,
	BT_DISTANCE_MAX_ENABLED = 2,
	BT_DISTANCE_SPRING_ENABLED = 4
};
struct btDistanceConstraintParameters
{
	btScalar m_minDistance, m_maxDistance, m_tolerance, m_stiffness, m_damping;
	unsigned int m_flags;
	explicit btDistanceConstraintParameters(btScalar lengthScale = 1)
		: m_minDistance(0), m_maxDistance(0), m_tolerance(btScalar(0.025)*lengthScale),
		m_stiffness(0), m_damping(0), m_flags(BT_DISTANCE_MAX_ENABLED) {}
	btConstraintStatus validate() const;
};
struct btDistanceConstraintRow
{
	btScalar m_error, m_lower, m_upper;
	bool m_keepBias;
};
// Portable, solver-independent row builder. The two-bound mode always emits two rows.
int btBuildDistanceConstraintRows(const btDistanceConstraintParameters& parameters,
	btScalar distance, btDistanceConstraintRow rows[2]);
// Map an implicit force spring (or hard ERP row) into Bullet's SI coefficients.
btConstraintStatus btCompleteDistanceConstraintRow(const btDistanceConstraintParameters& parameters,
	const btDistanceConstraintRow& row, btScalar response, btScalar velocity, btScalar dt, btScalar erp,
	btScalar& diagonal, btScalar& rhs, btScalar& cfm);

struct btDistanceConstraintEndpoint
{
	BT_DECLARE_ALIGNED_ALLOCATOR();
	btRigidBody* m_rigidBody;
	btMultiBody* m_multiBody;
	int m_link;
	// Maps actor-local coordinates into COM/link-local coordinates.
	btTransform m_actorToCom;
	btDistanceConstraintEndpoint() : m_rigidBody(0), m_multiBody(0), m_link(-1), m_actorToCom(btTransform::getIdentity()) {}
	explicit btDistanceConstraintEndpoint(btRigidBody* body) : m_rigidBody(body), m_multiBody(0), m_link(-1), m_actorToCom(btTransform::getIdentity()) {}
	btDistanceConstraintEndpoint(btMultiBody* body, int link) : m_rigidBody(0), m_multiBody(body), m_link(link), m_actorToCom(btTransform::getIdentity()) {}
	btTransform comWorld() const;
	btTransform frameWorld(const btConstraintLocalFrame& frame) const;
	btCollisionObject* collider() const;
	void wake() const;
	bool valid() const;
	bool sameObject(const btDistanceConstraintEndpoint& other) const;
};

// Shared category state. Endpoints are borrowed and must outlive the constraint.
class btDistanceConstraintState
{
	btDistanceConstraintState(const btDistanceConstraintState&);
	btDistanceConstraintState& operator=(const btDistanceConstraintState&);
protected:
	btDistanceConstraintEndpoint m_endpoints[2];
	btConstraintLocalFrame m_frames[2];
	btDistanceConstraintParameters m_parameters;
	btConstraintCommonParameters m_common;
	btConstraintStatus m_status;
	btDistanceConstraintRow m_rows[2];
	btVector3 m_points[2], m_normal;
	btJointFeedback m_feedback;
	btScalar m_distance, m_sumImpulse, m_dt;
	int m_rowCount;
	bool m_broken;
	int prepareDistance();
	void finishDistanceRow(btScalar impulse, btScalar dt);
public:
	BT_DECLARE_ALIGNED_ALLOCATOR();
	btDistanceConstraintState(const btDistanceConstraintEndpoint& a, const btDistanceConstraintEndpoint& b,
		const btVector3& pivotA, const btVector3& pivotB);
	virtual ~btDistanceConstraintState() {}
	btConstraintStatus setParameters(const btDistanceConstraintParameters& parameters);
	const btDistanceConstraintParameters& getParameters() const { return m_parameters; }
	btConstraintStatus setCommonParameters(const btConstraintCommonParameters& common);
	const btConstraintCommonParameters& getCommonParameters() const { return m_common; }
	btConstraintStatus setFrames(const btConstraintLocalFrame& a, const btConstraintLocalFrame& b);
	btScalar getDistance() const;
	const btConstraintStatus& getStatus() const { return m_status; }
	const btJointFeedback& getDistanceFeedback() const { return m_feedback; }
	bool isBroken() const { return m_broken; }
	void repair();
	void debugDrawDistance(btIDebugDraw* drawer) const;
	btConstraintStatus writeDescription(uint64_t id, const btConstraintObjectRef& a,
		const btConstraintObjectRef& b, btConstraintDescription& description) const;
	btConstraintStatus readDescription(const btConstraintDescription& description);
	static btConstraintStatus validateDescription(const btConstraintDescription& description);
};

class btDistanceConstraint : public btTypedConstraint, public btDistanceConstraintState
{
public:
	BT_DECLARE_ALIGNED_ALLOCATOR();
	btDistanceConstraint(btRigidBody& a, btRigidBody& b, const btVector3& pivotA, const btVector3& pivotB);
	btDistanceConstraint(btRigidBody& a, const btVector3& pivotA, const btVector3& worldPivot);
	virtual void getInfo1(btConstraintInfo1* info);
	virtual void getInfo2(btConstraintInfo2* info);
	virtual bool finalizeExtensionRow(btSolverConstraint& row, int index, const btContactSolverInfo& info,
		const btSolverBody& a, const btSolverBody& b);
	virtual void onExtensionRowSolved(int index, btScalar impulse, btScalar dt);
	virtual void setParam(int number, btScalar value, int axis = -1);
	virtual btScalar getParam(int number, int axis = -1) const;
};

// Application IDs resolve to existing Bullet bodies; no Px types or pointers are serialized.
class btDistanceConstraintResolver
{
public:
	virtual ~btDistanceConstraintResolver() {}
	virtual btConstraintStatus resolve(const btConstraintObjectRef& reference, btDistanceConstraintEndpoint& endpoint) const = 0;
};
class btDistanceConstraintFactory : public btConstraintRuntimeFactory
{
	struct Entry
	{
		uint64_t m_id;
		btDistanceConstraintState* m_state;
		btTypedConstraint* m_rigid;
		class btMultiBodyDistanceConstraint* m_multi;
		btConstraintObjectRef m_refs[2];
		btCollisionObject* m_colliders[2];
		bool m_ignored;
		bool m_swapped;
	};
	btDiscreteDynamicsWorld& m_world;
	btMultiBodyDynamicsWorld* m_multiWorld;
	const btDistanceConstraintResolver& m_resolver;
	btAlignedObjectArray<Entry> m_entries;
	btConstraintStatus resolveDescription(const btConstraintDescription&, btDistanceConstraintEndpoint (&)[2]) const;
	void updateCollision(Entry& entry, bool enabled);
	btDistanceConstraintFactory(const btDistanceConstraintFactory&);
	btDistanceConstraintFactory& operator=(const btDistanceConstraintFactory&);
public:
	btDistanceConstraintFactory(btDiscreteDynamicsWorld& world, const btDistanceConstraintResolver& resolver);
	btDistanceConstraintFactory(btMultiBodyDynamicsWorld& world, const btDistanceConstraintResolver& resolver);
	virtual ~btDistanceConstraintFactory();
	virtual btConstraintStatus canCreate(const btConstraintDescription& description) const;
	virtual btConstraintStatus create(const btConstraintDescription& description);
	virtual void remove(uint64_t id);
	// Remove every referencing constraint BEFORE removing or destroying a body/link.
	void removeObject(const btConstraintObjectRef& reference);
	btConstraintStatus update(uint64_t id, const btConstraintDescription& description);
	btDistanceConstraintState* find(uint64_t id) const;
};
#endif
