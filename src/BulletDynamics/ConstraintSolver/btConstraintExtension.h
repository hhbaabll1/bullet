/*
Bullet Continuous Collision Detection and Physics Library
New constraint extension interfaces, 2026. Distributed under the Bullet zlib license.
This is a new extension, not an unmodified upstream Bullet or PhysX source file.
*/
#ifndef BT_CONSTRAINT_EXTENSION_H
#define BT_CONSTRAINT_EXTENSION_H

#include "LinearMath/btAlignedObjectArray.h"
#include "LinearMath/btTransform.h"
#include <stdint.h>

// btAlignedObjectArray::operator= placement-constructs over existing elements.
// Constraint records own nested arrays, so destroy their previous values before
// copying. Keep this behavior local to the opt-in API, without changing Bullet's
// original container. Construct copies directly instead of resize + placement.
template <typename T>
class btConstraintValueArray : public btAlignedObjectArray<T>
{
	typedef btAlignedObjectArray<T> Base;
public:
	btConstraintValueArray() {}
	btConstraintValueArray(const btConstraintValueArray& other) { *this = static_cast<const Base&>(other); }
	btConstraintValueArray& operator=(const btConstraintValueArray& other) { return *this = static_cast<const Base&>(other); }
	btConstraintValueArray& operator=(const Base& other)
	{
		if (static_cast<const Base*>(this) != &other)
		{
			Base::clear(); Base::reserve(other.size());
			for (int i = 0; i < other.size(); ++i) Base::push_back(other[i]);
		}
		return *this;
	}
};

// These IDs belong to a separate schema. Never cast them to btTypedConstraintType.
enum btConstraintCategoryId
{
	BT_CONSTRAINT_CATEGORY_FIXED = 0x1001,
	BT_CONSTRAINT_CATEGORY_REVOLUTE = 0x1002,
	BT_CONSTRAINT_CATEGORY_PRISMATIC = 0x1003,
	BT_CONSTRAINT_CATEGORY_SPHERICAL = 0x1004,
	BT_CONSTRAINT_CATEGORY_DISTANCE = 0x1005,
	BT_CONSTRAINT_CATEGORY_D6 = 0x1006,
	BT_CONSTRAINT_CATEGORY_GEAR = 0x1007,
	BT_CONSTRAINT_CATEGORY_RACK_AND_PINION = 0x1008,
	BT_CONSTRAINT_CATEGORY_ARTICULATION_JOINT = 0x2001,
	BT_CONSTRAINT_CATEGORY_AXIS_LIMIT = 0x2002,
	BT_CONSTRAINT_CATEGORY_AXIS_DRIVE = 0x2003,
	BT_CONSTRAINT_CATEGORY_AXIS_FRICTION = 0x2004,
	BT_CONSTRAINT_CATEGORY_VELOCITY_LIMIT = 0x2005,
	BT_CONSTRAINT_CATEGORY_MIMIC = 0x3001,
	BT_CONSTRAINT_CATEGORY_FIXED_TENDON = 0x3002,
	BT_CONSTRAINT_CATEGORY_SPATIAL_TENDON = 0x3003,
	BT_CONSTRAINT_CATEGORY_LOOP = 0x3004,
	BT_CONSTRAINT_CATEGORY_CONTACT = 0x4001,
	BT_CONSTRAINT_CATEGORY_CONTACT_FRICTION = 0x4002,
	BT_CONSTRAINT_CATEGORY_TORSIONAL_FRICTION = 0x4003,
	BT_CONSTRAINT_CATEGORY_MOBILITY = 0x4004,
	BT_CONSTRAINT_CATEGORY_CUSTOM = 0x5001,
	BT_CONSTRAINT_CATEGORY_PULLEY = 0x5002,
	BT_CONSTRAINT_CATEGORY_SUSPENSION_STOP = 0x5003,
	BT_CONSTRAINT_CATEGORY_STICKY_TIRE = 0x5004,
	BT_CONSTRAINT_CATEGORY_PARTICLE_DENSITY = 0x6001,
	BT_CONSTRAINT_CATEGORY_PARTICLE_CONTACT = 0x6002,
	BT_CONSTRAINT_CATEGORY_CLOTH_SPRING = 0x6003,
	BT_CONSTRAINT_CATEGORY_INFLATABLE = 0x6004,
	BT_CONSTRAINT_CATEGORY_SHAPE_MATCHING = 0x6005,
	BT_CONSTRAINT_CATEGORY_NODE_MOTION = 0x6006,
	BT_CONSTRAINT_CATEGORY_FEM_SURFACE = 0x7001,
	BT_CONSTRAINT_CATEGORY_FEM_BENDING = 0x7002,
	BT_CONSTRAINT_CATEGORY_FEM_VOLUME = 0x7003,
	BT_CONSTRAINT_CATEGORY_ATTACHMENT = 0x8001,
	BT_CONSTRAINT_CATEGORY_CROSS_DOMAIN_CONTACT = 0x8002
};

enum btConstraintObjectDomain
{
	BT_CONSTRAINT_WORLD = 0,
	BT_CONSTRAINT_RIGID_BODY = 1,
	BT_CONSTRAINT_MULTIBODY_LINK = 2,
	BT_CONSTRAINT_PARTICLE = 3,
	BT_CONSTRAINT_SURFACE = 4,
	BT_CONSTRAINT_VOLUME = 5
};
enum btConstraintMotion { BT_CONSTRAINT_LOCKED, BT_CONSTRAINT_LIMITED, BT_CONSTRAINT_FREE };
enum btConstraintArticulationJointType { BT_CONSTRAINT_ART_FIXED, BT_CONSTRAINT_ART_PRISMATIC, BT_CONSTRAINT_ART_REVOLUTE, BT_CONSTRAINT_ART_REVOLUTE_UNWRAPPED, BT_CONSTRAINT_ART_SPHERICAL };
enum btConstraintAngularDriveConfiguration { BT_CONSTRAINT_DRIVE_LEGACY, BT_CONSTRAINT_DRIVE_SWING_TWIST, BT_CONSTRAINT_DRIVE_SLERP };
enum btConstraintAxis { BT_CONSTRAINT_X, BT_CONSTRAINT_Y, BT_CONSTRAINT_Z, BT_CONSTRAINT_TWIST, BT_CONSTRAINT_SWING1, BT_CONSTRAINT_SWING2 };
enum btConstraintBoundUnit { BT_CONSTRAINT_IMPULSE, BT_CONSTRAINT_FORCE };
enum btConstraintSpringMode { BT_CONSTRAINT_NO_SPRING, BT_CONSTRAINT_FORCE_SPRING, BT_CONSTRAINT_ACCELERATION_SPRING };
enum btConstraintBiasMode { BT_CONSTRAINT_NO_BIAS, BT_CONSTRAINT_ERP_BIAS, BT_CONSTRAINT_KEEP_BIAS };
enum btConstraintFrameSpace { BT_CONSTRAINT_COM_FRAME, BT_CONSTRAINT_ACTOR_FRAME, BT_CONSTRAINT_WORLD_FRAME };
enum btConstraintStatusCode
{
	BT_CONSTRAINT_OK, BT_CONSTRAINT_INVALID_PARAMETER, BT_CONSTRAINT_UNSUPPORTED,
	BT_CONSTRAINT_MISSING_REFERENCE, BT_CONSTRAINT_DUPLICATE_ID, BT_CONSTRAINT_STALE_FRAME,
	BT_CONSTRAINT_INVALID_FORMAT
};

struct btConstraintStatus
{
	btConstraintStatusCode m_code;
	int m_index;
	const char* m_message;  // Static diagnostic text, never owned.
	btConstraintStatus(btConstraintStatusCode code = BT_CONSTRAINT_OK, const char* message = "ok", int index = -1)
		: m_code(code), m_index(index), m_message(message) {}
	bool isOk() const { return m_code == BT_CONSTRAINT_OK; }
};

struct btConstraintObjectRef
{
	uint64_t m_id;  // Zero is the world. IDs are application-assigned, never pointer casts.
	btConstraintObjectDomain m_domain;
	int m_link;  // MultiBody base = -1; non-MultiBody = -1.
	btConstraintObjectRef(uint64_t id = 0, btConstraintObjectDomain domain = BT_CONSTRAINT_WORLD, int link = -1)
		: m_id(id), m_domain(domain), m_link(link) {}
	bool operator==(const btConstraintObjectRef& other) const
	{ return m_id == other.m_id && m_domain == other.m_domain && m_link == other.m_link; }
};

struct btConstraintLimitParameters
{
	btScalar m_lower, m_upper, m_restitution, m_bounceThreshold, m_stiffness, m_damping;
	btConstraintLimitParameters() : m_lower(0), m_upper(0), m_restitution(0), m_bounceThreshold(0), m_stiffness(0), m_damping(0) {}
};
struct btConstraintDriveParameters
{
	btScalar m_position, m_velocity, m_stiffness, m_damping, m_maxEffort;
	btConstraintSpringMode m_mode;
	btConstraintBoundUnit m_effortUnit;
	bool m_enabled, m_freeSpin, m_outputForce;
	btConstraintDriveParameters() : m_position(0), m_velocity(0), m_stiffness(0), m_damping(0), m_maxEffort(0),
		m_mode(BT_CONSTRAINT_NO_SPRING), m_effortUnit(BT_CONSTRAINT_FORCE), m_enabled(false), m_freeSpin(false), m_outputForce(false) {}
};
struct btConstraintFrictionParameters
{
	btScalar m_staticEffort, m_dynamicEffort, m_viscousCoefficient, m_legacyCoefficient;
	bool m_useLegacy;
	btConstraintFrictionParameters() : m_staticEffort(0), m_dynamicEffort(0), m_viscousCoefficient(0), m_legacyCoefficient(0), m_useLegacy(false) {}
};
struct btConstraintEnvelopeParameters
{
	btScalar m_maxEffort, m_velocityDependentResistance, m_maxActuatorVelocity, m_speedEffortGradient;
	btConstraintEnvelopeParameters() : m_maxEffort(0), m_velocityDependentResistance(0), m_maxActuatorVelocity(0), m_speedEffortGradient(0) {}
};
struct btConstraintAxisParameters
{
	btConstraintMotion m_motion;
	btConstraintLimitParameters m_limit;
	btConstraintDriveParameters m_drive;
	btConstraintFrictionParameters m_friction;
	btConstraintEnvelopeParameters m_envelope;
	btScalar m_armature, m_maxVelocity;
	btConstraintAxisParameters() : m_motion(BT_CONSTRAINT_LOCKED), m_armature(0), m_maxVelocity(0) {}
};
struct btConstraintLocalFrame
{
	BT_DECLARE_ALIGNED_ALLOCATOR();
	btConstraintFrameSpace m_space;
	btTransform m_transform;
	btConstraintLocalFrame() : m_space(BT_CONSTRAINT_COM_FRAME), m_transform(btTransform::getIdentity()) {}
};

// Neutral extension defaults; per-category factories explicitly apply the
// corresponding source contract without changing existing Bullet defaults.
struct btConstraintCommonParameters
{
	bool m_enabled, m_collisionEnabled, m_outputForce, m_autoWake;
	btScalar m_breakForce, m_breakTorque;
	btScalar m_inverseMassScale[2], m_inverseInertiaScale[2];
	btConstraintCommonParameters() : m_enabled(true), m_collisionEnabled(false), m_outputForce(false), m_autoWake(true),
		m_breakForce(SIMD_INFINITY), m_breakTorque(SIMD_INFINITY)
	{ for (int i = 0; i < 2; ++i) { m_inverseMassScale[i] = 1; m_inverseInertiaScale[i] = 1; } }
};

// The application resolves actor/COM offsets explicitly. A Bullet rigid-body
// world transform is a COM transform, not necessarily an actor transform.
struct btConstraintCoordinateState
{
	BT_DECLARE_ALIGNED_ALLOCATOR();
	btTransform m_comWorld, m_actorWorld;
	btVector3 m_linearVelocity, m_angularVelocity;
	btAlignedObjectArray<btScalar> m_position, m_velocity;
	btConstraintCoordinateState() : m_comWorld(btTransform::getIdentity()), m_actorWorld(btTransform::getIdentity()),
		m_linearVelocity(0, 0, 0), m_angularVelocity(0, 0, 0) {}
};
class btConstraintCoordinateProvider
{
public:
	virtual ~btConstraintCoordinateProvider() {}
	// Read only. Capture is called by the application before the solver; never by getInfo*.
	virtual btConstraintStatus read(const btConstraintObjectRef& object, btConstraintCoordinateState& out) const = 0;
};
struct btConstraintFrameSnapshot
{
	uint64_t m_step;
	btScalar m_timeStep;
	btAlignedObjectArray<btConstraintObjectRef> m_objects;
	btConstraintValueArray<btConstraintCoordinateState> m_states;
	btConstraintFrameSnapshot() : m_step(0), m_timeStep(0) {}
	btConstraintStatus capture(uint64_t step, btScalar dt, const btAlignedObjectArray<btConstraintObjectRef>& objects,
		const btConstraintCoordinateProvider& provider);
};
class btConstraintAngleTracker
{
	uint64_t m_step;
	btScalar m_wrapped, m_continuous;
	bool m_initialized;
public:
	btConstraintAngleTracker() : m_step(0), m_wrapped(0), m_continuous(0), m_initialized(false) {}
	btConstraintStatus prepare(uint64_t step, btScalar wrappedAngle);
	btScalar getAngle() const { return m_continuous; }
	void reset() { m_initialized = false; }
};

struct btConstraintJacobian
{
	BT_DECLARE_ALIGNED_ALLOCATOR();
	btVector3 m_linear, m_angular;  // World basis, velocity measured at COM.
	// MultiBody generalized order: base omega, base linear velocity, joint DOFs.
	btAlignedObjectArray<btScalar> m_generalized;
	btConstraintJacobian() : m_linear(0, 0, 0), m_angular(0, 0, 0) {}
};
struct btConstraintJacobianTerm
{
	BT_DECLARE_ALIGNED_ALLOCATOR();
	btConstraintObjectRef m_object;
	btConstraintJacobian m_measurement, m_response; // Intentionally distinct (fixed tendons).
	btScalar m_inverseMassScale, m_inverseInertiaScale;
	btConstraintJacobianTerm() : m_inverseMassScale(1), m_inverseInertiaScale(1) {}
};
struct btConstraintRowState
{
	btScalar m_accumulatedImpulse;
	uint64_t m_lastStep;
	btConstraintRowState() : m_accumulatedImpulse(0), m_lastStep(0) {}
};
struct btConstraintRowDescriptor
{
	uint64_t m_ownerId, m_rowId;
	btConstraintValueArray<btConstraintJacobianTerm> m_terms;
	// Convention: q=geometricError; corrective target is targetVelocity - ERP*q/dt.
	btScalar m_geometricError, m_targetVelocity, m_lowerBound, m_upperBound;
	btConstraintBoundUnit m_boundUnit;
	btConstraintSpringMode m_springMode;
	btConstraintBiasMode m_biasMode;
	btScalar m_erp, m_stiffness, m_damping, m_restitution, m_bounceThreshold;
	bool m_outputForce, m_angular, m_dynamicBounds;
	btConstraintRowState m_state;
	btConstraintRowDescriptor() : m_ownerId(0), m_rowId(0), m_geometricError(0), m_targetVelocity(0),
		m_lowerBound(-SIMD_INFINITY), m_upperBound(SIMD_INFINITY), m_boundUnit(BT_CONSTRAINT_IMPULSE),
		m_springMode(BT_CONSTRAINT_NO_SPRING), m_biasMode(BT_CONSTRAINT_NO_BIAS), m_erp(btScalar(0.2)),
		m_stiffness(0), m_damping(0), m_restitution(0), m_bounceThreshold(0),
		m_outputForce(false), m_angular(false), m_dynamicBounds(false) {}
};

btConstraintStatus btValidateConstraintRow(const btConstraintRowDescriptor& row);
btConstraintStatus btValidateConstraintFrameSnapshot(const btConstraintFrameSnapshot& frame);
btConstraintStatus btValidateNativeHardConstraintRow(const btConstraintRowDescriptor& row);
bool btConstraintJacobianEqual(const btConstraintJacobian& a, const btConstraintJacobian& b);
bool btConstraintScalarFinite(btScalar value);
btConstraintStatus btConstraintBoundsToImpulse(btScalar lower, btScalar upper, btConstraintBoundUnit unit,
	btScalar dt, btScalar& impulseLower, btScalar& impulseUpper);
btTransform btConstraintFrameToWorld(const btConstraintLocalFrame& local, const btConstraintCoordinateState& object);
btScalar btConstraintTargetVelocity(const btConstraintRowDescriptor& row, btScalar dt);

// Version 1 describes common parameters and references, NOT a .bullet runtime snapshot.
// Category-specific data is versioned opaque payload until its category module exists.
struct btConstraintDescription
{
	BT_DECLARE_ALIGNED_ALLOCATOR();
	uint64_t m_id;
	btConstraintCategoryId m_category;
	btAlignedObjectArray<btConstraintObjectRef> m_objects;
	btAlignedObjectArray<uint64_t> m_constraintReferences;
	btConstraintLocalFrame m_frameA, m_frameB;
	btConstraintCommonParameters m_common;
	btConstraintAxisParameters m_axes[6];
	unsigned int m_payloadVersion;
	btAlignedObjectArray<unsigned char> m_payload;
	btConstraintDescription() : m_id(0), m_category(BT_CONSTRAINT_CATEGORY_CUSTOM), m_payloadVersion(0) {}
};
struct btConstraintDescriptionGraph
{
	unsigned int m_schemaVersion;
	btAlignedObjectArray<btConstraintObjectRef> m_objects;
	btConstraintValueArray<btConstraintDescription> m_constraints;
	btConstraintDescriptionGraph() : m_schemaVersion(1) {}
};
class btConstraintDescriptionImporter
{
public:
	static btConstraintStatus validate(const btConstraintDescriptionGraph& graph);
	// Transactional: destination remains unchanged on failure. No physics objects created.
	static btConstraintStatus load(const btConstraintDescriptionGraph& source, btConstraintDescriptionGraph& destination);
	// Portable little-endian encoding; scalar values are IEEE binary64, IDs uint64.
	static btConstraintStatus writeBinary(const btConstraintDescriptionGraph& source, btAlignedObjectArray<unsigned char>& bytes);
	static btConstraintStatus readBinary(const unsigned char* bytes, int size, btConstraintDescriptionGraph& destination);
};
class btConstraintRuntimeFactory
{
public:
	virtual ~btConstraintRuntimeFactory() {}
	// Category implementations must explicitly opt in. Loading descriptions is not implementation.
	virtual btConstraintStatus canCreate(const btConstraintDescription& description) const = 0;
	virtual btConstraintStatus create(const btConstraintDescription& description) = 0;
	virtual void remove(uint64_t constraintId) = 0;
};

#endif
