/* New Bullet constraint extension interfaces. Distributed under the Bullet zlib license. */
#include "btConstraintExtension.h"
#include <cmath>
#include <cstring>
#include <limits>

namespace
{
btConstraintStatus invalid(const char* text, int index = -1) { return btConstraintStatus(BT_CONSTRAINT_INVALID_PARAMETER, text, index); }
bool nonnegative(btScalar x) { return btConstraintScalarFinite(x) && x >= 0; }
bool vectorFinite(const btVector3& v) { return btConstraintScalarFinite(v.x()) && btConstraintScalarFinite(v.y()) && btConstraintScalarFinite(v.z()); }
bool objectValid(const btConstraintObjectRef& object)
{
	return object.m_domain >= BT_CONSTRAINT_WORLD && object.m_domain <= BT_CONSTRAINT_VOLUME &&
		((object.m_domain == BT_CONSTRAINT_WORLD) == (object.m_id == 0)) &&
		(object.m_domain == BT_CONSTRAINT_MULTIBODY_LINK ? object.m_link >= -1 : object.m_link == -1);
}
bool categoryValid(unsigned int value)
{
	return (value >= 0x1001 && value <= 0x1008) || (value >= 0x2001 && value <= 0x2005) ||
		(value >= 0x3001 && value <= 0x3004) || (value >= 0x4001 && value <= 0x4004) ||
		(value >= 0x5001 && value <= 0x5004) || (value >= 0x6001 && value <= 0x6006) ||
		(value >= 0x7001 && value <= 0x7003) || (value >= 0x8001 && value <= 0x8002);
}
bool frameValid(const btConstraintLocalFrame& frame)
{
	if (frame.m_space < BT_CONSTRAINT_COM_FRAME || frame.m_space > BT_CONSTRAINT_WORLD_FRAME || !vectorFinite(frame.m_transform.getOrigin())) return false;
	const btMatrix3x3& b = frame.m_transform.getBasis();
	for (int i = 0; i < 3; ++i)
	{
		if (!vectorFinite(b[i]) || btFabs(b[i].length2() - 1) > btScalar(1e-4)) return false;
		for (int j = i + 1; j < 3; ++j) if (btFabs(b[i].dot(b[j])) > btScalar(1e-4)) return false;
	}
	return btFabs(b.determinant() - 1) <= btScalar(1e-4);
}
bool jacobianFinite(const btConstraintJacobian& j)
{
	if (!vectorFinite(j.m_linear) || !vectorFinite(j.m_angular)) return false;
	for (int i = 0; i < j.m_generalized.size(); ++i) if (!btConstraintScalarFinite(j.m_generalized[i])) return false;
	return true;
}
bool axisValid(const btConstraintAxisParameters& a)
{
	return a.m_motion >= BT_CONSTRAINT_LOCKED && a.m_motion <= BT_CONSTRAINT_FREE &&
		btConstraintScalarFinite(a.m_limit.m_lower) && btConstraintScalarFinite(a.m_limit.m_upper) && a.m_limit.m_lower <= a.m_limit.m_upper &&
		nonnegative(a.m_limit.m_stiffness) && nonnegative(a.m_limit.m_damping) && nonnegative(a.m_limit.m_bounceThreshold) &&
		nonnegative(a.m_limit.m_restitution) && a.m_limit.m_restitution <= 1 &&
		btConstraintScalarFinite(a.m_drive.m_position) && btConstraintScalarFinite(a.m_drive.m_velocity) &&
		nonnegative(a.m_drive.m_stiffness) && nonnegative(a.m_drive.m_damping) && nonnegative(a.m_drive.m_maxEffort) &&
		a.m_drive.m_mode >= BT_CONSTRAINT_NO_SPRING && a.m_drive.m_mode <= BT_CONSTRAINT_ACCELERATION_SPRING &&
		a.m_drive.m_effortUnit >= BT_CONSTRAINT_IMPULSE && a.m_drive.m_effortUnit <= BT_CONSTRAINT_FORCE &&
		nonnegative(a.m_friction.m_staticEffort) && nonnegative(a.m_friction.m_dynamicEffort) &&
		a.m_friction.m_staticEffort >= a.m_friction.m_dynamicEffort && nonnegative(a.m_friction.m_viscousCoefficient) && nonnegative(a.m_friction.m_legacyCoefficient) &&
		nonnegative(a.m_envelope.m_maxEffort) && nonnegative(a.m_envelope.m_velocityDependentResistance) &&
		nonnegative(a.m_envelope.m_maxActuatorVelocity) && nonnegative(a.m_envelope.m_speedEffortGradient) &&
		nonnegative(a.m_armature) && nonnegative(a.m_maxVelocity);
}
bool commonValid(const btConstraintCommonParameters& c)
{
	return nonnegative(c.m_breakForce) && nonnegative(c.m_breakTorque) &&
		nonnegative(c.m_inverseMassScale[0]) && nonnegative(c.m_inverseMassScale[1]) &&
		nonnegative(c.m_inverseInertiaScale[0]) && nonnegative(c.m_inverseInertiaScale[1]);
}
const int kMaxRecords = 4096;
const int kMaxPayload = 1024 * 1024;
const uint64_t kMaxGraphBytes = 16 * 1024 * 1024;
struct BinaryWriter
{
	btAlignedObjectArray<unsigned char>& m_bytes;
	explicit BinaryWriter(btAlignedObjectArray<unsigned char>& bytes) : m_bytes(bytes) {}
	void u64(uint64_t x) { for (int i = 0; i < 8; ++i) { m_bytes.push_back(static_cast<unsigned char>(x & 255)); x >>= 8; } }
	void scalar(btScalar value) { double d = static_cast<double>(value); uint64_t bits; std::memcpy(&bits, &d, 8); u64(bits); }
	void flag(bool v) { u64(v ? 1 : 0); }
};
struct BinaryReader
{
	const unsigned char* m_bytes;
	int m_size, m_offset;
	bool m_ok;
	BinaryReader(const unsigned char* bytes, int size) : m_bytes(bytes), m_size(size), m_offset(0), m_ok(bytes != 0 && size >= 0) {}
	uint64_t u64()
	{
		if (!m_ok || m_size - m_offset < 8) { m_ok = false; return 0; }
		uint64_t result = 0; for (int i = 0; i < 8; ++i) result |= uint64_t(m_bytes[m_offset++]) << (8 * i); return result;
	}
	btScalar scalar()
	{
		uint64_t bits = u64(); double value; std::memcpy(&value, &bits, 8);
		if (!std::isfinite(value) || std::fabs(value) > double(std::numeric_limits<btScalar>::max())) { m_ok = false; return 0; }
		return static_cast<btScalar>(value);
	}
	int count(int max = kMaxRecords) { uint64_t n = u64(); if (n > uint64_t(max)) { m_ok = false; return 0; } return int(n); }
	bool flag() { uint64_t n = u64(); if (n > 1) m_ok = false; return n == 1; }
	unsigned int enumeration(unsigned int max = 0xffff) { uint64_t n = u64(); if (n > max) { m_ok = false; return 0; } return static_cast<unsigned int>(n); }
};
void writeRef(BinaryWriter& w, const btConstraintObjectRef& o) { w.u64(o.m_id); w.u64(o.m_domain); w.u64(uint64_t(int64_t(o.m_link))); }
btConstraintObjectRef readRef(BinaryReader& r)
{
	btConstraintObjectRef o; o.m_id = r.u64(); o.m_domain = btConstraintObjectDomain(r.enumeration(BT_CONSTRAINT_VOLUME));
	uint64_t link = r.u64(); if (link == UINT64_MAX) o.m_link = -1; else if (link <= uint64_t(0x7fffffff)) o.m_link = int(link); else r.m_ok = false;
	return o;
}
void writeFrame(BinaryWriter& w, const btConstraintLocalFrame& f)
{
	w.u64(f.m_space); for (int i = 0; i < 3; ++i) w.scalar(f.m_transform.getOrigin()[i]);
	for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j) w.scalar(f.m_transform.getBasis()[i][j]);
}
void readFrame(BinaryReader& r, btConstraintLocalFrame& f)
{
	f.m_space = btConstraintFrameSpace(r.enumeration(BT_CONSTRAINT_WORLD_FRAME));
	for (int i = 0; i < 3; ++i) f.m_transform.getOrigin()[i] = r.scalar();
	for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j) f.m_transform.getBasis()[i][j] = r.scalar();
}
void writeCommon(BinaryWriter& w, const btConstraintCommonParameters& c)
{
	w.flag(c.m_enabled); w.flag(c.m_collisionEnabled); w.flag(c.m_outputForce); w.flag(c.m_autoWake);
	w.scalar(c.m_breakForce); w.scalar(c.m_breakTorque);
	for (int i = 0; i < 2; ++i) { w.scalar(c.m_inverseMassScale[i]); w.scalar(c.m_inverseInertiaScale[i]); }
}
void readCommon(BinaryReader& r, btConstraintCommonParameters& c)
{
	c.m_enabled = r.flag(); c.m_collisionEnabled = r.flag(); c.m_outputForce = r.flag(); c.m_autoWake = r.flag();
	c.m_breakForce = r.scalar(); c.m_breakTorque = r.scalar();
	for (int i = 0; i < 2; ++i) { c.m_inverseMassScale[i] = r.scalar(); c.m_inverseInertiaScale[i] = r.scalar(); }
}
void writeAxis(BinaryWriter& w, const btConstraintAxisParameters& a)
{
	w.u64(a.m_motion);
	w.scalar(a.m_limit.m_lower); w.scalar(a.m_limit.m_upper); w.scalar(a.m_limit.m_restitution); w.scalar(a.m_limit.m_bounceThreshold); w.scalar(a.m_limit.m_stiffness); w.scalar(a.m_limit.m_damping);
	w.scalar(a.m_drive.m_position); w.scalar(a.m_drive.m_velocity); w.scalar(a.m_drive.m_stiffness); w.scalar(a.m_drive.m_damping); w.scalar(a.m_drive.m_maxEffort);
	w.u64(a.m_drive.m_mode); w.u64(a.m_drive.m_effortUnit); w.flag(a.m_drive.m_enabled); w.flag(a.m_drive.m_freeSpin); w.flag(a.m_drive.m_outputForce);
	w.scalar(a.m_friction.m_staticEffort); w.scalar(a.m_friction.m_dynamicEffort); w.scalar(a.m_friction.m_viscousCoefficient); w.scalar(a.m_friction.m_legacyCoefficient); w.flag(a.m_friction.m_useLegacy);
	w.scalar(a.m_envelope.m_maxEffort); w.scalar(a.m_envelope.m_velocityDependentResistance); w.scalar(a.m_envelope.m_maxActuatorVelocity); w.scalar(a.m_envelope.m_speedEffortGradient);
	w.scalar(a.m_armature); w.scalar(a.m_maxVelocity);
}
void readAxis(BinaryReader& r, btConstraintAxisParameters& a)
{
	a.m_motion = btConstraintMotion(r.enumeration(BT_CONSTRAINT_FREE));
	a.m_limit.m_lower = r.scalar(); a.m_limit.m_upper = r.scalar(); a.m_limit.m_restitution = r.scalar(); a.m_limit.m_bounceThreshold = r.scalar(); a.m_limit.m_stiffness = r.scalar(); a.m_limit.m_damping = r.scalar();
	a.m_drive.m_position = r.scalar(); a.m_drive.m_velocity = r.scalar(); a.m_drive.m_stiffness = r.scalar(); a.m_drive.m_damping = r.scalar(); a.m_drive.m_maxEffort = r.scalar();
	a.m_drive.m_mode = btConstraintSpringMode(r.enumeration(BT_CONSTRAINT_ACCELERATION_SPRING)); a.m_drive.m_effortUnit = btConstraintBoundUnit(r.enumeration(BT_CONSTRAINT_FORCE));
	a.m_drive.m_enabled = r.flag(); a.m_drive.m_freeSpin = r.flag(); a.m_drive.m_outputForce = r.flag();
	a.m_friction.m_staticEffort = r.scalar(); a.m_friction.m_dynamicEffort = r.scalar(); a.m_friction.m_viscousCoefficient = r.scalar(); a.m_friction.m_legacyCoefficient = r.scalar(); a.m_friction.m_useLegacy = r.flag();
	a.m_envelope.m_maxEffort = r.scalar(); a.m_envelope.m_velocityDependentResistance = r.scalar(); a.m_envelope.m_maxActuatorVelocity = r.scalar(); a.m_envelope.m_speedEffortGradient = r.scalar();
	a.m_armature = r.scalar(); a.m_maxVelocity = r.scalar();
}
}

bool btConstraintScalarFinite(btScalar x) { return std::isfinite(static_cast<double>(x)); }
bool btConstraintJacobianEqual(const btConstraintJacobian& a, const btConstraintJacobian& b)
{
	if (a.m_linear != b.m_linear || a.m_angular != b.m_angular || a.m_generalized.size() != b.m_generalized.size()) return false;
	for (int i = 0; i < a.m_generalized.size(); ++i) if (a.m_generalized[i] != b.m_generalized[i]) return false;
	return true;
}
btConstraintStatus btValidateConstraintRow(const btConstraintRowDescriptor& row)
{
	if (!row.m_ownerId || !row.m_rowId || !row.m_terms.size()) return invalid("row owner, row ID and terms are required");
	if (!btConstraintScalarFinite(row.m_geometricError) || !btConstraintScalarFinite(row.m_targetVelocity) ||
		!btConstraintScalarFinite(row.m_lowerBound) || !btConstraintScalarFinite(row.m_upperBound) || row.m_lowerBound > row.m_upperBound ||
		row.m_boundUnit < BT_CONSTRAINT_IMPULSE || row.m_boundUnit > BT_CONSTRAINT_FORCE ||
		row.m_springMode < BT_CONSTRAINT_NO_SPRING || row.m_springMode > BT_CONSTRAINT_ACCELERATION_SPRING ||
		row.m_biasMode < BT_CONSTRAINT_NO_BIAS || row.m_biasMode > BT_CONSTRAINT_KEEP_BIAS ||
		!nonnegative(row.m_erp) || row.m_erp > 1 || !nonnegative(row.m_stiffness) || !nonnegative(row.m_damping) ||
		!nonnegative(row.m_restitution) || row.m_restitution > 1 || !nonnegative(row.m_bounceThreshold) || !btConstraintScalarFinite(row.m_state.m_accumulatedImpulse))
		return invalid("invalid scalar row parameter");
	if (row.m_springMode != BT_CONSTRAINT_NO_SPRING && row.m_restitution != 0)
		return invalid("spring and restitution modes are mutually exclusive");
	for (int i = 0; i < row.m_terms.size(); ++i)
	{
		const btConstraintJacobianTerm& t = row.m_terms[i];
		if (!objectValid(t.m_object) || !jacobianFinite(t.m_measurement) || !jacobianFinite(t.m_response) ||
			!nonnegative(t.m_inverseMassScale) || !nonnegative(t.m_inverseInertiaScale) ||
			t.m_measurement.m_generalized.size() != t.m_response.m_generalized.size()) return invalid("invalid Jacobian term", i);
	}
	return btConstraintStatus();
}
btConstraintStatus btValidateNativeHardConstraintRow(const btConstraintRowDescriptor& row)
{
	btConstraintStatus s = btValidateConstraintRow(row); if (!s.isOk()) return s;
	if (row.m_springMode != BT_CONSTRAINT_NO_SPRING || row.m_stiffness != 0 || row.m_damping != 0 ||
		row.m_restitution != 0 || row.m_bounceThreshold != 0 || row.m_biasMode == BT_CONSTRAINT_KEEP_BIAS || row.m_dynamicBounds || row.m_state.m_accumulatedImpulse != 0)
		return btConstraintStatus(BT_CONSTRAINT_UNSUPPORTED, "native adapter supports hard rows, fixed bounds, no imported warmstart only");
	for (int i = 0; i < row.m_terms.size(); ++i)
		if (!btConstraintJacobianEqual(row.m_terms[i].m_measurement, row.m_terms[i].m_response) || row.m_terms[i].m_inverseMassScale != 1 || row.m_terms[i].m_inverseInertiaScale != 1)
			return btConstraintStatus(BT_CONSTRAINT_UNSUPPORTED, "separate response Jacobian and local mass scaling require step 4", i);
	return btConstraintStatus();
}
btConstraintStatus btConstraintBoundsToImpulse(btScalar lower, btScalar upper, btConstraintBoundUnit unit, btScalar dt, btScalar& lo, btScalar& hi)
{
	if (!btConstraintScalarFinite(dt) || dt <= 0 || !btConstraintScalarFinite(lower) || !btConstraintScalarFinite(upper) || lower > upper ||
		(unit != BT_CONSTRAINT_IMPULSE && unit != BT_CONSTRAINT_FORCE)) return invalid("invalid bounds, unit or time step");
	btScalar l = lower, h = upper;
	if (unit == BT_CONSTRAINT_FORCE) { l *= dt; h *= dt; }
	if (!btConstraintScalarFinite(l) || !btConstraintScalarFinite(h)) return invalid("force to impulse overflow");
	lo = l; hi = h; return btConstraintStatus();
}
btScalar btConstraintTargetVelocity(const btConstraintRowDescriptor& row, btScalar dt)
{
	btAssert(dt > 0);
	return row.m_targetVelocity - (row.m_biasMode == BT_CONSTRAINT_ERP_BIAS ? row.m_erp * row.m_geometricError / dt : btScalar(0));
}
btTransform btConstraintFrameToWorld(const btConstraintLocalFrame& local, const btConstraintCoordinateState& object)
{
	if (local.m_space == BT_CONSTRAINT_WORLD_FRAME) return local.m_transform;
	return (local.m_space == BT_CONSTRAINT_ACTOR_FRAME ? object.m_actorWorld : object.m_comWorld) * local.m_transform;
}
btConstraintStatus btValidateConstraintFrameSnapshot(const btConstraintFrameSnapshot& frame)
{
	if (!btConstraintScalarFinite(frame.m_timeStep) || frame.m_timeStep <= 0 || frame.m_objects.size() != frame.m_states.size())
		return invalid("invalid frame time step or state count");
	for (int i = 0; i < frame.m_objects.size(); ++i)
	{
		if (!objectValid(frame.m_objects[i])) return invalid("invalid frame object", i);
		for (int j = 0; j < i; ++j)
			if (frame.m_objects[i] == frame.m_objects[j] || (frame.m_objects[i].m_id == frame.m_objects[j].m_id && frame.m_objects[i].m_domain != frame.m_objects[j].m_domain))
				return invalid("duplicate or conflicting frame identity", i);
		const btConstraintCoordinateState& s = frame.m_states[i];
		btConstraintLocalFrame f; f.m_transform = s.m_comWorld;
		if (!frameValid(f)) return invalid("invalid COM frame", i);
		f.m_transform = s.m_actorWorld;
		if (!frameValid(f) || !vectorFinite(s.m_linearVelocity) || !vectorFinite(s.m_angularVelocity)) return invalid("invalid actor frame or velocity", i);
		for (int j = 0; j < s.m_position.size(); ++j) if (!btConstraintScalarFinite(s.m_position[j])) return invalid("invalid coordinate position", i);
		for (int j = 0; j < s.m_velocity.size(); ++j) if (!btConstraintScalarFinite(s.m_velocity[j])) return invalid("invalid coordinate velocity", i);
	}
	return btConstraintStatus();
}
btConstraintStatus btConstraintFrameSnapshot::capture(uint64_t step, btScalar dt, const btAlignedObjectArray<btConstraintObjectRef>& objects, const btConstraintCoordinateProvider& provider)
{
	if (!btConstraintScalarFinite(dt) || dt <= 0) return invalid("snapshot time step must be positive");
	btConstraintFrameSnapshot next; next.m_step = step; next.m_timeStep = dt;
	for (int i = 0; i < objects.size(); ++i)
	{
		if (!objectValid(objects[i])) return invalid("invalid snapshot object", i);
		for (int j = 0; j < i; ++j) if (objects[i] == objects[j]) return btConstraintStatus(BT_CONSTRAINT_DUPLICATE_ID, "duplicate snapshot object", i);
		btConstraintCoordinateState state;
		if (objects[i].m_domain != BT_CONSTRAINT_WORLD) { btConstraintStatus s = provider.read(objects[i], state); if (!s.isOk()) return s; }
		btConstraintLocalFrame f; f.m_transform = state.m_comWorld; if (!frameValid(f)) return invalid("invalid COM transform", i);
		f.m_transform = state.m_actorWorld; if (!frameValid(f) || !vectorFinite(state.m_linearVelocity) || !vectorFinite(state.m_angularVelocity)) return invalid("invalid coordinate state", i);
		for (int j = 0; j < state.m_position.size(); ++j) if (!btConstraintScalarFinite(state.m_position[j])) return invalid("invalid coordinate position", i);
		for (int j = 0; j < state.m_velocity.size(); ++j) if (!btConstraintScalarFinite(state.m_velocity[j])) return invalid("invalid coordinate velocity", i);
		next.m_objects.push_back(objects[i]); next.m_states.push_back(state);
	}
	btConstraintStatus status = btValidateConstraintFrameSnapshot(next);
	if (status.isOk()) *this = next;
	return status;
}
btConstraintStatus btConstraintAngleTracker::prepare(uint64_t step, btScalar wrappedAngle)
{
	if (!btConstraintScalarFinite(wrappedAngle) || wrappedAngle < -SIMD_PI || wrappedAngle > SIMD_PI) return invalid("angle must be in [-pi, pi]");
	if (m_initialized && step < m_step) return btConstraintStatus(BT_CONSTRAINT_STALE_FRAME, "angle frame precedes committed frame");
	if (m_initialized && step == m_step)
		return wrappedAngle == m_wrapped ? btConstraintStatus() : btConstraintStatus(BT_CONSTRAINT_STALE_FRAME, "same angle frame has changed input");
	if (!m_initialized) m_continuous = wrappedAngle;
	else { btScalar delta = wrappedAngle - m_wrapped; if (delta > SIMD_PI) delta -= SIMD_2_PI; if (delta < -SIMD_PI) delta += SIMD_2_PI; m_continuous += delta; }
	m_wrapped = wrappedAngle; m_step = step; m_initialized = true; return btConstraintStatus();
}
btConstraintStatus btConstraintDescriptionImporter::validate(const btConstraintDescriptionGraph& graph)
{
	if (graph.m_schemaVersion != 1) return btConstraintStatus(BT_CONSTRAINT_INVALID_FORMAT, "unsupported description schema");
	if (graph.m_objects.size() > kMaxRecords || graph.m_constraints.size() > kMaxRecords) return invalid("description record limit exceeded");
	// Bound both parsing work and the portable byte-array size before allocating it.
	uint64_t totalBytes = 32 + uint64_t(graph.m_objects.size()) * 24;
	for (int i = 0; i < graph.m_objects.size(); ++i)
	{
		if (!objectValid(graph.m_objects[i])) return invalid("invalid object reference", i);
		for (int j = 0; j < i; ++j)
		{
			if (graph.m_objects[i] == graph.m_objects[j]) return btConstraintStatus(BT_CONSTRAINT_DUPLICATE_ID, "duplicate object reference", i);
			if (graph.m_objects[i].m_id == graph.m_objects[j].m_id && graph.m_objects[i].m_domain != graph.m_objects[j].m_domain) return invalid("one object ID has different domains", i);
		}
	}
	for (int i = 0; i < graph.m_constraints.size(); ++i)
	{
		const btConstraintDescription& d = graph.m_constraints[i];
		totalBytes += 8 * (2 + 1 + 1 + 26 + 10 + 6 * 28 + 2) + uint64_t(d.m_objects.size()) * 24 + uint64_t(d.m_constraintReferences.size()) * 8 + uint64_t(d.m_payload.size());
		if (totalBytes > kMaxGraphBytes) return invalid("description byte budget exceeded", i);
		if (!d.m_id || !categoryValid(d.m_category) || !frameValid(d.m_frameA) || !frameValid(d.m_frameB) || !commonValid(d.m_common) || d.m_objects.size() == 0 ||
			d.m_objects.size() > kMaxRecords || d.m_constraintReferences.size() > kMaxRecords || d.m_payload.size() > kMaxPayload || (d.m_payload.size() && !d.m_payloadVersion)) return invalid("invalid constraint description", i);
		for (int j = 0; j < i; ++j) if (d.m_id == graph.m_constraints[j].m_id) return btConstraintStatus(BT_CONSTRAINT_DUPLICATE_ID, "duplicate constraint ID", i);
		for (int j = 0; j < 6; ++j) if (!axisValid(d.m_axes[j])) return invalid("invalid common axis parameters", i);
		for (int j = 0; j < d.m_objects.size(); ++j)
		{
			bool found = false; for (int k = 0; k < graph.m_objects.size(); ++k) if (d.m_objects[j] == graph.m_objects[k]) found = true;
			if (!found) return btConstraintStatus(BT_CONSTRAINT_MISSING_REFERENCE, "unresolved object reference", i);
		}
		for (int j = 0; j < d.m_constraintReferences.size(); ++j)
		{
			bool found = false; for (int k = 0; k < graph.m_constraints.size(); ++k) if (d.m_constraintReferences[j] == graph.m_constraints[k].m_id) found = true;
			if (!found) return btConstraintStatus(BT_CONSTRAINT_MISSING_REFERENCE, "unresolved constraint reference", i);
		}
	}
	return btConstraintStatus();
}
btConstraintStatus btConstraintDescriptionImporter::load(const btConstraintDescriptionGraph& source, btConstraintDescriptionGraph& destination)
{
	btConstraintStatus result = validate(source); if (result.isOk() && &source != &destination) destination = source; return result;
}
btConstraintStatus btConstraintDescriptionImporter::writeBinary(const btConstraintDescriptionGraph& source, btAlignedObjectArray<unsigned char>& bytes)
{
	static_assert(sizeof(double) == 8 && std::numeric_limits<double>::is_iec559, "description format requires IEEE binary64");
	btConstraintStatus s = validate(source); if (!s.isOk()) return s;
	btAlignedObjectArray<unsigned char> next; BinaryWriter w(next);
	w.u64(UINT64_C(0x3130524758435442)); // byte order: BTCXGR01
	w.u64(1); w.u64(source.m_objects.size()); w.u64(source.m_constraints.size());
	for (int i = 0; i < source.m_objects.size(); ++i) writeRef(w, source.m_objects[i]);
	for (int i = 0; i < source.m_constraints.size(); ++i)
	{
		const btConstraintDescription& d = source.m_constraints[i]; w.u64(d.m_id); w.u64(d.m_category);
		w.u64(d.m_objects.size()); for (int j = 0; j < d.m_objects.size(); ++j) writeRef(w, d.m_objects[j]);
		w.u64(d.m_constraintReferences.size()); for (int j = 0; j < d.m_constraintReferences.size(); ++j) w.u64(d.m_constraintReferences[j]);
		writeFrame(w, d.m_frameA); writeFrame(w, d.m_frameB); writeCommon(w, d.m_common); for (int j = 0; j < 6; ++j) writeAxis(w, d.m_axes[j]);
		w.u64(d.m_payloadVersion); w.u64(d.m_payload.size()); for (int j = 0; j < d.m_payload.size(); ++j) next.push_back(d.m_payload[j]);
	}
	bytes = next; return btConstraintStatus();
}
btConstraintStatus btConstraintDescriptionImporter::readBinary(const unsigned char* bytes, int size, btConstraintDescriptionGraph& destination)
{
	if (size < 0 || uint64_t(size) > kMaxGraphBytes) return btConstraintStatus(BT_CONSTRAINT_INVALID_FORMAT, "description byte budget exceeded");
	BinaryReader r(bytes, size); btConstraintDescriptionGraph next;
	if (r.u64() != UINT64_C(0x3130524758435442) || r.u64() != 1) return btConstraintStatus(BT_CONSTRAINT_INVALID_FORMAT, "bad graph magic or version");
	int objects = r.count(), constraints = r.count();
	for (int i = 0; i < objects && r.m_ok; ++i) next.m_objects.push_back(readRef(r));
	for (int i = 0; i < constraints && r.m_ok; ++i)
	{
		btConstraintDescription d; d.m_id = r.u64(); d.m_category = btConstraintCategoryId(r.enumeration());
		int refs = r.count(); for (int j = 0; j < refs && r.m_ok; ++j) d.m_objects.push_back(readRef(r));
		refs = r.count(); for (int j = 0; j < refs && r.m_ok; ++j) d.m_constraintReferences.push_back(r.u64());
		readFrame(r, d.m_frameA); readFrame(r, d.m_frameB); readCommon(r, d.m_common); for (int j = 0; j < 6; ++j) readAxis(r, d.m_axes[j]);
		d.m_payloadVersion = r.enumeration(0xffffffffu); int payload = r.count(kMaxPayload);
		if (r.m_size - r.m_offset < payload) r.m_ok = false;
		for (int j = 0; j < payload && r.m_ok; ++j) d.m_payload.push_back(r.m_bytes[r.m_offset++]);
		next.m_constraints.push_back(d);
	}
	if (!r.m_ok || r.m_offset != size) return btConstraintStatus(BT_CONSTRAINT_INVALID_FORMAT, "truncated, oversized, nonfinite or trailing graph data");
	return load(next, destination);
}
