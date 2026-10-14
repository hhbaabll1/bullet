/* New Bullet constraint extension interfaces. Distributed under the Bullet zlib license. */
#include "btMultiBodyRowDescriptorConstraint.h"
#include "btMultiBodyLinkCollider.h"
#include "BulletDynamics/ConstraintSolver/btContactSolverInfo.h"

namespace
{
bool validLink(btMultiBody* body, int link) { return body && link >= -1 && link < body->getNumLinks(); }
int island(btMultiBody* body, int link)
{
	if (!validLink(body, link)) return -1;
	btMultiBodyLinkCollider* c = link < 0 ? body->getBaseCollider() : body->getLink(link).m_collider;
	return c ? c->getIslandTag() : -1;
}
bool spatialZero(const btConstraintJacobian& j) { return j.m_linear == btVector3(0, 0, 0) && j.m_angular == btVector3(0, 0, 0); }
}
btMultiBodyRowDescriptorConstraint::btMultiBodyRowDescriptorConstraint(btMultiBody* a, int linkA, uint64_t idA, btMultiBody* b, int linkB, uint64_t idB)
	: btMultiBodyConstraint(a, b, linkA, linkB, 0, false, MULTIBODY_CONSTRAINT_ROW_DESCRIPTOR),
	  m_objectA(idA, BT_CONSTRAINT_MULTIBODY_LINK, linkA), m_objectB(b ? btConstraintObjectRef(idB, BT_CONSTRAINT_MULTIBODY_LINK, linkB) : btConstraintObjectRef()),
	  m_step(0), m_timeStep(0), m_prepared(false), m_hasSnapshot(false), m_status(BT_CONSTRAINT_STALE_FRAME, "prepare is required") { m_maxAppliedImpulse = SIMD_INFINITY; }
void btMultiBodyRowDescriptorConstraint::invalidate()
{
	m_prepared = false; m_numRows = 0; m_status = btConstraintStatus(BT_CONSTRAINT_STALE_FRAME, "prepare is required");
}
btConstraintStatus btMultiBodyRowDescriptorConstraint::prepare(const btConstraintFrameSnapshot& frame, const btAlignedObjectArray<btConstraintRowDescriptor>& rows)
{
	invalidate();
	m_status = btValidateConstraintFrameSnapshot(frame); if (!m_status.isOk()) return m_status;
	if (m_hasSnapshot && frame.m_step <= m_step)
		return m_status = btConstraintStatus(BT_CONSTRAINT_STALE_FRAME, "a newer positive-dt frame is required");
	if (!validLink(m_bodyA, m_linkA) || !m_objectA.m_id || (m_bodyB && (!validLink(m_bodyB, m_linkB) || !m_objectB.m_id)))
		return m_status = btConstraintStatus(BT_CONSTRAINT_INVALID_PARAMETER, "invalid MultiBody identities or links");
	if (m_bodyA == m_bodyB || (m_bodyB && m_objectA.m_id == m_objectB.m_id))
		return m_status = btConstraintStatus(BT_CONSTRAINT_UNSUPPORTED, "same-tree Jacobian merging belongs to step 4");
	bool foundA = false, foundB = false;
	for (int i = 0; i < frame.m_objects.size(); ++i) { foundA = foundA || frame.m_objects[i] == m_objectA; foundB = foundB || frame.m_objects[i] == m_objectB; }
	if (!foundA || !foundB || frame.m_objects.size() != frame.m_states.size()) return m_status = btConstraintStatus(BT_CONSTRAINT_MISSING_REFERENCE, "snapshot does not contain both endpoints");
	for (int i = 0; i < rows.size(); ++i)
	{
		const btConstraintRowDescriptor& row = rows[i]; m_status = btValidateNativeHardConstraintRow(row); if (!m_status.isOk()) return m_status;
		if (row.m_terms.size() != 2 || !(row.m_terms[0].m_object == m_objectA) || !(row.m_terms[1].m_object == m_objectB)) return m_status = btConstraintStatus(BT_CONSTRAINT_UNSUPPORTED, "MultiBody adapter requires ordered A/B terms", i);
		if (row.m_outputForce) return m_status = btConstraintStatus(BT_CONSTRAINT_UNSUPPORTED, "generalized rows do not yet provide spatial feedback reconstruction", i);
		if (row.m_ownerId != rows[0].m_ownerId) return m_status = btConstraintStatus(BT_CONSTRAINT_UNSUPPORTED, "one owner required for a native row set", i);
		for (int j = 0; j < i; ++j) if (row.m_rowId == rows[j].m_rowId) return m_status = btConstraintStatus(BT_CONSTRAINT_DUPLICATE_ID, "duplicate row ID", i);
		if (!spatialZero(row.m_terms[0].m_measurement) || !spatialZero(row.m_terms[1].m_measurement) ||
			row.m_terms[0].m_measurement.m_generalized.size() != 6 + m_bodyA->getNumDofs() ||
			row.m_terms[1].m_measurement.m_generalized.size() != (m_bodyB ? 6 + m_bodyB->getNumDofs() : 0))
			return m_status = btConstraintStatus(BT_CONSTRAINT_UNSUPPORTED, "expected generalized Jacobians with zero spatial fields", i);
		btScalar lo, hi; m_status = btConstraintBoundsToImpulse(row.m_lowerBound, row.m_upperBound, row.m_boundUnit, frame.m_timeStep, lo, hi); if (!m_status.isOk()) return m_status;
		if (!btConstraintScalarFinite(btConstraintTargetVelocity(row, frame.m_timeStep))) return m_status = btConstraintStatus(BT_CONSTRAINT_INVALID_PARAMETER, "row target overflow", i);
	}
	m_rows = rows; m_numRows = rows.size(); m_step = frame.m_step; m_timeStep = frame.m_timeStep; finalizeMultiDof(); m_prepared = true; m_hasSnapshot = true;
	return m_status = btConstraintStatus();
}
void btMultiBodyRowDescriptorConstraint::finalizeMultiDof()
{
	allocateJacobiansMultiDof();
	// No imported warmstart is claimed. Clear caches explicitly when a new snapshot is prepared.
	for (int i = 0; i < m_data.size(); ++i) m_data[i] = 0;
	for (int i = 0; i < m_rows.size(); ++i)
	{
		for (int j = 0; j < m_rows[i].m_terms[0].m_measurement.m_generalized.size(); ++j) jacobianA(i)[j] = m_rows[i].m_terms[0].m_measurement.m_generalized[j];
		if (m_bodyB) for (int j = 0; j < m_rows[i].m_terms[1].m_measurement.m_generalized.size(); ++j) jacobianB(i)[j] = m_rows[i].m_terms[1].m_measurement.m_generalized[j];
	}
	m_numDofsFinalized = m_jacSizeBoth;
}
int btMultiBodyRowDescriptorConstraint::getIslandIdA() const { return island(m_bodyA, m_linkA); }
int btMultiBodyRowDescriptorConstraint::getIslandIdB() const { return island(m_bodyB, m_linkB); }
void btMultiBodyRowDescriptorConstraint::createConstraintRows(btMultiBodyConstraintArray& rows, btMultiBodyJacobianData& data, const btContactSolverInfo& info)
{
	if (!m_prepared) { m_status = btConstraintStatus(BT_CONSTRAINT_STALE_FRAME, "no prepared rows"); return; }
	if (!btConstraintScalarFinite(info.m_timeStep) || info.m_timeStep <= 0 || btFabs(info.m_timeStep / m_timeStep - 1) > btScalar(1e-4) ||
		m_jacSizeA != 6 + m_bodyA->getNumDofs() || m_jacSizeBoth != m_jacSizeA + (m_bodyB ? 6 + m_bodyB->getNumDofs() : 0))
	{
		m_status = btConstraintStatus(BT_CONSTRAINT_STALE_FRAME, "solver dt or MultiBody topology changed after prepare"); return;
	}
	if (!btConstraintScalarFinite(m_maxAppliedImpulse) || m_maxAppliedImpulse < 0 ||
		(!m_bodyB && (!data.m_solverBodyPool || data.m_fixedBodyId < 0 || data.m_fixedBodyId >= data.m_solverBodyPool->size())))
	{
		m_status = btConstraintStatus(BT_CONSTRAINT_INVALID_PARAMETER, "invalid impulse cap or missing fixed solver body"); return;
	}
	const btVector3 zero(0, 0, 0);
	for (int i = 0; i < m_rows.size(); ++i)
	{
		const btConstraintRowDescriptor& row = m_rows[i]; btScalar lo, hi;
		btConstraintBoundsToImpulse(row.m_lowerBound, row.m_upperBound, row.m_boundUnit, m_timeStep, lo, hi);
		lo = btMax(lo, -m_maxAppliedImpulse); hi = btMin(hi, m_maxAppliedImpulse);
		if (lo > hi) { m_status = btConstraintStatus(BT_CONSTRAINT_INVALID_PARAMETER, "native cap has empty intersection with bounds", i); return; }
	}
	for (int i = 0; i < m_rows.size(); ++i)
	{
		const btConstraintRowDescriptor& row = m_rows[i]; btScalar lo, hi;
		btConstraintBoundsToImpulse(row.m_lowerBound, row.m_upperBound, row.m_boundUnit, m_timeStep, lo, hi);
		btMultiBodySolverConstraint& out = rows.expandNonInitializing();
		out.m_solverBodyIdA = data.m_fixedBodyId; out.m_solverBodyIdB = data.m_fixedBodyId;
		// Bias is included exactly once in desiredVelocity; pass zero position error.
		fillMultiBodyConstraint(out, data, jacobianA(i), m_bodyB ? jacobianB(i) : 0, zero, zero, zero, zero, 0, info,
			btMax(lo, -m_maxAppliedImpulse), btMin(hi, m_maxAppliedImpulse), row.m_angular, 1, false, btConstraintTargetVelocity(row, m_timeStep));
		out.m_orgConstraint = this; out.m_orgDofIndex = i;
	}
}
