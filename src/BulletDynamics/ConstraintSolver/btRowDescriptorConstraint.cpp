/* New Bullet constraint extension interfaces. Distributed under the Bullet zlib license. */
#include "btRowDescriptorConstraint.h"
#include <limits>

btRowDescriptorConstraint::btRowDescriptorConstraint(btRigidBody& a, uint64_t idA, btRigidBody& b, uint64_t idB)
	: btTypedConstraint(ROW_DESCRIPTOR_CONSTRAINT_TYPE, a, b), m_objectA(idA, BT_CONSTRAINT_RIGID_BODY), m_objectB(idB, BT_CONSTRAINT_RIGID_BODY),
	  m_step(0), m_timeStep(0), m_prepared(false), m_hasSnapshot(false), m_status(BT_CONSTRAINT_STALE_FRAME, "prepare is required") { setEnabled(false); }
btRowDescriptorConstraint::btRowDescriptorConstraint(btRigidBody& a, uint64_t idA)
	: btTypedConstraint(ROW_DESCRIPTOR_CONSTRAINT_TYPE, a), m_objectA(idA, BT_CONSTRAINT_RIGID_BODY), m_objectB(),
	  m_step(0), m_timeStep(0), m_prepared(false), m_hasSnapshot(false), m_status(BT_CONSTRAINT_STALE_FRAME, "prepare is required") { setEnabled(false); }
void btRowDescriptorConstraint::invalidate()
{
	m_prepared = false; setEnabled(false); m_status = btConstraintStatus(BT_CONSTRAINT_STALE_FRAME, "prepare is required");
}
btConstraintStatus btRowDescriptorConstraint::prepare(const btConstraintFrameSnapshot& frame, const btAlignedObjectArray<btConstraintRowDescriptor>& rows)
{
	invalidate();
	m_status = btValidateConstraintFrameSnapshot(frame); if (!m_status.isOk()) return m_status;
	if (m_hasSnapshot && frame.m_step <= m_step)
		return m_status = btConstraintStatus(BT_CONSTRAINT_STALE_FRAME, "a newer positive-dt frame is required");
	if (m_objectA.m_id == 0 || (m_objectB.m_domain != BT_CONSTRAINT_WORLD && (m_objectB.m_id == 0 || &m_rbA == &m_rbB || m_objectA.m_id == m_objectB.m_id)))
		return m_status = btConstraintStatus(BT_CONSTRAINT_INVALID_PARAMETER, "invalid rigid-body identities");
	bool foundA = false, foundB = false;
	for (int i = 0; i < frame.m_objects.size(); ++i) { foundA = foundA || frame.m_objects[i] == m_objectA; foundB = foundB || frame.m_objects[i] == m_objectB; }
	if (!foundA || !foundB || frame.m_objects.size() != frame.m_states.size())
		return m_status = btConstraintStatus(BT_CONSTRAINT_MISSING_REFERENCE, "snapshot does not contain both endpoints");
	for (int i = 0; i < rows.size(); ++i)
	{
		const btConstraintRowDescriptor& row = rows[i]; m_status = btValidateNativeHardConstraintRow(row); if (!m_status.isOk()) return m_status;
		if (row.m_terms.size() != 2 || !(row.m_terms[0].m_object == m_objectA) || !(row.m_terms[1].m_object == m_objectB))
			return m_status = btConstraintStatus(BT_CONSTRAINT_UNSUPPORTED, "rigid adapter requires ordered A/B terms", i);
		if (row.m_terms[0].m_measurement.m_generalized.size() || row.m_terms[1].m_measurement.m_generalized.size())
			return m_status = btConstraintStatus(BT_CONSTRAINT_UNSUPPORTED, "rigid adapter requires spatial Jacobians", i);
		if (row.m_outputForce != rows[0].m_outputForce || row.m_ownerId != rows[0].m_ownerId)
			return m_status = btConstraintStatus(BT_CONSTRAINT_UNSUPPORTED, "rigid feedback and owner apply to the whole constraint", i);
		for (int j = 0; j < i; ++j) if (row.m_rowId == rows[j].m_rowId) return m_status = btConstraintStatus(BT_CONSTRAINT_DUPLICATE_ID, "duplicate row ID", i);
		btScalar lo, hi; m_status = btConstraintBoundsToImpulse(row.m_lowerBound, row.m_upperBound, row.m_boundUnit, frame.m_timeStep, lo, hi); if (!m_status.isOk()) return m_status;
		if (!btConstraintScalarFinite(btConstraintTargetVelocity(row, frame.m_timeStep))) return m_status = btConstraintStatus(BT_CONSTRAINT_INVALID_PARAMETER, "row target overflow", i);
	}
	m_rows = rows; m_step = frame.m_step; m_timeStep = frame.m_timeStep; m_prepared = true; m_hasSnapshot = true;
	enableFeedback(rows.size() ? rows[0].m_outputForce : false); setEnabled(rows.size() != 0);
	return m_status = btConstraintStatus();
}
void btRowDescriptorConstraint::getInfo1(btConstraintInfo1* info)
{
	info->m_numConstraintRows = m_prepared && isEnabled() ? m_rows.size() : 0; info->nub = 0;
}
void btRowDescriptorConstraint::getInfo2(btConstraintInfo2* info)
{
	if (m_prepared && !m_rows.size()) return; // A valid inactive constraint snapshot.
	if (!m_prepared || !isEnabled()) { m_status = btConstraintStatus(BT_CONSTRAINT_STALE_FRAME, "no prepared rows"); return; }
	if (!btConstraintScalarFinite(info->fps) || btFabs(info->fps * m_timeStep - 1) > btScalar(1e-4))
	{
		// getInfo2 cannot change the row count. Zero the rows and expose the failure.
		for (int i = 0; i < m_rows.size(); ++i) { int at = i * info->rowskip; info->m_lowerLimit[at] = 0; info->m_upperLimit[at] = 0; info->m_constraintError[at] = 0; }
		m_status = btConstraintStatus(BT_CONSTRAINT_STALE_FRAME, "solver dt differs from prepared dt"); return;
	}
	for (int i = 0; i < m_rows.size(); ++i)
	{
		const btConstraintRowDescriptor& row = m_rows[i]; int at = i * info->rowskip;
		const btConstraintJacobian& a = row.m_terms[0].m_measurement; const btConstraintJacobian& b = row.m_terms[1].m_measurement;
		for (int j = 0; j < 3; ++j)
		{
			info->m_J1linearAxis[at+j] = a.m_linear[j]; info->m_J1angularAxis[at+j] = a.m_angular[j];
			if (info->m_J2linearAxis) info->m_J2linearAxis[at+j] = b.m_linear[j]; if (info->m_J2angularAxis) info->m_J2angularAxis[at+j] = b.m_angular[j];
		}
		info->m_constraintError[at] = btConstraintTargetVelocity(row, m_timeStep); info->cfm[at] = 0;
		btConstraintBoundsToImpulse(row.m_lowerBound, row.m_upperBound, row.m_boundUnit, m_timeStep, info->m_lowerLimit[at], info->m_upperLimit[at]);
	}
}
void btRowDescriptorConstraint::setParam(int, btScalar, int)
{
	invalidate(); m_status = btConstraintStatus(BT_CONSTRAINT_UNSUPPORTED, "set descriptor parameters and prepare instead of setParam");
}
btScalar btRowDescriptorConstraint::getParam(int, int) const { return std::numeric_limits<btScalar>::quiet_NaN(); }
