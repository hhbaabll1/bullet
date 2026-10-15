/* New Bullet extension, 2026. Distributed under the Bullet zlib license. */
#include "btMultiBodyDistanceConstraint.h"
#include "btMultiBodyLinkCollider.h"
#include "BulletDynamics/ConstraintSolver/btContactSolverInfo.h"
#include "BulletDynamics/Dynamics/btRigidBody.h"

namespace
{
int island(const btDistanceConstraintEndpoint& e)
{ btCollisionObject* c=e.collider(); return c?c->getIslandTag():-1; }
btVector3 anchorTorque(const btDistanceConstraintEndpoint& e,const btVector3& p,const btVector3& n)
{ return (p-e.comWorld().getOrigin()).cross(n); }
void fillJacobian(btMultiBody* body,int link,const btVector3& point,const btVector3& linear,
	btScalar linearScale,btScalar angularScale,btScalar* measurement,btScalar* response,btMultiBodyJacobianData& data)
{
	const int size=6+body->getNumDofs();
	body->fillConstraintJacobianMultiDof(link,point,btVector3(0,0,0),linear,measurement,data.scratch_r,data.scratch_v,data.scratch_m);
	btAlignedObjectArray<btScalar> angular; angular.resize(size);
	body->fillConstraintJacobianMultiDof(link,point,(point-(link<0?body->getBasePos():body->getLink(link).m_cachedWorldTransform.getOrigin())).cross(linear),
		btVector3(0,0,0),&angular[0],data.scratch_r,data.scratch_v,data.scratch_m);
	for(int i=0;i<size;++i) response[i]=linearScale*measurement[i]+(angularScale-linearScale)*angular[i];
}
}
btMultiBodyDistanceConstraint::btMultiBodyDistanceConstraint(const btDistanceConstraintEndpoint& a,const btDistanceConstraintEndpoint& b,
	const btVector3& pa,const btVector3& pb)
	: btMultiBodyConstraint(a.m_multiBody?a.m_multiBody:b.m_multiBody,
		a.m_multiBody?b.m_multiBody:a.m_multiBody,
		a.m_multiBody?a.m_link:b.m_link,
		(a.m_multiBody?b:a).m_multiBody?(a.m_multiBody?b:a).m_link:-1,
		0,false,MULTIBODY_CONSTRAINT_DISTANCE),
	  btDistanceConstraintState(a.m_multiBody?a:b,a.m_multiBody?b:a,a.m_multiBody?pa:pb,a.m_multiBody?pb:pa)
{ m_maxAppliedImpulse=SIMD_INFINITY; }
void btMultiBodyDistanceConstraint::finalizeMultiDof()
{ allocateJacobiansMultiDof();m_numDofsFinalized=m_jacSizeBoth; }
int btMultiBodyDistanceConstraint::getIslandIdA() const {return island(m_endpoints[0]);}
int btMultiBodyDistanceConstraint::getIslandIdB() const {return island(m_endpoints[1]);}
void btMultiBodyDistanceConstraint::createConstraintRows(btMultiBodyConstraintArray& output,btMultiBodyJacobianData& data,const btContactSolverInfo& info)
{
	const int count=prepareDistance();m_numRows=count;
	if(!count)return;
	if(!btConstraintScalarFinite(info.m_timeStep)||info.m_timeStep<=0 || !m_bodyA)
	{m_status=btConstraintStatus(BT_CONSTRAINT_INVALID_PARAMETER,"invalid Distance solver step or endpoints");m_numRows=0;return;}
	finalizeMultiDof();
	for(int j=0;j<count;++j)m_data[j]=0; // distance rows are rebuilt from the current geometry
	const int na=6+m_bodyA->getNumDofs(),nb=m_bodyB?6+m_bodyB->getNumDofs():0;
	const bool same=m_bodyB&&m_bodyA==m_bodyB;
	for(int row=0;row<count;++row)
	{
		btAlignedObjectArray<btScalar> ja,ra,jb,rb;ja.resize(na);ra.resize(na);
		fillJacobian(m_bodyA,m_linkA,m_points[0],m_normal,m_common.m_inverseMassScale[0],m_common.m_inverseInertiaScale[0],&ja[0],&ra[0],data);
		if(m_bodyB){jb.resize(nb);rb.resize(nb);fillJacobian(m_bodyB,m_linkB,m_points[1],-m_normal,m_common.m_inverseMassScale[1],m_common.m_inverseInertiaScale[1],&jb[0],&rb[0],data);}
		btMultiBodySolverConstraint& s=output.expandNonInitializing();
		s.m_solverBodyIdA=data.m_fixedBodyId;s.m_solverBodyIdB=m_endpoints[1].m_rigidBody?m_endpoints[1].m_rigidBody->getCompanionId():data.m_fixedBodyId;
		fillMultiBodyConstraint(s,data,&ja[0],m_bodyB?&jb[0]:0,
			btVector3(0,0,0),m_normal,m_points[0],m_points[1],0,info,
			m_rows[row].m_lower,m_rows[row].m_upper,false,1,false,0);
		s.m_orgConstraint=this;s.m_orgDofIndex=row;
		btScalar w=0,v=0;
		btAlignedObjectArray<btScalar> response;
		response.resize(na);
		if(same) for(int i=0;i<na;++i)ra[i]+=rb[i];
		m_bodyA->calcAccelerationDeltasMultiDof(&ra[0],&response[0],data.scratch_r,data.scratch_v);
		for(int i=0;i<na;++i)
		{
			data.m_deltaVelocitiesUnitImpulse[s.m_jacAindex+i]=response[i];
			w+=ja[i]*response[i];v+=ja[i]*m_bodyA->getVelocityVector()[i];
		}
		if(m_bodyB)
		{
			if(!same){response.resize(nb);m_bodyB->calcAccelerationDeltasMultiDof(&rb[0],&response[0],data.scratch_r,data.scratch_v);}
			for(int i=0;i<nb;++i)
			{
				const btScalar d=same?response[i]:response[i];
				data.m_deltaVelocitiesUnitImpulse[s.m_jacBindex+i]=same?0:d;
				w+=jb[i]*d;v+=jb[i]*m_bodyB->getVelocityVector()[i];
			}
		}
		else if(m_endpoints[1].m_rigidBody)
		{
			btRigidBody* body=m_endpoints[1].m_rigidBody;
			const btVector3 n=-m_normal,torque=anchorTorque(m_endpoints[1],m_points[1],n);
			s.m_linearResponseOffsetB=m_common.m_inverseMassScale[1]-1;
			s.m_angularComponentB=body->getInvInertiaTensorWorld()*torque*body->getAngularFactor()*m_common.m_inverseInertiaScale[1];
			w+=n.dot(n*body->getInvMass()*body->getLinearFactor()*m_common.m_inverseMassScale[1])+torque.dot(s.m_angularComponentB*body->getAngularFactor());
			v+=n.dot(body->getLinearVelocity())+torque.dot(body->getAngularVelocity());
		}
		m_status=btCompleteDistanceConstraintRow(m_parameters,m_rows[row],w,v,info.m_timeStep,info.m_erp,s.m_jacDiagABInv,s.m_rhs,s.m_cfm);
		if(!m_status.isOk()||w<=SIMD_EPSILON){s.m_lowerLimit=s.m_upperLimit=0;s.m_rhs=s.m_jacDiagABInv=s.m_cfm=0;}
	}
}
void btMultiBodyDistanceConstraint::onExtensionRowSolved(int index,btScalar impulse,btScalar dt)
{finishDistanceRow(impulse,dt);if(index==m_rowCount-1&&btFabs(m_sumImpulse)/dt>m_common.m_breakForce)m_broken=true;}
