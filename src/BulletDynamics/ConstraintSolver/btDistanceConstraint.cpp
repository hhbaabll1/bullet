// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of NVIDIA CORPORATION nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
// OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Copyright (c) 2008-2025 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.

// Altered implementation: PhysX Distance geometry adapted to Bullet types and
// Bullet SI row preparation. No PhysX solver loop is included.
#include "btDistanceConstraint.h"
#include "btContactSolverInfo.h"
#include "BulletDynamics/Featherstone/btMultiBody.h"
#include "BulletDynamics/Featherstone/btMultiBodyLinkCollider.h"
#include "BulletDynamics/Featherstone/btMultiBodyDistanceConstraint.h"
#include "BulletDynamics/Featherstone/btMultiBodyDynamicsWorld.h"
#include "BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h"
#include "btConstraintSolver.h"
#include "LinearMath/btIDebugDraw.h"
#include <cstring>
#include <limits>

namespace
{
btConstraintStatus invalid(const char* message) { return btConstraintStatus(BT_CONSTRAINT_INVALID_PARAMETER, message); }
bool finiteVector(const btVector3& v) { return btConstraintScalarFinite(v.x()) && btConstraintScalarFinite(v.y()) && btConstraintScalarFinite(v.z()); }
bool validTransform(const btTransform& t)
{
	if (!finiteVector(t.getOrigin())) return false;
	const btMatrix3x3& b=t.getBasis();
	for (int i=0;i<3;++i)
	{
		if (!finiteVector(b[i]) || btFabs(b[i].length2()-1)>btScalar(1e-4)) return false;
		for (int j=0;j<i;++j) if (btFabs(b[i].dot(b[j]))>btScalar(1e-4)) return false;
	}
	return btFabs(b.determinant()-1)<btScalar(1e-4);
}
bool validFrame(const btConstraintLocalFrame& f)
{ return f.m_space>=BT_CONSTRAINT_COM_FRAME && f.m_space<=BT_CONSTRAINT_WORLD_FRAME && validTransform(f.m_transform); }
btConstraintStatus validateCommon(const btConstraintCommonParameters& c)
{
	if (!btConstraintScalarFinite(c.m_breakForce)||!btConstraintScalarFinite(c.m_breakTorque)||c.m_breakForce<0||c.m_breakTorque<0) return invalid("invalid break thresholds");
	for(int i=0;i<2;++i) if(!btConstraintScalarFinite(c.m_inverseMassScale[i])||!btConstraintScalarFinite(c.m_inverseInertiaScale[i])||c.m_inverseMassScale[i]<0||c.m_inverseInertiaScale[i]<0) return invalid("invalid local mass scale");
	return btConstraintStatus();
}
bool endpointDynamic(const btDistanceConstraintEndpoint& e)
{ return e.m_multiBody || (e.m_rigidBody && (!e.m_rigidBody->isStaticObject() || e.m_rigidBody->isKinematicObject())); }
void zeroFeedback(btJointFeedback& f)
{ f.m_appliedForceBodyA.setZero(); f.m_appliedForceBodyB.setZero(); f.m_appliedTorqueBodyA.setZero(); f.m_appliedTorqueBodyB.setZero(); }
void putDouble(btAlignedObjectArray<unsigned char>& bytes, btScalar value)
{
	double d=double(value); uint64_t bits; std::memcpy(&bits,&d,8);
	for(int i=0;i<8;++i) bytes.push_back(static_cast<unsigned char>(bits>>(i*8)));
}
btScalar getDouble(const unsigned char* bytes)
{
	uint64_t bits=0; for(int i=0;i<8;++i) bits|=uint64_t(bytes[i])<<(i*8);
	double d; std::memcpy(&d,&bits,8); return btScalar(d);
}
btConstraintStatus decode(const btConstraintDescription& d, btDistanceConstraintParameters& p, bool& broken)
{
	if(d.m_category!=BT_CONSTRAINT_CATEGORY_DISTANCE || d.m_payloadVersion!=1 || d.m_payload.size()!=46 || d.m_payload[0]!=1) return btConstraintStatus(BT_CONSTRAINT_INVALID_FORMAT,"invalid Distance payload");
	if(d.m_objects.size()!=2 || d.m_constraintReferences.size()!=0 || d.m_payload[45]>1) return invalid("Distance requires exactly two endpoints and no joint references");
	for(int i=0;i<2;++i)
	{
		const btConstraintObjectRef& r=d.m_objects[i];
		if(r.m_domain!=BT_CONSTRAINT_WORLD && r.m_domain!=BT_CONSTRAINT_RIGID_BODY && r.m_domain!=BT_CONSTRAINT_MULTIBODY_LINK) return invalid("Distance endpoint domain");
		if((r.m_domain==BT_CONSTRAINT_WORLD)!=(r.m_id==0) || (r.m_domain!=BT_CONSTRAINT_MULTIBODY_LINK && r.m_link!=-1) || r.m_link < -1) return invalid("Distance endpoint identity");
	}
	if(!d.m_id || d.m_objects[0]==d.m_objects[1] || !validFrame(d.m_frameA)||!validFrame(d.m_frameB)) return invalid("invalid Distance identity or frames");
	p.m_minDistance=getDouble(&d.m_payload[1]); p.m_maxDistance=getDouble(&d.m_payload[9]); p.m_tolerance=getDouble(&d.m_payload[17]); p.m_stiffness=getDouble(&d.m_payload[25]); p.m_damping=getDouble(&d.m_payload[33]);
	p.m_flags=0; for(int i=0;i<4;++i) p.m_flags|=unsigned(d.m_payload[41+i])<<(8*i);
	broken=d.m_payload[45]!=0;
	btConstraintStatus s=p.validate(); return s.isOk()?validateCommon(d.m_common):s;
}
}
btConstraintStatus btDistanceConstraintParameters::validate() const
{
	if(!btConstraintScalarFinite(m_minDistance)||!btConstraintScalarFinite(m_maxDistance)||m_minDistance<0||m_maxDistance<0 || !btConstraintScalarFinite(m_tolerance)||!btConstraintScalarFinite(m_stiffness)||!btConstraintScalarFinite(m_damping) || (m_flags&~7u)) return invalid("invalid Distance parameters");
	// Source setters permit min > max, negative finite tolerance/stiffness/damping.
	// Do not silently change that API contract. Singular runtime coefficients fail safely.
	return btConstraintStatus();
}
int btBuildDistanceConstraintRows(const btDistanceConstraintParameters& p, btScalar distance, btDistanceConstraintRow r[2])
{
	if(!p.validate().isOk() || !btConstraintScalarFinite(distance) || distance<0) return 0;
	const bool min=(p.m_flags&BT_DISTANCE_MIN_ENABLED)!=0, max=(p.m_flags&BT_DISTANCE_MAX_ENABLED)!=0;
	if(!min&&!max) return 0;
	if(min&&max&&p.m_minDistance==p.m_maxDistance)
	{
		const btScalar e=distance-p.m_maxDistance;
		r[0].m_error=e>p.m_tolerance?e-p.m_tolerance:e< -p.m_tolerance?e+p.m_tolerance:0;
		r[0].m_lower=-SIMD_INFINITY; r[0].m_upper=SIMD_INFINITY; r[0].m_keepBias=false; return 1;
	}
	int n=0;
	if(min) { r[n].m_error=distance-p.m_minDistance+p.m_tolerance; r[n].m_lower=0; r[n].m_upper=SIMD_INFINITY; r[n].m_keepBias=distance>=p.m_minDistance; ++n; }
	if(max) { r[n].m_error=distance-p.m_maxDistance-p.m_tolerance; r[n].m_lower=-SIMD_INFINITY; r[n].m_upper=0; r[n].m_keepBias=distance<=p.m_maxDistance; ++n; }
	return n;
}
btConstraintStatus btCompleteDistanceConstraintRow(const btDistanceConstraintParameters& p, const btDistanceConstraintRow& r,
	btScalar w, btScalar v, btScalar dt, btScalar erp, btScalar& diagonal, btScalar& rhs, btScalar& cfm)
{
	diagonal=rhs=cfm=0;
	if(!btConstraintScalarFinite(w)||!btConstraintScalarFinite(v)||!btConstraintScalarFinite(dt)||dt<=0 || !btConstraintScalarFinite(erp) || !btConstraintScalarFinite(r.m_error)) return invalid("non-finite Distance row");
	if(w<=SIMD_EPSILON) return btConstraintStatus(); // static/kinematic or redundant row: no impulse
	if(p.m_flags&BT_DISTANCE_SPRING_ENABLED)
	{
		const btScalar a=dt*(dt*p.m_stiffness+p.m_damping), denominator=1+a*w;
		if(!btConstraintScalarFinite(denominator)||denominator<=SIMD_EPSILON) return invalid("singular Distance spring response");
		diagonal=a/denominator; rhs=(-dt*p.m_stiffness*r.m_error-a*v)/denominator; cfm=1/denominator;
	}
	else { diagonal=1/w; rhs=(-erp*r.m_error/dt-v)/w; cfm=0; }
	if(!btConstraintScalarFinite(diagonal)||!btConstraintScalarFinite(rhs)||!btConstraintScalarFinite(cfm)) { diagonal=rhs=cfm=0; return invalid("Distance coefficient overflow"); }
	return btConstraintStatus();
}
btTransform btDistanceConstraintEndpoint::comWorld() const
{
	if(m_rigidBody) return m_rigidBody->getCenterOfMassTransform();
	if(m_multiBody) return m_link<0?m_multiBody->getBaseWorldTransform():m_multiBody->getLink(m_link).m_cachedWorldTransform;
	return btTransform::getIdentity();
}
btTransform btDistanceConstraintEndpoint::frameWorld(const btConstraintLocalFrame& f) const
{ return f.m_space==BT_CONSTRAINT_WORLD_FRAME?f.m_transform:comWorld()*(f.m_space==BT_CONSTRAINT_ACTOR_FRAME?m_actorToCom*f.m_transform:f.m_transform); }
btCollisionObject* btDistanceConstraintEndpoint::collider() const
{ return m_rigidBody?static_cast<btCollisionObject*>(m_rigidBody):m_multiBody?(m_link<0?m_multiBody->getBaseCollider():m_multiBody->getLink(m_link).m_collider):0; }
void btDistanceConstraintEndpoint::wake() const
{ if(m_rigidBody) m_rigidBody->activate(true); if(m_multiBody) m_multiBody->wakeUp(); if(collider()) collider()->activate(true); }
bool btDistanceConstraintEndpoint::valid() const
{ return !(m_rigidBody&&m_multiBody) && (!m_multiBody?(m_link==-1):(m_link>=-1&&m_link<m_multiBody->getNumLinks())) && validTransform(m_actorToCom); }
bool btDistanceConstraintEndpoint::sameObject(const btDistanceConstraintEndpoint& o) const
{ return m_rigidBody==o.m_rigidBody&&m_multiBody==o.m_multiBody&&m_link==o.m_link; }
btDistanceConstraintState::btDistanceConstraintState(const btDistanceConstraintEndpoint& a,const btDistanceConstraintEndpoint& b,const btVector3& pa,const btVector3& pb)
	: m_normal(1,0,0),m_distance(0),m_sumImpulse(0),m_dt(0),m_rowCount(0),m_broken(false)
{
	m_endpoints[0]=a; m_endpoints[1]=b; m_frames[0].m_transform.setOrigin(pa); m_frames[1].m_transform.setOrigin(pb);
	m_common.m_outputForce=true; zeroFeedback(m_feedback);
}
btConstraintStatus btDistanceConstraintState::setParameters(const btDistanceConstraintParameters& p)
{
	btConstraintStatus s=p.validate(); if(!s.isOk()) return s;
	m_parameters=p; if(m_common.m_autoWake) for(int i=0;i<2;++i) m_endpoints[i].wake(); return m_status=btConstraintStatus();
}
btConstraintStatus btDistanceConstraintState::setCommonParameters(const btConstraintCommonParameters& c)
{
	btConstraintStatus s=validateCommon(c); if(!s.isOk()) return s;
	m_common=c; if(c.m_autoWake) for(int i=0;i<2;++i) m_endpoints[i].wake(); return m_status=btConstraintStatus();
}
btConstraintStatus btDistanceConstraintState::setFrames(const btConstraintLocalFrame& a,const btConstraintLocalFrame& b)
{
	if(!validFrame(a)||!validFrame(b)) return invalid("invalid Distance frames");
	m_frames[0]=a;m_frames[1]=b; if(m_common.m_autoWake) for(int i=0;i<2;++i) m_endpoints[i].wake(); return m_status=btConstraintStatus();
}
btScalar btDistanceConstraintState::getDistance() const
{ return (m_endpoints[0].frameWorld(m_frames[0]).getOrigin()-m_endpoints[1].frameWorld(m_frames[1]).getOrigin()).length(); }
void btDistanceConstraintState::repair() { m_broken=false; if(m_common.m_autoWake) for(int i=0;i<2;++i) m_endpoints[i].wake(); }
int btDistanceConstraintState::prepareDistance()
{
	m_rowCount=0; m_sumImpulse=0; zeroFeedback(m_feedback);
	if(!m_endpoints[0].valid()||!m_endpoints[1].valid()||m_endpoints[0].sameObject(m_endpoints[1])||(!endpointDynamic(m_endpoints[0])&&!endpointDynamic(m_endpoints[1]))) { m_status=invalid("invalid Distance endpoint pair"); return 0; }
	m_status=m_parameters.validate(); if(!m_status.isOk()||!m_common.m_enabled||m_broken) return 0;
	for(int i=0;i<2;++i) m_points[i]=m_endpoints[i].frameWorld(m_frames[i]).getOrigin();
	m_normal=m_points[0]-m_points[1];m_distance=m_normal.length();
	if(!btConstraintScalarFinite(m_distance)) {m_status=invalid("invalid Distance geometry");return 0;}
	m_normal=m_distance<btScalar(1.192092896e-7)?btVector3(1,0,0):m_normal/m_distance;
	return m_rowCount=btBuildDistanceConstraintRows(m_parameters,m_distance,m_rows);
}
void btDistanceConstraintState::finishDistanceRow(btScalar impulse,btScalar dt)
{
	m_sumImpulse+=impulse; m_dt=dt;
	const btVector3 force=m_normal*(m_sumImpulse/dt);
	if(m_common.m_outputForce)
	{
		m_feedback.m_appliedForceBodyA=force; m_feedback.m_appliedForceBodyB=-force;
		m_feedback.m_appliedTorqueBodyA=(m_points[0]-m_endpoints[0].comWorld().getOrigin()).cross(force);
		m_feedback.m_appliedTorqueBodyB=(m_points[1]-m_endpoints[1].comWorld().getOrigin()).cross(-force);
	}
	// Distance has no independent torque at its joint anchors. COM lever-arm
	// torques are exposed above, but are not a breakTorque contribution.
}
void btDistanceConstraintState::debugDrawDistance(btIDebugDraw* drawer) const
{
	if(!drawer) return;
	const btTransform a=m_endpoints[0].frameWorld(m_frames[0]),b=m_endpoints[1].frameWorld(m_frames[1]);
	drawer->drawTransform(a,btScalar(.1));drawer->drawTransform(b,btScalar(.1));
	const btScalar d=(a.getOrigin()-b.getOrigin()).length();
	btVector3 color(0,1,0);
	if((m_parameters.m_flags&BT_DISTANCE_MAX_ENABLED)&&d>m_parameters.m_maxDistance) color.setValue(1,0,0);
	if((m_parameters.m_flags&BT_DISTANCE_MIN_ENABLED)&&d<m_parameters.m_minDistance) color.setValue(0,0,1);
	drawer->drawLine(a.getOrigin(),b.getOrigin(),color);
}
btConstraintStatus btDistanceConstraintState::validateDescription(const btConstraintDescription& d)
{ btDistanceConstraintParameters p;bool broken;return decode(d,p,broken); }
btConstraintStatus btDistanceConstraintState::writeDescription(uint64_t id,const btConstraintObjectRef& a,const btConstraintObjectRef& b,btConstraintDescription& d) const
{
	btConstraintDescription n;n.m_id=id;n.m_category=BT_CONSTRAINT_CATEGORY_DISTANCE;n.m_objects.push_back(a);n.m_objects.push_back(b);
	n.m_frameA=m_frames[0];n.m_frameB=m_frames[1];n.m_common=m_common;n.m_payloadVersion=1;n.m_payload.push_back(1);
	putDouble(n.m_payload,m_parameters.m_minDistance);putDouble(n.m_payload,m_parameters.m_maxDistance);putDouble(n.m_payload,m_parameters.m_tolerance);putDouble(n.m_payload,m_parameters.m_stiffness);putDouble(n.m_payload,m_parameters.m_damping);
	for(int i=0;i<4;++i)n.m_payload.push_back(static_cast<unsigned char>(m_parameters.m_flags>>(i*8)));
	n.m_payload.push_back(m_broken?1:0);
	btConstraintStatus s=validateDescription(n);if(!s.isOk())return s;
	// Clear nested arrays before Bullet's placement-copy assignment.
	d.m_objects.clear();d.m_constraintReferences.clear();d.m_payload.clear();d=n;return s;
}
btConstraintStatus btDistanceConstraintState::readDescription(const btConstraintDescription& d)
{
	btDistanceConstraintParameters p;bool broken;btConstraintStatus s=decode(d,p,broken);if(!s.isOk())return s;
	m_parameters=p;m_common=d.m_common;m_frames[0]=d.m_frameA;m_frames[1]=d.m_frameB;m_broken=broken;
	if(m_common.m_autoWake)for(int i=0;i<2;++i)m_endpoints[i].wake();return m_status=btConstraintStatus();
}
btDistanceConstraint::btDistanceConstraint(btRigidBody& a,btRigidBody& b,const btVector3& pa,const btVector3& pb)
	: btTypedConstraint(DISTANCE_CONSTRAINT_TYPE,a,b),btDistanceConstraintState(btDistanceConstraintEndpoint(&a),btDistanceConstraintEndpoint(&b),pa,pb) {}
btDistanceConstraint::btDistanceConstraint(btRigidBody& a,const btVector3& pa,const btVector3& pb)
	: btTypedConstraint(DISTANCE_CONSTRAINT_TYPE,a),btDistanceConstraintState(btDistanceConstraintEndpoint(&a),btDistanceConstraintEndpoint(),pa,pb) {}
void btDistanceConstraint::getInfo1(btConstraintInfo1* info) {info->m_numConstraintRows=prepareDistance();info->nub=0;}
void btDistanceConstraint::getInfo2(btConstraintInfo2* info)
{
	for(int i=0;i<m_rowCount;++i)
	{
		const int o=i*info->rowskip; const btVector3 ra=m_points[0]-m_rbA.getCenterOfMassPosition(),rb=m_points[1]-m_rbB.getCenterOfMassPosition();
		for(int j=0;j<3;++j){info->m_J1linearAxis[o+j]=m_normal[j];info->m_J2linearAxis[o+j]=-m_normal[j];info->m_J1angularAxis[o+j]=ra.cross(m_normal)[j];info->m_J2angularAxis[o+j]=rb.cross(-m_normal)[j];}
		info->m_constraintError[o]=-info->erp*m_rows[i].m_error*info->fps;info->m_lowerLimit[o]=m_rows[i].m_lower;info->m_upperLimit[o]=m_rows[i].m_upper;
	}
}
bool btDistanceConstraint::finalizeExtensionRow(btSolverConstraint& r,int index,const btContactSolverInfo& info,const btSolverBody& a,const btSolverBody& b)
{
	r.m_linearResponseOffsetA=m_common.m_inverseMassScale[0]-1;r.m_linearResponseOffsetB=m_common.m_inverseMassScale[1]-1;
	r.m_angularComponentA=m_rbA.getInvInertiaTensorWorld()*r.m_relpos1CrossNormal*m_rbA.getAngularFactor()*m_common.m_inverseInertiaScale[0];
	r.m_angularComponentB=m_rbB.getInvInertiaTensorWorld()*r.m_relpos2CrossNormal*m_rbB.getAngularFactor()*m_common.m_inverseInertiaScale[1];
	const btScalar w=r.m_contactNormal1.dot(r.m_contactNormal1*a.internalGetInvMass()*m_common.m_inverseMassScale[0]*m_rbA.getLinearFactor())+r.m_contactNormal2.dot(r.m_contactNormal2*b.internalGetInvMass()*m_common.m_inverseMassScale[1]*m_rbB.getLinearFactor())+r.m_relpos1CrossNormal.dot(r.m_angularComponentA*m_rbA.getAngularFactor())+r.m_relpos2CrossNormal.dot(r.m_angularComponentB*m_rbB.getAngularFactor());
	const btScalar v=r.m_contactNormal1.dot(m_rbA.getLinearVelocity()+a.m_externalForceImpulse)+r.m_contactNormal2.dot(m_rbB.getLinearVelocity()+b.m_externalForceImpulse)+r.m_relpos1CrossNormal.dot(m_rbA.getAngularVelocity()+a.m_externalTorqueImpulse)+r.m_relpos2CrossNormal.dot(m_rbB.getAngularVelocity()+b.m_externalTorqueImpulse);
	m_status=btCompleteDistanceConstraintRow(m_parameters,m_rows[index],w,v,info.m_timeStep,info.m_erp,r.m_jacDiagABInv,r.m_rhs,r.m_cfm);
	r.m_frictionIndex=index;
	if(!m_status.isOk()||w<=SIMD_EPSILON){r.m_lowerLimit=r.m_upperLimit=0;r.m_rhs=0;r.m_jacDiagABInv=0;r.m_cfm=0;}
	return true;
}
void btDistanceConstraint::onExtensionRowSolved(int index,btScalar impulse,btScalar dt)
{
	finishDistanceRow(impulse,dt);
	if(index==m_rowCount-1 && btFabs(m_sumImpulse)/dt>m_common.m_breakForce)m_broken=true;
}
void btDistanceConstraint::setParam(int,btScalar,int) {m_status=btConstraintStatus(BT_CONSTRAINT_UNSUPPORTED,"use Distance parameters; ERP comes from solver info");}
btScalar btDistanceConstraint::getParam(int,int) const {return std::numeric_limits<btScalar>::quiet_NaN();}

btDistanceConstraintFactory::btDistanceConstraintFactory(btDiscreteDynamicsWorld& w,const btDistanceConstraintResolver& r)
	:m_world(w),m_multiWorld(0),m_resolver(r) {}
btDistanceConstraintFactory::btDistanceConstraintFactory(btMultiBodyDynamicsWorld& w,const btDistanceConstraintResolver& r)
	:m_world(w),m_multiWorld(&w),m_resolver(r) {}
btDistanceConstraintFactory::~btDistanceConstraintFactory()
{ while(m_entries.size())remove(m_entries[m_entries.size()-1].m_id); }
btConstraintStatus btDistanceConstraintFactory::resolveDescription(const btConstraintDescription& d,btDistanceConstraintEndpoint (&e)[2]) const
{
	btConstraintStatus s=btDistanceConstraintState::validateDescription(d);if(!s.isOk())return s;
	for(int i=0;i<2;++i)
	{
		if(d.m_objects[i].m_domain==BT_CONSTRAINT_WORLD)e[i]=btDistanceConstraintEndpoint();
		else {s=m_resolver.resolve(d.m_objects[i],e[i]);if(!s.isOk())return s;}
		if(!e[i].valid())return invalid("resolver returned invalid Distance endpoint");
		if((d.m_objects[i].m_domain==BT_CONSTRAINT_WORLD && (e[i].m_rigidBody||e[i].m_multiBody)) ||
			(d.m_objects[i].m_domain==BT_CONSTRAINT_RIGID_BODY && (!e[i].m_rigidBody||e[i].m_multiBody)) ||
			(d.m_objects[i].m_domain==BT_CONSTRAINT_MULTIBODY_LINK && (!e[i].m_multiBody||e[i].m_rigidBody||e[i].m_link!=d.m_objects[i].m_link))) return invalid("resolver endpoint type does not match reference");
		if(e[i].m_multiBody&&!m_multiWorld)return btConstraintStatus(BT_CONSTRAINT_UNSUPPORTED,"MultiBody world required");
		if(e[i].collider())
		{
			bool registered=false;const btCollisionObjectArray& objects=m_world.getCollisionObjectArray();
			for(int j=0;j<objects.size();++j)if(objects[j]==e[i].collider())registered=true;
			if(!registered)return btConstraintStatus(BT_CONSTRAINT_MISSING_REFERENCE,"endpoint collider is not in world");
		}
	}
	if(e[0].sameObject(e[1])||(!endpointDynamic(e[0])&&!endpointDynamic(e[1])))return invalid("invalid Distance pair: same endpoint or no rigid body response");
	if(d.m_objects[0].m_domain==BT_CONSTRAINT_WORLD && d.m_objects[1].m_domain==BT_CONSTRAINT_WORLD)return invalid("world-world Distance");
	return btConstraintStatus();
}
btConstraintStatus btDistanceConstraintFactory::canCreate(const btConstraintDescription& d) const
{
	if(find(d.m_id))return btConstraintStatus(BT_CONSTRAINT_DUPLICATE_ID,"Distance id exists");
	btDistanceConstraintEndpoint e[2];btConstraintStatus s=resolveDescription(d,e);if(!s.isOk())return s;
	const btConstraintSolverType type=m_world.getConstraintSolver()->getSolverType();
	if(type!=BT_SEQUENTIAL_IMPULSE_SOLVER && type!=BT_MULTIBODY_SOLVER)return btConstraintStatus(BT_CONSTRAINT_UNSUPPORTED,"Distance solver backend has no local-response hook");
	return btConstraintStatus();
}
void btDistanceConstraintFactory::updateCollision(Entry& e,bool enabled)
{
	if(!e.m_colliders[0]||!e.m_colliders[1]||e.m_ignored==!enabled)return;
	if(!enabled)
	{
		bool already=false;
		for(int i=0;i<m_entries.size();++i)if(m_entries[i].m_ignored &&
			((m_entries[i].m_colliders[0]==e.m_colliders[0]&&m_entries[i].m_colliders[1]==e.m_colliders[1]) ||
			 (m_entries[i].m_colliders[1]==e.m_colliders[0]&&m_entries[i].m_colliders[0]==e.m_colliders[1])))already=true;
		if(!already){e.m_colliders[0]->setIgnoreCollisionCheck(e.m_colliders[1],true);e.m_colliders[1]->setIgnoreCollisionCheck(e.m_colliders[0],true);}
		e.m_ignored=true;
	}
	else
	{
		e.m_ignored=false;bool other=false;
		for(int i=0;i<m_entries.size();++i)if(m_entries[i].m_ignored &&
			((m_entries[i].m_colliders[0]==e.m_colliders[0]&&m_entries[i].m_colliders[1]==e.m_colliders[1]) ||
			 (m_entries[i].m_colliders[1]==e.m_colliders[0]&&m_entries[i].m_colliders[0]==e.m_colliders[1])))other=true;
		if(!other){e.m_colliders[0]->setIgnoreCollisionCheck(e.m_colliders[1],false);e.m_colliders[1]->setIgnoreCollisionCheck(e.m_colliders[0],false);}
	}
}
btConstraintStatus btDistanceConstraintFactory::create(const btConstraintDescription& d)
{
	btConstraintStatus s=canCreate(d);if(!s.isOk())return s;
	btDistanceConstraintEndpoint e[2];s=resolveDescription(d,e);if(!s.isOk())return s;
	Entry n={};n.m_id=d.m_id;n.m_refs[0]=d.m_objects[0];n.m_refs[1]=d.m_objects[1];n.m_colliders[0]=e[0].collider();n.m_colliders[1]=e[1].collider();
	if(e[0].m_multiBody||e[1].m_multiBody)
	{
		if(!m_multiWorld)return btConstraintStatus(BT_CONSTRAINT_UNSUPPORTED,"MultiBody world required");
		n.m_multi=new btMultiBodyDistanceConstraint(e[0],e[1],btVector3(0,0,0),btVector3(0,0,0));n.m_state=n.m_multi;
	}
	else
	{
		btRigidBody* a=e[0].m_rigidBody?e[0].m_rigidBody:e[1].m_rigidBody;
		btRigidBody* b=e[0].m_rigidBody?e[1].m_rigidBody:0;
		if(b)n.m_rigid=new btDistanceConstraint(*a,*b,btVector3(0,0,0),btVector3(0,0,0));
		else n.m_rigid=new btDistanceConstraint(*a,btVector3(0,0,0),btVector3(0,0,0));
		n.m_state=static_cast<btDistanceConstraint*>(n.m_rigid);
	}
	btConstraintDescription normalized=d;
	const bool swap=(e[1].m_multiBody&&!e[0].m_multiBody)||(e[1].m_rigidBody&&!e[0].m_rigidBody);
	n.m_swapped=swap;
	if(swap){btConstraintLocalFrame t=normalized.m_frameA;normalized.m_frameA=normalized.m_frameB;normalized.m_frameB=t;}
	s=n.m_state->readDescription(normalized);
	if(!s.isOk()){delete n.m_multi;delete n.m_rigid;return s;}
	if(n.m_rigid){n.m_rigid->setBreakingImpulseThreshold(SIMD_INFINITY);m_world.addConstraint(n.m_rigid,false);}
	else m_multiWorld->addMultiBodyConstraint(n.m_multi);
	m_entries.push_back(n);updateCollision(m_entries[m_entries.size()-1],d.m_common.m_collisionEnabled);
	return btConstraintStatus();
}
void btDistanceConstraintFactory::remove(uint64_t id)
{
	for(int i=0;i<m_entries.size();++i)if(m_entries[i].m_id==id)
	{
		Entry& e=m_entries[i];updateCollision(e,true);
		if(e.m_rigid)m_world.removeConstraint(e.m_rigid);
		else m_multiWorld->removeMultiBodyConstraint(e.m_multi);
		delete e.m_multi;delete e.m_rigid;m_entries.removeAtIndex(i);return;
	}
}
void btDistanceConstraintFactory::removeObject(const btConstraintObjectRef& r)
{
	for(int i=m_entries.size()-1;i>=0;--i)
		if((m_entries[i].m_refs[0].m_id==r.m_id&&m_entries[i].m_refs[0].m_domain==r.m_domain) ||
		   (m_entries[i].m_refs[1].m_id==r.m_id&&m_entries[i].m_refs[1].m_domain==r.m_domain))remove(m_entries[i].m_id);
}
btConstraintStatus btDistanceConstraintFactory::update(uint64_t id,const btConstraintDescription& d)
{
	for(int i=0;i<m_entries.size();++i)if(m_entries[i].m_id==id)
	{
		Entry& e=m_entries[i];if(d.m_id!=id||d.m_objects.size()!=2||!(d.m_objects[0]==e.m_refs[0])||!(d.m_objects[1]==e.m_refs[1]))return invalid("Distance update cannot change identity or endpoints");
		btConstraintDescription n=d;
		if(e.m_swapped){btConstraintLocalFrame t=n.m_frameA;n.m_frameA=n.m_frameB;n.m_frameB=t;}
		btConstraintStatus s=e.m_state->readDescription(n);if(!s.isOk())return s;
		updateCollision(e,d.m_common.m_collisionEnabled);return s;
	}
	return btConstraintStatus(BT_CONSTRAINT_MISSING_REFERENCE,"Distance id not found");
}
btDistanceConstraintState* btDistanceConstraintFactory::find(uint64_t id) const
{for(int i=0;i<m_entries.size();++i)if(m_entries[i].m_id==id)return m_entries[i].m_state;return 0;}
