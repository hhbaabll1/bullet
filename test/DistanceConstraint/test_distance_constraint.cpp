/* New Bullet extension integration tests, 2026. Distributed under Bullet zlib. */
#include "BulletDynamics/ConstraintSolver/btDistanceConstraint.h"
#include "BulletDynamics/Featherstone/btMultiBodyDistanceConstraint.h"
#include "BulletDynamics/Featherstone/btMultiBodyDynamicsWorld.h"
#include "BulletDynamics/Featherstone/btMultiBodyConstraintSolver.h"
#include "BulletDynamics/Featherstone/btMultiBodyLinkCollider.h"
#include "BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.h"
#include "BulletCollision/CollisionDispatch/btDefaultCollisionConfiguration.h"
#include "BulletCollision/CollisionDispatch/btCollisionDispatcher.h"
#include "BulletCollision/BroadphaseCollision/btDbvtBroadphase.h"
#include "BulletCollision/CollisionShapes/btSphereShape.h"
#include <cstdio>
#include <cstring>
#include <cmath>
#include <limits>
#define CHECK(x) do { if(!(x)){std::fprintf(stderr,"FAIL line %d: %s\n",__LINE__,#x);return false;} }while(0)
static bool near(btScalar a,btScalar b,btScalar eps=btScalar(2e-2)) {return btFabs(a-b)<=eps;}

static bool rows()
{
	btDistanceConstraintParameters p(2);btDistanceConstraintRow r[2];
	CHECK(p.m_flags==BT_DISTANCE_MAX_ENABLED && near(p.m_tolerance,btScalar(.05)) && p.m_minDistance==0 && p.m_maxDistance==0);
	CHECK(btBuildDistanceConstraintRows(p,btScalar(1),r)==1 && near(r[0].m_error,btScalar(.95)) && r[0].m_upper==0);
	p.m_flags=BT_DISTANCE_MIN_ENABLED;p.m_minDistance=2;p.m_tolerance=btScalar(.1);
	CHECK(btBuildDistanceConstraintRows(p,btScalar(1),r)==1 && near(r[0].m_error,btScalar(-.9)) && r[0].m_lower==0 && !r[0].m_keepBias);
	CHECK(btBuildDistanceConstraintRows(p,btScalar(2.5),r)==1 && r[0].m_keepBias);
	p.m_flags=BT_DISTANCE_MIN_ENABLED|BT_DISTANCE_MAX_ENABLED;p.m_maxDistance=4;
	CHECK(btBuildDistanceConstraintRows(p,btScalar(3),r)==2 && r[0].m_keepBias && r[1].m_keepBias);
	CHECK(r[0].m_lower==0 && r[1].m_upper==0 && near(r[0].m_error,btScalar(1.1)) && near(r[1].m_error,btScalar(-1.1)));
	p.m_maxDistance=2;CHECK(btBuildDistanceConstraintRows(p,btScalar(2.05),r)==1 && near(r[0].m_error,0) && r[0].m_lower<0 && r[0].m_upper>0);
	CHECK(btBuildDistanceConstraintRows(p,btScalar(2.5),r)==1 && near(r[0].m_error,btScalar(.4)));
	p.m_flags=0;CHECK(btBuildDistanceConstraintRows(p,btScalar(2),r)==0);
	p.m_flags=BT_DISTANCE_SPRING_ENABLED|BT_DISTANCE_MAX_ENABLED;p.m_stiffness=100;p.m_damping=2;
	btScalar d,rhs,cfm;btDistanceConstraintRow row={btScalar(1),-SIMD_INFINITY,btScalar(0),false};
	CHECK(btCompleteDistanceConstraintRow(p,row,btScalar(.5),0,btScalar(.1),btScalar(.2),d,rhs,cfm).isOk());
	CHECK(near(d,btScalar(.75)) && near(rhs,btScalar(-6.25)) && near(cfm,btScalar(.625)));
	p.m_flags=BT_DISTANCE_MAX_ENABLED;
	CHECK(btCompleteDistanceConstraintRow(p,row,btScalar(.5),0,btScalar(.1),btScalar(.2),d,rhs,cfm).isOk());
	CHECK(near(d,2) && near(rhs,-4) && cfm==0);
	p.m_tolerance=-1;p.m_stiffness=-5;p.m_damping=-5;CHECK(p.validate().isOk());
	CHECK(btCompleteDistanceConstraintRow(p,row,0,0,btScalar(.1),btScalar(.2),d,rhs,cfm).isOk() && d==0);
	p.m_maxDistance=std::numeric_limits<btScalar>::quiet_NaN();CHECK(!p.validate().isOk());
	return true;
}
struct RigidWorld
{
	btDefaultCollisionConfiguration config;btCollisionDispatcher dispatcher;btDbvtBroadphase broad;btSequentialImpulseConstraintSolver solver;
	btDiscreteDynamicsWorld world;btSphereShape shape;
	RigidWorld():dispatcher(&config),world(&dispatcher,&broad,&solver,&config),shape(btScalar(.1)){world.setGravity(btVector3(0,0,0));}
};
static btRigidBody* makeBody(btCollisionShape& shape,btScalar mass,const btVector3& position)
{
	btVector3 inertia(0,0,0);if(mass>0)shape.calculateLocalInertia(mass,inertia);
	btRigidBody::btRigidBodyConstructionInfo ci(mass,0,&shape,inertia);btRigidBody* body=new btRigidBody(ci);
	body->setWorldTransform(btTransform(btQuaternion::getIdentity(),position));body->setDamping(0,0);return body;
}
static bool rigid()
{
	RigidWorld w;btRigidBody* body=makeBody(w.shape,2,btVector3(2,0,0));w.world.addRigidBody(body);
	btDistanceConstraint c(*body,btVector3(0,0,0),btVector3(0,0,0));
	btDistanceConstraintParameters p;p.m_maxDistance=1;p.m_tolerance=0;
	CHECK(c.setParameters(p).isOk());w.world.addConstraint(&c);
	w.world.stepSimulation(btScalar(.1),0,btScalar(.1));
	CHECK(body->getLinearVelocity().x()<btScalar(-.5) && c.getStatus().isOk());
	CHECK(near(c.getDistance(),(body->getCenterOfMassPosition()).length(),btScalar(1e-3)));
	CHECK(c.getDistanceFeedback().m_appliedForceBodyA.x()<0);
	w.world.removeConstraint(&c);w.world.removeRigidBody(body);delete body;
	return true;
}
static bool rigid_modes()
{
	RigidWorld w;btRigidBody* a=makeBody(w.shape,2,btVector3(0,0,0));btRigidBody* b=makeBody(w.shape,3,btVector3(2,0,0));
	w.world.addRigidBody(a);w.world.addRigidBody(b);
	btDistanceConstraint c(*a,*b,btVector3(0,0,0),btVector3(0,0,0));
	btDistanceConstraintParameters p;p.m_flags=BT_DISTANCE_MIN_ENABLED|BT_DISTANCE_MAX_ENABLED;p.m_minDistance=1;p.m_maxDistance=3;p.m_tolerance=0;
	CHECK(c.setParameters(p).isOk());w.world.addConstraint(&c);
	btTypedConstraint::btConstraintInfo1 info;c.getInfo1(&info);CHECK(info.m_numConstraintRows==2);
	w.world.stepSimulation(btScalar(.1),0,btScalar(.1));CHECK(near(a->getLinearVelocity().x(),0)&&near(b->getLinearVelocity().x(),0));
	p.m_flags=BT_DISTANCE_MIN_ENABLED;p.m_minDistance=3;CHECK(c.setParameters(p).isOk());
	w.world.stepSimulation(btScalar(.1),0,btScalar(.1));CHECK(a->getLinearVelocity().x()<0&&b->getLinearVelocity().x()>0);
	CHECK(near(a->getLinearVelocity().x()*2+b->getLinearVelocity().x()*3,0,btScalar(.01)));
	w.world.removeConstraint(&c);w.world.removeRigidBody(a);w.world.removeRigidBody(b);delete a;delete b;
	return true;
}
static bool rigid_offset()
{
	RigidWorld w;btRigidBody* body=makeBody(w.shape,2,btVector3(2,0,0));w.world.addRigidBody(body);
	btDistanceConstraint c(*body,btVector3(0,1,0),btVector3(0,0,0));
	btDistanceConstraintParameters p;p.m_maxDistance=1;p.m_tolerance=0;CHECK(c.setParameters(p).isOk());
	w.world.addConstraint(&c);w.world.stepSimulation(btScalar(.1),0,btScalar(.1));
	CHECK(body->getLinearVelocity().x()<0 && body->getAngularVelocity().z()>0);
	CHECK(c.getDistanceFeedback().m_appliedTorqueBodyA.z()>0);
	w.world.removeConstraint(&c);w.world.removeRigidBody(body);delete body;return true;
}
static bool spring()
{
	RigidWorld w;btRigidBody* a=makeBody(w.shape,2,btVector3(2,0,0));w.world.addRigidBody(a);
	btDistanceConstraint c(*a,btVector3(0,0,0),btVector3(0,0,0));
	btDistanceConstraintParameters p;p.m_flags=BT_DISTANCE_MAX_ENABLED|BT_DISTANCE_SPRING_ENABLED;p.m_maxDistance=1;p.m_tolerance=0;p.m_stiffness=100;p.m_damping=2;
	CHECK(c.setParameters(p).isOk());w.world.addConstraint(&c);
	w.world.stepSimulation(btScalar(.1),0,btScalar(.1));
	CHECK(c.getStatus().isOk() && a->getLinearVelocity().x()<btScalar(-.1) && a->getLinearVelocity().x()>btScalar(-10));
	w.world.removeConstraint(&c);w.world.removeRigidBody(a);delete a;return true;
}
static bool break_scale()
{
	RigidWorld w;btRigidBody* a=makeBody(w.shape,2,btVector3(0,0,0));btRigidBody* b=makeBody(w.shape,3,btVector3(2,0,0));
	w.world.addRigidBody(a);w.world.addRigidBody(b);
	btDistanceConstraint c(*a,*b,btVector3(0,0,0),btVector3(0,0,0));
	btDistanceConstraintParameters p;p.m_flags=BT_DISTANCE_MIN_ENABLED;p.m_minDistance=3;p.m_tolerance=0;CHECK(c.setParameters(p).isOk());
	btConstraintCommonParameters common;common.m_inverseMassScale[0]=0;common.m_inverseInertiaScale[0]=0;common.m_breakForce=1;
	CHECK(c.setCommonParameters(common).isOk());w.world.addConstraint(&c);w.world.stepSimulation(btScalar(.1),0,btScalar(.1));
	CHECK(near(a->getLinearVelocity().x(),0) && b->getLinearVelocity().x()>0 && c.isBroken());
	btTypedConstraint::btConstraintInfo1 info;c.getInfo1(&info);CHECK(info.m_numConstraintRows==0);
	c.repair();c.getInfo1(&info);CHECK(info.m_numConstraintRows==1);
	w.world.removeConstraint(&c);w.world.removeRigidBody(a);w.world.removeRigidBody(b);delete a;delete b;return true;
}
static bool zero_response()
{
	RigidWorld w;btRigidBody* a=makeBody(w.shape,0,btVector3(2,0,0));
	a->setCollisionFlags(a->getCollisionFlags()|btCollisionObject::CF_KINEMATIC_OBJECT);
	a->setActivationState(DISABLE_DEACTIVATION);a->setInterpolationWorldTransform(a->getWorldTransform());w.world.addRigidBody(a);
	btDistanceConstraint c(*a,btVector3(0,0,0),btVector3(0,0,0));
	btDistanceConstraintParameters p;p.m_maxDistance=1;p.m_tolerance=0;CHECK(c.setParameters(p).isOk());w.world.addConstraint(&c);
	w.world.stepSimulation(btScalar(.1),0,btScalar(.1));
	if(!c.getStatus().isOk()||!near(a->getLinearVelocity().x(),0))std::fprintf(stderr,"zero response status=%s vel=%g\n",c.getStatus().m_message,double(a->getLinearVelocity().x()));
	CHECK(c.getStatus().isOk()&&near(a->getLinearVelocity().x(),0));
	w.world.removeConstraint(&c);w.world.removeRigidBody(a);delete a;return true;
}
struct MultiWorld
{
	btDefaultCollisionConfiguration config;btCollisionDispatcher dispatcher;btDbvtBroadphase broad;btMultiBodyConstraintSolver solver;
	btMultiBodyDynamicsWorld world;btSphereShape shape;
	MultiWorld():dispatcher(&config),world(&dispatcher,&broad,&solver,&config),shape(btScalar(.1)){world.setGravity(btVector3(0,0,0));}
};
static void initBase(MultiWorld& w,btMultiBody& m,btMultiBodyLinkCollider& c,const btVector3& x)
{
	m.finalizeMultiDof();m.setLinearDamping(0);m.setAngularDamping(0);
	m.setBaseWorldTransform(btTransform(btQuaternion::getIdentity(),x));
	c.setCollisionShape(&w.shape);c.setWorldTransform(m.getBaseWorldTransform());m.setBaseCollider(&c);
	w.world.addMultiBody(&m);w.world.addCollisionObject(&c,1,0);
}
static bool multibody()
{
	MultiWorld w;btMultiBody a(0,2,btVector3(1,1,1),false,false),b(0,3,btVector3(1,1,1),false,false);
	btMultiBodyLinkCollider ca(&a,-1),cb(&b,-1);initBase(w,a,ca,btVector3(2,0,0));initBase(w,b,cb,btVector3(0,0,0));
	btMultiBodyDistanceConstraint c(btDistanceConstraintEndpoint(&a,-1),btDistanceConstraintEndpoint(&b,-1),btVector3(0,0,0),btVector3(0,0,0));
	btDistanceConstraintParameters p;p.m_maxDistance=1;p.m_tolerance=0;CHECK(c.setParameters(p).isOk());
	w.world.addMultiBodyConstraint(&c);w.world.stepSimulation(btScalar(.1),0,btScalar(.1));
	CHECK(c.getStatus().isOk() && a.getBaseVel().x()<0 && b.getBaseVel().x()>0);
	CHECK(near(a.getBaseVel().x()*2+b.getBaseVel().x()*3,0,btScalar(.01)));
	w.world.removeMultiBodyConstraint(&c);w.world.removeCollisionObject(&ca);w.world.removeCollisionObject(&cb);w.world.removeMultiBody(&a);w.world.removeMultiBody(&b);
	a.setBaseCollider(0);b.setBaseCollider(0);
	return true;
}
static bool multibody_mixed()
{
	MultiWorld w;btMultiBody mb(0,2,btVector3(1,1,1),false,false);btMultiBodyLinkCollider collider(&mb,-1);
	initBase(w,mb,collider,btVector3(2,0,0));
	btRigidBody* rb=makeBody(w.shape,3,btVector3(0,0,0));w.world.addRigidBody(rb);
	btMultiBodyDistanceConstraint c(btDistanceConstraintEndpoint(rb),btDistanceConstraintEndpoint(&mb,-1),btVector3(0,0,0),btVector3(0,0,0));
	btDistanceConstraintParameters p;p.m_maxDistance=1;p.m_tolerance=0;CHECK(c.setParameters(p).isOk());
	w.world.addMultiBodyConstraint(&c);w.world.stepSimulation(btScalar(.1),0,btScalar(.1));
	CHECK(c.getStatus().isOk()&&mb.getBaseVel().x()<0&&rb->getLinearVelocity().x()>0);
	CHECK(near(mb.getBaseVel().x()*2+rb->getLinearVelocity().x()*3,0,btScalar(.01)));
	w.world.removeMultiBodyConstraint(&c);w.world.removeRigidBody(rb);delete rb;
	w.world.removeCollisionObject(&collider);w.world.removeMultiBody(&mb);mb.setBaseCollider(0);return true;
}
static bool multibody_link()
{
	MultiWorld w;btMultiBody body(1,0,btVector3(0,0,0),true,false);
	body.setupPrismatic(0,2,btVector3(1,1,1),-1,btQuaternion::getIdentity(),btVector3(1,0,0),btVector3(0,0,0),btVector3(0,0,0),true);
	body.finalizeMultiDof();body.setLinearDamping(0);body.setAngularDamping(0);
	btMultiBodyLinkCollider collider(&body,0);collider.setCollisionShape(&w.shape);collider.setWorldTransform(btTransform::getIdentity());body.getLink(0).m_collider=&collider;
	w.world.addMultiBody(&body);w.world.addCollisionObject(&collider,1,0);
	btMultiBodyDistanceConstraint c(btDistanceConstraintEndpoint(&body,0),btDistanceConstraintEndpoint(),btVector3(0,0,0),btVector3(0,0,0));
	btDistanceConstraintParameters p;p.m_flags=BT_DISTANCE_MIN_ENABLED;p.m_minDistance=1;p.m_tolerance=0;CHECK(c.setParameters(p).isOk());
	w.world.addMultiBodyConstraint(&c);w.world.stepSimulation(btScalar(.1),0,btScalar(.1));
	CHECK(c.getStatus().isOk()&&body.getJointVel(0)>0);
	w.world.removeMultiBodyConstraint(&c);w.world.removeCollisionObject(&collider);w.world.removeMultiBody(&body);body.getLink(0).m_collider=0;return true;
}
static bool multibody_same_tree()
{
	MultiWorld w;btMultiBody mb(2,0,btVector3(0,0,0),true,false);
	mb.setupPrismatic(0,2,btVector3(1,1,1),-1,btQuaternion::getIdentity(),btVector3(1,0,0),btVector3(0,0,0),btVector3(0,0,0),true);
	mb.setupPrismatic(1,3,btVector3(1,1,1),-1,btQuaternion::getIdentity(),btVector3(1,0,0),btVector3(0,0,0),btVector3(0,0,0),true);
	mb.finalizeMultiDof();mb.setLinearDamping(0);mb.setAngularDamping(0);
	btMultiBodyLinkCollider ca(&mb,0),cb(&mb,1);ca.setCollisionShape(&w.shape);cb.setCollisionShape(&w.shape);
	ca.setWorldTransform(btTransform::getIdentity());cb.setWorldTransform(btTransform::getIdentity());mb.getLink(0).m_collider=&ca;mb.getLink(1).m_collider=&cb;
	w.world.addMultiBody(&mb);w.world.addCollisionObject(&ca,1,0);w.world.addCollisionObject(&cb,1,0);
	btMultiBodyDistanceConstraint c(btDistanceConstraintEndpoint(&mb,0),btDistanceConstraintEndpoint(&mb,1),btVector3(0,0,0),btVector3(0,0,0));
	btDistanceConstraintParameters p;p.m_flags=BT_DISTANCE_MIN_ENABLED;p.m_minDistance=1;p.m_tolerance=0;CHECK(c.setParameters(p).isOk());
	w.world.addMultiBodyConstraint(&c);w.world.stepSimulation(btScalar(.1),0,btScalar(.1));
	CHECK(c.getStatus().isOk() && mb.getJointVel(0)*mb.getJointVel(1)<0);
	CHECK(near(2*mb.getJointVel(0)+3*mb.getJointVel(1),0,btScalar(.03)));
	w.world.removeMultiBodyConstraint(&c);w.world.removeCollisionObject(&ca);w.world.removeCollisionObject(&cb);w.world.removeMultiBody(&mb);
	mb.getLink(0).m_collider=0;mb.getLink(1).m_collider=0;return true;
}
struct Resolver : btDistanceConstraintResolver
{
	btRigidBody* rb;btMultiBody* mb;btRigidBody* rbOther;
	Resolver(btRigidBody* r=0,btMultiBody* m=0,btRigidBody* r2=0):rb(r),mb(m),rbOther(r2){}
	virtual btConstraintStatus resolve(const btConstraintObjectRef& r,btDistanceConstraintEndpoint& e) const
	{
		if(r.m_id==1&&r.m_domain==BT_CONSTRAINT_RIGID_BODY&&rb)e=btDistanceConstraintEndpoint(rb);
		else if(r.m_id==3&&r.m_domain==BT_CONSTRAINT_RIGID_BODY&&rbOther)e=btDistanceConstraintEndpoint(rbOther);
		else if(r.m_id==2&&r.m_domain==BT_CONSTRAINT_MULTIBODY_LINK&&mb)e=btDistanceConstraintEndpoint(mb,r.m_link);
		else return btConstraintStatus(BT_CONSTRAINT_MISSING_REFERENCE,"test object missing");
		return btConstraintStatus();
	}
};
static btConstraintDescription makeDescription(btDistanceConstraintState& state,uint64_t id,const btConstraintObjectRef& a,const btConstraintObjectRef& b)
{btConstraintDescription d;btConstraintStatus status=state.writeDescription(id,a,b,d);if(!status.isOk())std::fprintf(stderr,"write error %s\n",status.m_message);return d;}
static bool factory()
{
	RigidWorld w;btRigidBody* body=makeBody(w.shape,2,btVector3(2,0,0));w.world.addRigidBody(body);Resolver resolver(body);
	btDistanceConstraintFactory f(w.world,resolver);
	btDistanceConstraint prototype(*body,btVector3(0,0,0),btVector3(0,0,0));
	btDistanceConstraintParameters p;p.m_maxDistance=1;p.m_tolerance=0;CHECK(prototype.setParameters(p).isOk());
	btConstraintDescription d=makeDescription(prototype,5,btConstraintObjectRef(1,BT_CONSTRAINT_RIGID_BODY),btConstraintObjectRef());
	CHECK(f.canCreate(d).isOk() && f.create(d).isOk() && f.find(5));
	CHECK(f.canCreate(d).m_code==BT_CONSTRAINT_DUPLICATE_ID);
	w.world.stepSimulation(btScalar(.1),0,btScalar(.1));CHECK(body->getLinearVelocity().x()<0);
	d.m_common.m_enabled=false;CHECK(f.update(5,d).isOk());CHECK(!f.find(5)->getCommonParameters().m_enabled);
	d.m_common.m_enabled=true;d.m_payload[9]=0;CHECK(f.update(5,d).isOk());
	f.removeObject(btConstraintObjectRef(1,BT_CONSTRAINT_RIGID_BODY));CHECK(!f.find(5));
	w.world.removeRigidBody(body);delete body;return true;
}
static bool factory_mixed()
{
	MultiWorld w;btMultiBody mb(0,2,btVector3(1,1,1),false,false);btMultiBodyLinkCollider collider(&mb,-1);
	initBase(w,mb,collider,btVector3(2,0,0));btRigidBody* rb=makeBody(w.shape,3,btVector3(0,0,0));w.world.addRigidBody(rb);
	Resolver resolver(rb,&mb);btDistanceConstraintFactory factory(w.world,resolver);
	btMultiBodyDistanceConstraint proto(btDistanceConstraintEndpoint(rb),btDistanceConstraintEndpoint(&mb,-1),btVector3(0,0,0),btVector3(0,0,0));
	btDistanceConstraintParameters p;p.m_maxDistance=1;p.m_tolerance=0;CHECK(proto.setParameters(p).isOk());
	btConstraintDescription d=makeDescription(proto,9,btConstraintObjectRef(1,BT_CONSTRAINT_RIGID_BODY),btConstraintObjectRef(2,BT_CONSTRAINT_MULTIBODY_LINK,-1));
	CHECK(factory.create(d).isOk());w.world.stepSimulation(btScalar(.1),0,btScalar(.1));
	CHECK(mb.getBaseVel().x()<0 && rb->getLinearVelocity().x()>0);
	d.m_common.m_enabled=false;CHECK(factory.update(9,d).isOk()&&!factory.find(9)->getCommonParameters().m_enabled);
	factory.removeObject(btConstraintObjectRef(2,BT_CONSTRAINT_MULTIBODY_LINK,-1));CHECK(!factory.find(9));
	w.world.removeRigidBody(rb);delete rb;w.world.removeCollisionObject(&collider);w.world.removeMultiBody(&mb);mb.setBaseCollider(0);return true;
}
static bool factory_pair()
{
	RigidWorld w;btRigidBody* a=makeBody(w.shape,2,btVector3(0,0,0));btRigidBody* b=makeBody(w.shape,3,btVector3(2,0,0));
	w.world.addRigidBody(a);w.world.addRigidBody(b);Resolver resolver(a,0,b);btDistanceConstraintFactory f(w.world,resolver);
	btDistanceConstraint proto(*a,*b,btVector3(0,0,0),btVector3(0,0,0));
	btConstraintDescription d=makeDescription(proto,11,btConstraintObjectRef(1,BT_CONSTRAINT_RIGID_BODY),btConstraintObjectRef(3,BT_CONSTRAINT_RIGID_BODY));
	CHECK(f.create(d).isOk());CHECK(!a->checkCollideWith(b)&&!b->checkCollideWith(a));
	d.m_common.m_collisionEnabled=true;CHECK(f.update(11,d).isOk());CHECK(a->checkCollideWith(b)&&b->checkCollideWith(a));
	d.m_common.m_collisionEnabled=false;CHECK(f.update(11,d).isOk());CHECK(!a->checkCollideWith(b));
	f.remove(11);CHECK(a->checkCollideWith(b)&&b->checkCollideWith(a));
	w.world.removeRigidBody(a);w.world.removeRigidBody(b);delete a;delete b;return true;
}
static bool graph()
{
	RigidWorld w;btRigidBody* body=makeBody(w.shape,2,btVector3(1,0,0));w.world.addRigidBody(body);Resolver resolver(body);
	btDistanceConstraint proto(*body,btVector3(0,0,0),btVector3(0,0,0));
	btDistanceConstraintParameters p;p.m_flags=BT_DISTANCE_MIN_ENABLED|BT_DISTANCE_MAX_ENABLED|BT_DISTANCE_SPRING_ENABLED;
	p.m_minDistance=1;p.m_maxDistance=2;p.m_stiffness=31;p.m_damping=4;CHECK(proto.setParameters(p).isOk());
	btConstraintDescription d=makeDescription(proto,7,btConstraintObjectRef(1,BT_CONSTRAINT_RIGID_BODY),btConstraintObjectRef());
	btConstraintDescriptionGraph graph;graph.m_objects.push_back(d.m_objects[0]);graph.m_objects.push_back(d.m_objects[1]);graph.m_constraints.push_back(d);
	btAlignedObjectArray<unsigned char> bytes;CHECK(btConstraintDescriptionImporter::writeBinary(graph,bytes).isOk());
	btConstraintDescriptionGraph loaded;CHECK(btConstraintDescriptionImporter::readBinary(&bytes[0],bytes.size(),loaded).isOk());
	btDistanceConstraintFactory f(w.world,resolver);CHECK(f.create(loaded.m_constraints[0]).isOk());
	CHECK(near(f.find(7)->getParameters().m_stiffness,31));
	loaded.m_constraints[0].m_payload[0]=2;CHECK(!f.update(7,loaded.m_constraints[0]).isOk());
	f.remove(7);w.world.removeRigidBody(body);delete body;return true;
}
static bool invalid()
{
	RigidWorld w;btRigidBody* body=makeBody(w.shape,2,btVector3(1,0,0));w.world.addRigidBody(body);Resolver resolver(body);
	btDistanceConstraintFactory f(w.world,resolver);btDistanceConstraint proto(*body,btVector3(0,0,0),btVector3(0,0,0));
	btConstraintDescription d=makeDescription(proto,8,btConstraintObjectRef(1,BT_CONSTRAINT_RIGID_BODY),btConstraintObjectRef());
	CHECK(!f.canCreate(btConstraintDescription()).isOk());
	d.m_payload[0]=2;CHECK(!f.canCreate(d).isOk());d.m_payload[0]=1;
	d.m_objects[0]=d.m_objects[1];CHECK(!f.canCreate(d).isOk());d.m_objects[0]=btConstraintObjectRef(1,BT_CONSTRAINT_RIGID_BODY);
	d.m_payload[45]=2;CHECK(!f.canCreate(d).isOk());
	w.world.removeRigidBody(body);delete body;return true;
}
int main(int argc,char** argv)
{
	if(argc!=2)return 2;
	struct Case {const char* name;bool(*run)();} cases[]={{"rows",rows},{"rigid",rigid},{"rigid_modes",rigid_modes},{"rigid_offset",rigid_offset},{"spring",spring},{"break_scale",break_scale},{"zero_response",zero_response},{"multibody",multibody},{"multibody_link",multibody_link},{"multibody_mixed",multibody_mixed},{"multibody_same_tree",multibody_same_tree},{"factory",factory},{"factory_pair",factory_pair},{"factory_mixed",factory_mixed},{"graph",graph},{"invalid",invalid}};
	for(unsigned i=0;i<sizeof(cases)/sizeof(cases[0]);++i)if(std::strcmp(cases[i].name,argv[1])==0)
	{if(!cases[i].run())return 1;std::printf("PASS %s (%s)\n",argv[1],sizeof(btScalar)==8?"double":"float");return 0;}
	return 2;
}
