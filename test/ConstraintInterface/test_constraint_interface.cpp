/* New Bullet constraint extension interface tests. Bullet zlib license. */
#include "btBulletDynamicsCommon.h"
#include "BulletDynamics/ConstraintSolver/btConstraintExtension.h"
#include "BulletDynamics/ConstraintSolver/btRowDescriptorConstraint.h"
#include "BulletDynamics/Featherstone/btMultiBodyRowDescriptorConstraint.h"
#include "BulletDynamics/Featherstone/btMultiBodyDynamicsWorld.h"
#include "BulletDynamics/Featherstone/btMultiBodyConstraintSolver.h"
#include "BulletDynamics/Featherstone/btMultiBodyLinkCollider.h"
#include "BulletSoftBody/btConstraintElementOperator.h"
#include <cstdio>
#include <cstring>
#include <limits>
#include <type_traits>

// Do not use assert: Release/NDEBUG must still execute every check.
#define CHECK(condition) do { if (!(condition)) { std::fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #condition); return false; } } while (0)
static bool near(btScalar a, btScalar b, btScalar eps = btScalar(2e-5)) { return btFabs(a-b) <= eps; }

struct Provider : btConstraintCoordinateProvider
{
	mutable int m_reads;
	Provider() : m_reads(0) {}
	virtual btConstraintStatus read(const btConstraintObjectRef&, btConstraintCoordinateState& state) const
	{ ++m_reads; state.m_comWorld.setOrigin(btVector3(1, 2, 3)); state.m_actorWorld.setOrigin(btVector3(4, 5, 6)); state.m_position.push_back(btScalar(0.25)); state.m_velocity.push_back(1); return btConstraintStatus(); }
};
static btConstraintFrameSnapshot frame(const btConstraintObjectRef& a, const btConstraintObjectRef& b, uint64_t step = 1, btScalar dt = btScalar(0.1))
{
	btConstraintFrameSnapshot result; Provider provider; btAlignedObjectArray<btConstraintObjectRef> objects;
	objects.push_back(a); if (!(a == b)) objects.push_back(b);
	result.capture(step, dt, objects, provider); return result;
}
static btConstraintRowDescriptor rigidRow(const btConstraintObjectRef& a, const btConstraintObjectRef& b)
{
	btConstraintRowDescriptor row; row.m_ownerId = 100; row.m_rowId = 1;
	btConstraintJacobianTerm ta, tb; ta.m_object = a; tb.m_object = b;
	ta.m_measurement.m_linear = btVector3(1,0,0); tb.m_measurement.m_linear = btVector3(-1,0,0);
	ta.m_response = ta.m_measurement; tb.m_response = tb.m_measurement;
	row.m_terms.push_back(ta); row.m_terms.push_back(tb); return row;
}
static btConstraintRowDescriptor multiRow(const btConstraintObjectRef& a, const btConstraintObjectRef& b)
{
	btConstraintRowDescriptor row; row.m_ownerId = 200; row.m_rowId = 1;
	btConstraintJacobianTerm ta, tb; ta.m_object = a; tb.m_object = b;
	ta.m_measurement.m_generalized.resize(6, btScalar(0)); ta.m_measurement.m_generalized[3] = 1;
	if (b.m_domain != BT_CONSTRAINT_WORLD) { tb.m_measurement.m_generalized.resize(6, btScalar(0)); tb.m_measurement.m_generalized[3] = -1; }
	ta.m_response = ta.m_measurement; tb.m_response = tb.m_measurement;
	row.m_terms.push_back(ta); row.m_terms.push_back(tb); return row;
}

static bool testMath()
{
	btScalar lo = 7, hi = 9;
	CHECK(btConstraintBoundsToImpulse(-10, 20, BT_CONSTRAINT_FORCE, btScalar(0.02), lo, hi).isOk());
	CHECK(near(lo, btScalar(-0.2)) && near(hi, btScalar(0.4)));
	CHECK(btConstraintBoundsToImpulse(-10, 20, BT_CONSTRAINT_IMPULSE, btScalar(0.02), lo, hi).isOk());
	CHECK(lo == -10 && hi == 20);
	CHECK(!btConstraintBoundsToImpulse(3, 2, BT_CONSTRAINT_FORCE, 1, lo, hi).isOk());
	CHECK(lo == -10 && hi == 20); // failed conversion does not modify outputs
	CHECK(!btConstraintBoundsToImpulse(-1, 1, BT_CONSTRAINT_FORCE, 0, lo, hi).isOk());
	CHECK(!btConstraintBoundsToImpulse(0, std::numeric_limits<btScalar>::max(), BT_CONSTRAINT_FORCE, 2, lo, hi).isOk());
	btConstraintRowDescriptor row; row.m_targetVelocity = 3; row.m_geometricError = btScalar(0.5); row.m_erp = btScalar(0.2); row.m_biasMode = BT_CONSTRAINT_ERP_BIAS;
	CHECK(near(btConstraintTargetVelocity(row, btScalar(0.1)), 2));
	btConstraintCoordinateState state; state.m_comWorld.setOrigin(btVector3(2,0,0)); state.m_actorWorld.setOrigin(btVector3(5,0,0));
	btConstraintLocalFrame local; local.m_transform.setOrigin(btVector3(1,0,0));
	CHECK(btConstraintFrameToWorld(local, state).getOrigin().x() == 3);
	local.m_space = BT_CONSTRAINT_ACTOR_FRAME; CHECK(btConstraintFrameToWorld(local, state).getOrigin().x() == 6);
	local.m_space = BT_CONSTRAINT_WORLD_FRAME; CHECK(btConstraintFrameToWorld(local, state).getOrigin().x() == 1);
	btConstraintAngleTracker tracker;
	CHECK(tracker.prepare(10, SIMD_PI - btScalar(0.1)).isOk());
	CHECK(tracker.prepare(11, -SIMD_PI + btScalar(0.1)).isOk());
	CHECK(near(tracker.getAngle(), SIMD_PI + btScalar(0.1)));
	const btScalar accumulated = tracker.getAngle();
	CHECK(tracker.prepare(11, -SIMD_PI + btScalar(0.1)).isOk() && tracker.getAngle() == accumulated);
	CHECK(!tracker.prepare(11, 0).isOk() && tracker.getAngle() == accumulated);
	CHECK(!tracker.prepare(9, 0).isOk());
	return true;
}

static bool testSnapshot()
{
	Provider provider; btAlignedObjectArray<btConstraintObjectRef> refs;
	refs.push_back(btConstraintObjectRef(1, BT_CONSTRAINT_RIGID_BODY)); refs.push_back(btConstraintObjectRef());
	btConstraintFrameSnapshot snapshot; CHECK(snapshot.capture(3, btScalar(0.01), refs, provider).isOk());
	CHECK(provider.m_reads == 1 && snapshot.m_states.size() == 2); // world is implicit
	CHECK(snapshot.m_states[1].m_comWorld == btTransform::getIdentity());
	CHECK(snapshot.capture(3, btScalar(0.01), refs, provider).isOk()); // replace owned coordinate arrays
	const btConstraintObjectRef duplicate = refs[0]; refs.push_back(duplicate);
	CHECK(!snapshot.capture(4, btScalar(0.01), refs, provider).isOk());
	CHECK(snapshot.m_step == 3 && snapshot.m_objects.size() == 2);
	snapshot.m_states[0].m_comWorld.getBasis()[0][0] = 2;
	CHECK(!btValidateConstraintFrameSnapshot(snapshot).isOk());
	return true;
}

static bool testGraph()
{
	btConstraintDescriptionGraph graph;
	graph.m_objects.push_back(btConstraintObjectRef(7, BT_CONSTRAINT_RIGID_BODY)); graph.m_objects.push_back(btConstraintObjectRef());
	btConstraintDescription a; a.m_id = 10; a.m_category = BT_CONSTRAINT_CATEGORY_D6; a.m_objects = graph.m_objects;
	a.m_frameA.m_transform.setOrigin(btVector3(1,2,3)); a.m_axes[0].m_motion = BT_CONSTRAINT_LIMITED;
	a.m_axes[0].m_limit.m_lower = -2; a.m_axes[0].m_limit.m_upper = 4; a.m_axes[3].m_drive.m_enabled = true;
	a.m_axes[3].m_drive.m_maxEffort = 8; a.m_axes[3].m_drive.m_mode = BT_CONSTRAINT_ACCELERATION_SPRING;
	a.m_axes[3].m_envelope.m_maxActuatorVelocity = 10; a.m_axes[3].m_friction.m_staticEffort = 2;
	a.m_common.m_breakTorque = 12; a.m_common.m_inverseMassScale[1] = btScalar(0.25); a.m_common.m_autoWake = false;
	a.m_payloadVersion = 1; a.m_payload.push_back(0); a.m_payload.push_back(255);
	graph.m_constraints.push_back(a);
	btConstraintDescription b = a; b.m_id = 11; b.m_category = BT_CONSTRAINT_CATEGORY_GEAR; b.m_constraintReferences.push_back(10); graph.m_constraints.push_back(b);
	CHECK(btConstraintDescriptionImporter::validate(graph).isOk());
	btAlignedObjectArray<unsigned char> bytes; CHECK(btConstraintDescriptionImporter::writeBinary(graph, bytes).isOk());
	CHECK(bytes.size() > 32 && std::memcmp(&bytes[0], "BTCXGR01", 8) == 0);
	btConstraintDescriptionGraph loaded; CHECK(btConstraintDescriptionImporter::readBinary(&bytes[0], bytes.size(), loaded).isOk());
	CHECK(btConstraintDescriptionImporter::readBinary(&bytes[0], bytes.size(), loaded).isOk()); // replace a populated graph
	CHECK(loaded.m_constraints.size() == 2 && loaded.m_constraints[1].m_constraintReferences[0] == 10);
	CHECK(loaded.m_constraints[0].m_payload[1] == 255 && loaded.m_constraints[0].m_axes[3].m_drive.m_enabled);
	CHECK(loaded.m_constraints[0].m_common.m_breakTorque == 12 && loaded.m_constraints[0].m_common.m_inverseMassScale[1] == btScalar(0.25) && !loaded.m_constraints[0].m_common.m_autoWake);
	btAlignedObjectArray<unsigned char> again; CHECK(btConstraintDescriptionImporter::writeBinary(loaded, again).isOk());
	CHECK(bytes.size() == again.size() && std::memcmp(&bytes[0], &again[0], bytes.size()) == 0);
	CHECK(!btConstraintDescriptionImporter::readBinary(&bytes[0], bytes.size()-1, loaded).isOk());
	CHECK(loaded.m_constraints.size() == 2); // transactional importer
	bytes[8] = 2; CHECK(!btConstraintDescriptionImporter::readBinary(&bytes[0], bytes.size(), loaded).isOk()); bytes[8] = 1;
	bytes.push_back(0); CHECK(!btConstraintDescriptionImporter::readBinary(&bytes[0], bytes.size(), loaded).isOk());
	CHECK(!btConstraintDescriptionImporter::readBinary(0, 8, loaded).isOk());
	graph.m_constraints[1].m_constraintReferences[0] = 99; CHECK(!btConstraintDescriptionImporter::load(graph, loaded).isOk());
	CHECK(loaded.m_constraints[1].m_constraintReferences[0] == 10);
	graph.m_constraints[1].m_constraintReferences[0] = 10; graph.m_constraints[1].m_id = 10;
	CHECK(!btConstraintDescriptionImporter::validate(graph).isOk());
	return true;
}

static bool testRigidRows()
{
	btSphereShape shape(1); btVector3 inertia; shape.calculateLocalInertia(2, inertia);
	btRigidBody::btRigidBodyConstructionInfo ci(2, 0, &shape, inertia); btRigidBody body(ci);
	btRowDescriptorConstraint constraint(body, 1);
	const btConstraintObjectRef a(1, BT_CONSTRAINT_RIGID_BODY), world;
	btAlignedObjectArray<btConstraintRowDescriptor> rows; btConstraintRowDescriptor row = rigidRow(a, world);
	row.m_targetVelocity = 3; row.m_geometricError = btScalar(0.5); row.m_biasMode = BT_CONSTRAINT_ERP_BIAS;
	row.m_lowerBound = -10; row.m_upperBound = 20; row.m_boundUnit = BT_CONSTRAINT_FORCE; rows.push_back(row);
	btConstraintFrameSnapshot snapshot = frame(a, world);
	CHECK(constraint.prepare(snapshot, rows).isOk());
	btTypedConstraint::btConstraintInfo1 count; constraint.getInfo1(&count); CHECK(count.m_numConstraintRows == 1);
	btScalar j1[4] = {}, a1[4] = {}, j2[4] = {}, a2[4] = {}, rhs[4] = {}, cfm[4] = {}, lo[4] = {}, hi[4] = {};
	btTypedConstraint::btConstraintInfo2 info = {}; info.fps = 10; info.rowskip = 4;
	info.m_J1linearAxis = j1; info.m_J1angularAxis = a1; info.m_J2linearAxis = j2; info.m_J2angularAxis = a2;
	info.m_constraintError = rhs; info.cfm = cfm; info.m_lowerLimit = lo; info.m_upperLimit = hi;
	constraint.getInfo2(&info); CHECK(near(rhs[0],2) && lo[0] == -1 && hi[0] == 2 && j1[0] == 1 && j2[0] == -1);
	rows[0].m_targetVelocity = 100; constraint.getInfo2(&info); CHECK(near(rhs[0],2)); // immutable copy
	constraint.getInfo1(&count); CHECK(count.m_numConstraintRows == 1 && constraint.getStatus().isOk());
	constraint.invalidate(); CHECK(!constraint.prepare(snapshot, rows).isOk()); // cannot replay an already used frame
	snapshot.m_step = 2; CHECK(constraint.prepare(snapshot, rows).isOk());
	info.fps = 20; constraint.getInfo2(&info); CHECK(!constraint.getStatus().isOk() && lo[0] == 0 && hi[0] == 0);
	rows.clear(); snapshot.m_step = 3; CHECK(constraint.prepare(snapshot, rows).isOk());
	constraint.getInfo1(&count); CHECK(count.m_numConstraintRows == 0 && constraint.getStatus().isOk());
	CHECK(POINT2POINT_CONSTRAINT_TYPE == 3 && D6_SPRING_2_CONSTRAINT_TYPE == 12);
	CHECK(constraint.getConstraintType() == ROW_DESCRIPTOR_CONSTRAINT_TYPE);
	return true;
}

static bool testRigidWorld()
{
	btDefaultCollisionConfiguration config; btCollisionDispatcher dispatcher(&config); btDbvtBroadphase broadphase;
	btSequentialImpulseConstraintSolver solver; btDiscreteDynamicsWorld world(&dispatcher, &broadphase, &solver, &config); world.setGravity(btVector3(0,0,0));
	btSphereShape shape(1); btVector3 inertia; shape.calculateLocalInertia(2, inertia);
	btRigidBody::btRigidBodyConstructionInfo ci(2,0,&shape,inertia); btRigidBody body(ci); body.setDamping(0,0); world.addRigidBody(&body);
	btRowDescriptorConstraint constraint(body, 1); btConstraintObjectRef a(1,BT_CONSTRAINT_RIGID_BODY), fixed;
	btAlignedObjectArray<btConstraintRowDescriptor> rows; btConstraintRowDescriptor row = rigidRow(a, fixed);
	row.m_targetVelocity = 10; row.m_lowerBound = -2; row.m_upperBound = 2; row.m_boundUnit = BT_CONSTRAINT_FORCE; rows.push_back(row);
	CHECK(constraint.prepare(frame(a,fixed),rows).isOk()); world.addConstraint(&constraint);
	world.stepSimulation(btScalar(0.1),0,btScalar(0.1));
	const bool ok = near(body.getLinearVelocity().x(),btScalar(0.1)) && constraint.getStatus().isOk();
	world.removeConstraint(&constraint); world.removeRigidBody(&body); CHECK(ok);
	return true;
}

static bool runMultiBody(bool twoTrees)
{
	btDefaultCollisionConfiguration config; btCollisionDispatcher dispatcher(&config); btDbvtBroadphase broadphase;
	btMultiBodyConstraintSolver solver; btMultiBodyDynamicsWorld world(&dispatcher,&broadphase,&solver,&config); world.setGravity(btVector3(0,0,0));
	btMultiBody a(0,2,btVector3(1,1,1),false,false), b(0,3,btVector3(1,1,1),false,false);
	a.finalizeMultiDof(); b.finalizeMultiDof(); a.setLinearDamping(0); a.setAngularDamping(0); b.setLinearDamping(0); b.setAngularDamping(0);
	a.setBaseWorldTransform(btTransform::getIdentity()); b.setBaseWorldTransform(btTransform(btQuaternion::getIdentity(),btVector3(10,0,0)));
	btSphereShape shape(1); btMultiBodyLinkCollider ca(&a,-1), cb(&b,-1); ca.setCollisionShape(&shape); cb.setCollisionShape(&shape);
	ca.setWorldTransform(a.getBaseWorldTransform()); cb.setWorldTransform(b.getBaseWorldTransform()); a.setBaseCollider(&ca); b.setBaseCollider(&cb);
	world.addMultiBody(&a); world.addCollisionObject(&ca,1,0);
	if(twoTrees) { world.addMultiBody(&b); world.addCollisionObject(&cb,1,0); }
	btConstraintObjectRef ra(1,BT_CONSTRAINT_MULTIBODY_LINK), rb = twoTrees ? btConstraintObjectRef(2,BT_CONSTRAINT_MULTIBODY_LINK) : btConstraintObjectRef();
	btMultiBodyRowDescriptorConstraint constraint(&a,-1,1,twoTrees?&b:0,-1,twoTrees?2:0);
	btAlignedObjectArray<btConstraintRowDescriptor> rows; btConstraintRowDescriptor row=multiRow(ra,rb);
	row.m_targetVelocity=10; row.m_lowerBound=-2; row.m_upperBound=2; row.m_boundUnit=BT_CONSTRAINT_FORCE; rows.push_back(row);
	CHECK(constraint.prepare(frame(ra,rb),rows).isOk()); world.addMultiBodyConstraint(&constraint);
	world.stepSimulation(btScalar(0.1),0,btScalar(0.1));
	const bool ok=near(a.getBaseVel().x(),btScalar(0.1)) && (!twoTrees || near(b.getBaseVel().x(),btScalar(-0.2/3))) && constraint.getStatus().isOk();
	if(!ok) std::fprintf(stderr,"MB velocities A=%g B=%g status=%s\n",double(a.getBaseVel().x()),double(b.getBaseVel().x()),constraint.getStatus().m_message);
	world.removeMultiBodyConstraint(&constraint); world.removeCollisionObject(&ca); world.removeMultiBody(&a);
	if(twoTrees) { world.removeCollisionObject(&cb); world.removeMultiBody(&b); }
	a.setBaseCollider(0); b.setBaseCollider(0); CHECK(ok); return true;
}
static bool testMultiBodyJoint()
{
	btDefaultCollisionConfiguration config; btCollisionDispatcher dispatcher(&config); btDbvtBroadphase broadphase;
	btMultiBodyConstraintSolver solver; btMultiBodyDynamicsWorld world(&dispatcher,&broadphase,&solver,&config); world.setGravity(btVector3(0,0,0));
	btMultiBody body(1,0,btVector3(0,0,0),true,false);
	body.setupPrismatic(0,2,btVector3(1,1,1),-1,btQuaternion::getIdentity(),btVector3(1,0,0),btVector3(0,0,0),btVector3(0,0,0),true);
	body.finalizeMultiDof(); body.setLinearDamping(0); body.setAngularDamping(0);
	btSphereShape shape(1); btMultiBodyLinkCollider collider(&body,0); collider.setCollisionShape(&shape); collider.setWorldTransform(btTransform::getIdentity()); body.getLink(0).m_collider=&collider;
	world.addMultiBody(&body); world.addCollisionObject(&collider,1,0);
	btConstraintObjectRef ref(3,BT_CONSTRAINT_MULTIBODY_LINK,0), fixed;
	btConstraintRowDescriptor row=multiRow(ref,fixed); row.m_terms[0].m_measurement.m_generalized[3]=0;
	row.m_terms[0].m_measurement.m_generalized.push_back(1); row.m_terms[0].m_response=row.m_terms[0].m_measurement;
	row.m_targetVelocity=10; row.m_lowerBound=-2; row.m_upperBound=2; row.m_boundUnit=BT_CONSTRAINT_FORCE;
	btAlignedObjectArray<btConstraintRowDescriptor> rows; rows.push_back(row);
	btMultiBodyRowDescriptorConstraint constraint(&body,0,3); CHECK(constraint.prepare(frame(ref,fixed),rows).isOk());
	world.addMultiBodyConstraint(&constraint); world.stepSimulation(btScalar(0.1),0,btScalar(0.1));
	const bool ok=near(body.getJointVel(0),btScalar(0.1)) && constraint.getStatus().isOk();
	world.removeMultiBodyConstraint(&constraint); world.removeCollisionObject(&collider); world.removeMultiBody(&body); body.getLink(0).m_collider=0;
	CHECK(ok); CHECK(constraint.prepare(frame(ref,fixed,2),rows).isOk()); // replace owned generalized rows
	rows.clear(); CHECK(constraint.prepare(frame(ref,fixed,3),rows).isOk() && constraint.getNumRows()==0);
	return true;
}
static bool testMultiBodyWorld() { CHECK(runMultiBody(false)); CHECK(runMultiBody(true)); CHECK(testMultiBodyJoint()); return true; }

static bool testRejected()
{
	btConstraintRowDescriptor row=rigidRow(btConstraintObjectRef(1,BT_CONSTRAINT_RIGID_BODY),btConstraintObjectRef());
	CHECK(btValidateNativeHardConstraintRow(row).isOk());
	row.m_terms[0].m_inverseMassScale=2; CHECK(btValidateNativeHardConstraintRow(row).m_code==BT_CONSTRAINT_UNSUPPORTED);
	row.m_terms[0].m_inverseMassScale=1; row.m_terms[0].m_response.m_linear[0]=2; CHECK(!btValidateNativeHardConstraintRow(row).isOk());
	row.m_terms[0].m_response=row.m_terms[0].m_measurement; row.m_springMode=BT_CONSTRAINT_FORCE_SPRING; row.m_stiffness=5;
	CHECK(btValidateConstraintRow(row).isOk() && btValidateNativeHardConstraintRow(row).m_code==BT_CONSTRAINT_UNSUPPORTED);
	row.m_restitution=btScalar(0.5); CHECK(!btValidateConstraintRow(row).isOk());
	row.m_restitution=0; row.m_springMode=BT_CONSTRAINT_NO_SPRING; row.m_stiffness=0; row.m_dynamicBounds=true; CHECK(!btValidateNativeHardConstraintRow(row).isOk());
	row.m_dynamicBounds=false; row.m_state.m_accumulatedImpulse=1; CHECK(!btValidateNativeHardConstraintRow(row).isOk());
	row.m_state.m_accumulatedImpulse=0; row.m_targetVelocity=std::numeric_limits<btScalar>::quiet_NaN(); CHECK(!btValidateConstraintRow(row).isOk());
	btMultiBody a(0,1,btVector3(1,1,1),false,false); a.finalizeMultiDof();
	btMultiBodyRowDescriptorConstraint sameTree(&a,-1,1,&a,-1,1); btConstraintObjectRef ref(1,BT_CONSTRAINT_MULTIBODY_LINK);
	btAlignedObjectArray<btConstraintRowDescriptor> rows; rows.push_back(multiRow(ref,ref));
	CHECK(sameTree.prepare(frame(ref,ref),rows).m_code==BT_CONSTRAINT_UNSUPPORTED);
	return true;
}

static bool testElement()
{
	static_assert(std::is_abstract<btConstraintElementOperator>::value,"nonrigid operator must provide a real implementation");
	static_assert(std::is_abstract<btConstraintElementContext>::value,"nonrigid context must provide reactions");
	btConstraintElementPoint point; CHECK(point.validate().isOk());
	point.m_object=btConstraintObjectRef(1,BT_CONSTRAINT_SURFACE); point.m_kind=BT_CONSTRAINT_ELEMENT_TRIANGLE; point.m_element=0;
	point.m_weights[0]=btScalar(0.25); point.m_weights[1]=btScalar(0.25); point.m_weights[2]=btScalar(0.5); CHECK(point.validate().isOk());
	point.m_weights[2]=btScalar(0.6); CHECK(!point.validate().isOk()); point.m_weights[2]=btScalar(0.5);
	point.m_weights[3]=btScalar(0.1); CHECK(!point.validate().isOk()); point.m_weights[3]=0;
	point.m_object=btConstraintObjectRef(2,BT_CONSTRAINT_VOLUME); point.m_kind=BT_CONSTRAINT_ELEMENT_TETRAHEDRON;
	for(int i=0;i<4;++i) point.m_weights[i]=btScalar(0.25); CHECK(point.validate().isOk());
	point.m_element=-1; CHECK(!point.validate().isOk());
	return true;
}

int main(int argc,char** argv)
{
	if(argc!=2) { std::fprintf(stderr,"one test case name required\n"); return 2; }
	struct Case { const char* name; bool (*run)(); } cases[]={{"math",testMath},{"snapshot",testSnapshot},{"graph",testGraph},{"rigid_rows",testRigidRows},{"rigid_world",testRigidWorld},{"multibody_world",testMultiBodyWorld},{"rejected",testRejected},{"element",testElement}};
	for(unsigned int i=0;i<sizeof(cases)/sizeof(cases[0]);++i) if(std::strcmp(argv[1],cases[i].name)==0) {
		if(!cases[i].run()) return 1; std::printf("PASS %s (%s)\n",cases[i].name,sizeof(btScalar)==8?"double":"float"); return 0;
	}
	return 2;
}
