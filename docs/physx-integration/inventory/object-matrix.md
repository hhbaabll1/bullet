# 对象组合与Bullet支撑接口

required_pairs 是移植后必须实现和验证的组合，不代表 Bullet 现有每个类已经支持。native 描述锁定源码的适用边界；GPU/CPU执行资格与物理类别分开。拒绝项不能通过静默省略约束处理。

## 统一支撑合同

S-COMMON：类型枚举只追加；新增类的构造/默认/访问器/范围检查/dirty标记；actor-local→COM-local→world变换；getInfo1/2或MB行构建；debug draw；单位与dt；禁用/碰撞/断裂/反馈；warmstart重建；sleep/autowake；world增删/引用生命周期；序列化和float/double；构建文件/示例/测试登记。

S-RB：继承 btTypedConstraint，实现 getInfo1/getInfo2，使用现有 btSequentialImpulseConstraintSolver；MLCP仅在行语义与后端能力一致时开放。

S-MB：继承 btMultiBodyConstraint，使用 createConstraintRows/fillMultiBodyConstraint 和广义坐标响应；同树双端和多DOF耦合要合并计算响应，不能按两个无关刚体累加。需要时扩展必要字段与消费者。

S-NONRIGID：在 BulletSoftBody/deformable 体系添加能量、位置、速度、接触、附件算子；参与同一world的RB/MB/FEM/particle反作用一致。GPU数据结构需转成Bullet风格CPU数据层，保留未来后端扩展边界。

## O-JOINT

PhysX边界：普通joint的PxRigidActor两端，可为RB/link/static/kinematic，单端null表示world；创建要求至少一端PxRigidBody，kinematic也满足；gear/rack另有引用类型检查。

待验收组合：RB-RB；RB-world；RB-static；RB-kinematic；RB-MB-link；MB-link-world；MB-link-static；MB-link-kinematic；MB-link-MB-link:same-tree；MB-link-MB-link:cross-tree

拒绝：同actor/同link自身；world-world/static-world/static-static（无PxRigidBody）；普通joint直接连接particle/FEM

Bullet接口：RB使用btTypedConstraint/getInfo1/getInfo2；含link使用btMultiBodyConstraint/createConstraintRows，树内两端必须共享response计算。kinematic-world/static/kinematic可创建但可能零response，求解阶段受控处理；各具体类现有constructor不都覆盖上述组合；这些是待实现目标。

## O-TREE

PhysX边界：articulation父子树；root没有inbound joint；有限合法axis/type组合。

待验收组合：MB-parent-link/child-link:fixed-base；MB-parent-link/child-link:floating-base

拒绝：直接RB-RB；跨树内部joint；particle/FEM节点；UNDEFINED作为可求解类型

Bullet接口：btMultiBody setup*与q/v转换；内部motor/limit/friction，armature需同步ABA/response/FD/ID。

## O-TREE-COUPLED

PhysX边界：同一articulation；mimic两个不同DOF；tendon成员都属于该树并满足拓扑校验。

待验收组合：同树同分支不同DOF；同树跨分支；固定基座；浮动基座

拒绝：跨树mimic/tendon；同DOF自mimic；RB/particle/FEM直接成员

Bullet接口：新增多DOF/多link Jacobian集合与树级响应，生命周期先于普通双体封装。

## O-LOOP

PhysX边界：external joint连接link形成闭环；addLoopJoint用于articulation loop/cache相关API，不能把这个API等同全部外部joint注册。

待验收组合：link-link:same-tree；link-link:cross-tree；link-RB；link-world

拒绝：同link自身；内部树拓扑直接成环

Bullet接口：MultiBody外部约束行；闭环响应与所选求解器一致。

## O-CONTACT

PhysX边界：刚性形状接触，至少一端动态响应；静态/运动学pair的检测与报告受filter控制。

待验收组合：RB-RB；RB-static；RB-kinematic；RB-link；link-static；link-kinematic；link-link:same-tree；link-link:cross-tree

拒绝：world无shape直接生成接触；零response对仍强制求解

Bullet接口：刚体SI接触和MultiBody接触共享材料合同；非刚体接触使用O-FEM/O-PARTICLE。

## O-MOBILITY

PhysX边界：RB和articulation各自运动属性，不混用其flag。

待验收组合：dynamic RB；kinematic RB；static actor；fixed MB；floating MB

拒绝：给static分配受约束自由度

Bullet接口：状态、质量响应、锁轴和唤醒统一。

## O-VEHICLE

PhysX边界：车辆刚体与轮接触状态构建悬架/sticky约束，world/road normal参照。

待验收组合：RB车体-world；RB车体-static road；RB车体-moving road

拒绝：车辆无有效接触仍激活sticky

Bullet接口：btRaycastVehicle扩展状态及独立row provider；MB车体属于Bullet扩展目标，不声称原vehicle模板直接支持。

## O-PARTICLE

PhysX边界：PBD particle buffers，fluid/solid/cloth/inflatable/cluster相位与拓扑；GPU物理路径。

待验收组合：particle-particle；particle-RB；particle-static；particle-kinematic；particle-link；particle-surface；particle-volume

拒绝：不兼容phase/非法buffer索引；无邻域数据却运行density算子

Bullet接口：新增CPU算子/数据层及Bullet非刚体适配；link和FEM反作用为硬验收项。GPU后端标签不要求搬进Bullet。

## O-FEM

PhysX边界：deformable surface/volume；同对象self接触和跨对象接触；volume simulation/collision mesh不同。

待验收组合：surface-RB/static/kinematic/link；volume-RB/static/kinematic/link；surface-surface/self；volume-volume/self；surface-volume；FEM-particle

拒绝：退化/越界网格；把不同mesh索引混用

Bullet接口：现有deformable/soft body框架扩展；与MB共用world时必须双向response。

## O-ATTACH

PhysX边界：严格采用AT-*的11组；rigid可经PxRigidActor处理link，具体后端约束另验。

待验收组合：SURFACE_VTX_RIGID；SURFACE_TRI_RIGID；SURFACE_VTX_WORLD；SURFACE_TRI_WORLD；VOLUME_VTX_RIGID；VOLUME_TET_RIGID；VOLUME_VTX_WORLD；VOLUME_TET_WORLD；SURFACE_TRI_SURFACE_TRI；VOLUME_TET_VOLUME_TET；VOLUME_TET_SURFACE_TRI

拒绝：EX-AT-*全部6组；两个world/rigid无deformable；无效元素/不归一重心/数组长度不符

Bullet接口：vertex anchor可参考现有类；triangle/tet重心到RB/MB及非刚体互连新增数据与响应算子。

## O-ATTACH-LEGACY

PhysX边界：仅deprecated显式入口集合，包含particle-tet及cone参数。

待验收组合：particle-tet；vertex-rigid；tet-rigid；tet-tet

拒绝：无效handle；已释放buffer/actor

Bullet接口：兼容入口转统一附件对象，不重复生成求解行；cone参数按原函数消费。

## O-ROW

PhysX边界：Px1DConstraint标量行，物理类别的中间表达。

待验收组合：RB rows；extended/articulation rows

拒绝：SPRING+RESTITUTION；ACCELERATION_SPRING without SPRING

Bullet接口：RB和MB row builder共同接口；不复制PhysX内存布局。

## O-BACKEND

PhysX边界：RB/EXT/STATIC/BLOCK分派与PGS/TGS/GPU执行路径。

待验收组合：RB SI；MB SI；RB MLCP；nonrigid operators

拒绝：不支持的后端静默丢约束

Bullet接口：明确能力表和fallback；不把BLOCK/STATIC作为新增物理类。

## O-ENGINEERING

PhysX边界：每个物理对象的状态/资源管理。

待验收组合：各覆盖项的全部合法对象组合

拒绝：悬垂引用；过期缓存；未知序列化版本

Bullet接口：所有新类和已有类扩展共同遵守S-COMMON。

## O-REJECT

PhysX边界：源码明确不支持或不属于约束分类的条目。

待验收组合：不适用

拒绝：尝试按当前PhysX已支持能力移植

Bullet接口：受控拒绝/范围说明；不创建虚假实现。
