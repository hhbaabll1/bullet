# 参数、单位、坐标和更新合同

阅读方式：每个 inventory 条目引用一个或多个 profile；本表摘要与 api-contracts.json 的逐声明合同、implementation-contracts.json 的构造/校验原文共同构成参数清单。API只负责声明，实际setter校验与求解消费是不同证据。原文仅作分析证据，保留原来源与版本；后续实际复制代码必须保留对应许可证。

所有拟新增公开类型和默认值必须在后续接口设计阶段落定；本清单不覆盖已有 Bullet 默认行为。只在显式选择新增语义的对象上启用。

## COMMON

**默认/构造：** actor0/1 与 localFrame0/1 由调用者给定；invMassScale 四项=1；breakForce/breakTorque 为 PX_MAX_F32（构造链）；具体 joint flags 见子类。

**合法性/坐标/子模式：** frame 为 actor 局部坐标，构建行前转换到 COM/世界；创建至少一端为 PxRigidBody（含kinematic与articulation link），并不保证该端有非零响应；不可同 actor、不可双 world。质量缩放需 finite>=0；力与力矩断裂分开；禁用、碰撞标志、反馈与 OUTPUT_FORCE 独立。

**单位：** 长度 L、质量 M、时间 T；角度 rad；线速度 L/T，角速度 rad/T，力 M L/T²，力矩 M L²/T²；线冲量 M L/T，角冲量 M L²/T。

**更新时机：** 步间修改参数并使缓存失效；拓扑变更先从 world/scene 移除。

**源码：** [PxJoint.h:38](https://github.com/NVIDIA-Omniverse/PhysX/blob/7845321d31fa3619917ebe127ab5e08e73de0bdb/physx/include/extensions/PxJoint.h#L38), [PxConstraint.h:39](https://github.com/NVIDIA-Omniverse/PhysX/blob/7845321d31fa3619917ebe127ab5e08e73de0bdb/physx/include/PxConstraint.h#L39), [PxConstraintDesc.h:40](https://github.com/NVIDIA-Omniverse/PhysX/blob/7845321d31fa3619917ebe127ab5e08e73de0bdb/physx/include/PxConstraintDesc.h#L40), [ExtJoint.h:53](https://github.com/NVIDIA-Omniverse/PhysX/blob/7845321d31fa3619917ebe127ab5e08e73de0bdb/physx/source/physxextensions/src/ExtJoint.h#L53), [NpConstraint.cpp:39](https://github.com/NVIDIA-Omniverse/PhysX/blob/7845321d31fa3619917ebe127ab5e08e73de0bdb/physx/source/physx/src/NpConstraint.cpp#L39), [ScConstraintCore.cpp:44](https://github.com/NVIDIA-Omniverse/PhysX/blob/7845321d31fa3619917ebe127ab5e08e73de0bdb/physx/source/simulationcontroller/src/ScConstraintCore.cpp#L44)

## LIMIT

**默认/构造：** base: restitution=0,bounceThreshold=0,stiffness=0,damping=0；linearPair lower/upper=±PX_MAX_F32/3，hard ctor bounceThreshold=2*scale.length（源码原式，不能擅改成 speed）；其余具体构造见 API 附录。

**合法性/坐标/子模式：** restitution∈[0,1]；k,c,bounceThreshold finite>=0；isSoft iff k>0 or c>0。linear radius>0；pair lower<=upper；cone 两角∈(0,π)；pyramid 各界∈(-π,π)且下界<=上界。当前版没有旧 contactDistance 字段。

**单位：** linear bound L，angular bound rad；bounceThreshold 对应行速度；force spring k 为力/长度或力矩/rad，damping 为力/速度；acceleration spring 另按响应质量换算。

**更新时机：** 步间修改参数并使缓存失效；拓扑变更先从 world/scene 移除。

**源码：** [PxJointLimit.h:38](https://github.com/NVIDIA-Omniverse/PhysX/blob/7845321d31fa3619917ebe127ab5e08e73de0bdb/physx/include/extensions/PxJointLimit.h#L38), [ExtConstraintHelper.h:40](https://github.com/NVIDIA-Omniverse/PhysX/blob/7845321d31fa3619917ebe127ab5e08e73de0bdb/physx/source/physxextensions/src/ExtConstraintHelper.h#L40)

## REV

**默认/构造：** driveVelocity=0,driveForceLimit=PX_MAX_F32,driveGearRatio=1,limit=[-π/2,π/2],flags=0。

**合法性/坐标/子模式：** LIMIT_ENABLED/DRIVE_ENABLED/DRIVE_FREESPIN；限位严格落在(-2π,2π)；速度方向、gearRatio、freeSpin 的单向冲量条件进入行构建。

**单位：** rad、rad/T、力矩；力限或冲量限经 COMMON 标志与 dt 统一。

**更新时机：** 步间修改参数并使缓存失效；拓扑变更先从 world/scene 移除。

**源码：** [PxRevoluteJoint.h:36](https://github.com/NVIDIA-Omniverse/PhysX/blob/7845321d31fa3619917ebe127ab5e08e73de0bdb/physx/include/extensions/PxRevoluteJoint.h#L36), [ExtRevoluteJoint.cpp:34](https://github.com/NVIDIA-Omniverse/PhysX/blob/7845321d31fa3619917ebe127ab5e08e73de0bdb/physx/source/physxextensions/src/ExtRevoluteJoint.cpp#L34)

## PRISM

**默认/构造：** limit=PxJointLinearLimitPair(scale)，flags=0。

**合法性/坐标/子模式：** 沿 joint X 轴平移；LIMIT_ENABLED 控制是否生成轴向界限行。

**单位：** L、L/T；其余 LIMIT。

**更新时机：** 步间修改参数并使缓存失效；拓扑变更先从 world/scene 移除。

**源码：** [PxPrismaticJoint.h:36](https://github.com/NVIDIA-Omniverse/PhysX/blob/7845321d31fa3619917ebe127ab5e08e73de0bdb/physx/include/extensions/PxPrismaticJoint.h#L36), [ExtPrismaticJoint.cpp:34](https://github.com/NVIDIA-Omniverse/PhysX/blob/7845321d31fa3619917ebe127ab5e08e73de0bdb/physx/source/physxextensions/src/ExtPrismaticJoint.cpp#L34)

## SPHERE

**默认/构造：** cone=(π/2,π/2)，flags=0。

**合法性/坐标/子模式：** 三个位置约束；绕 joint X twist 自由，Y/Z swing 椭圆锥限位；不可直接把 Bullet Euler 限角当同一几何。

**单位：** rad；其余 LIMIT。

**更新时机：** 步间修改参数并使缓存失效；拓扑变更先从 world/scene 移除。

**源码：** [PxSphericalJoint.h:36](https://github.com/NVIDIA-Omniverse/PhysX/blob/7845321d31fa3619917ebe127ab5e08e73de0bdb/physx/include/extensions/PxSphericalJoint.h#L36), [ExtSphericalJoint.cpp:34](https://github.com/NVIDIA-Omniverse/PhysX/blob/7845321d31fa3619917ebe127ab5e08e73de0bdb/physx/source/physxextensions/src/ExtSphericalJoint.cpp#L34)

## DIST

**默认/构造：** min=0,max=0,tolerance=0.025*scale.length,k=0,c=0；仅 MAX_DISTANCE_ENABLED 默认打开。

**合法性/坐标/子模式：** MIN/MAX/SPRING 三标志独立；min/max setter finite>=0；tolerance,k,c setter 实际只校验 finite（文档物理范围与实现区分）；双界相等、交叉界和零距离退化单独测试。

**单位：** distance,tolerance=L；k=M/T²，c=M/T。

**更新时机：** 步间修改参数并使缓存失效；拓扑变更先从 world/scene 移除。

**源码：** [PxDistanceJoint.h:35](https://github.com/NVIDIA-Omniverse/PhysX/blob/7845321d31fa3619917ebe127ab5e08e73de0bdb/physx/include/extensions/PxDistanceJoint.h#L35), [ExtDistanceJoint.cpp:34](https://github.com/NVIDIA-Omniverse/PhysX/blob/7845321d31fa3619917ebe127ab5e08e73de0bdb/physx/source/physxextensions/src/ExtDistanceJoint.cpp#L34)

## D6

**默认/构造：** 6轴全 LOCKED；twist ±π/2；cone π/2；pyramid 四界±π/2；distance=PX_MAX_F32；三linearPair 默认同 LIMIT；四个 useLimit 标志 false；pose identity；两velocity=0；drive k=c=0,forceLimit=PX_MAX_F32,flags=0；angularDriveConfig=LEGACY。

**合法性/坐标/子模式：** 轴顺序 X/Y/Z/TWIST/SWING1/SWING2；每轴 LOCKED/LIMITED/FREE；LEGACY、SWING_TWIST、SLERP 选项影响合法 drive ID 和路由；改 angular config 会重置角驱动参数；8个 drive 枚举不等于8个独立存储槽。distance/axis limits、cone/pyramid 各有独立开关，不能假定互斥；drive target autowake 默认 true。

**单位：** 位置 L、姿态 quaternion，角 rad；target velocities L/T、rad/T；force/acceleration drive 参数单位不同，见 LIMIT/ROW。

**更新时机：** 步间修改参数并使缓存失效；拓扑变更先从 world/scene 移除。

**源码：** [PxD6Joint.h:37](https://github.com/NVIDIA-Omniverse/PhysX/blob/7845321d31fa3619917ebe127ab5e08e73de0bdb/physx/include/extensions/PxD6Joint.h#L37), [ExtD6Joint.cpp:37](https://github.com/NVIDIA-Omniverse/PhysX/blob/7845321d31fa3619917ebe127ab5e08e73de0bdb/physx/source/physxextensions/src/ExtD6Joint.cpp#L37), [ExtD6Joint.h:36](https://github.com/NVIDIA-Omniverse/PhysX/blob/7845321d31fa3619917ebe127ab5e08e73de0bdb/physx/source/physxextensions/src/ExtD6Joint.h#L36)

## GEAR

**默认/构造：** hinges=null，gearRatio=0，error=0，累计角状态重置。

**合法性/坐标/子模式：** 关节引用只接受 Revolute/D6 或 articulation revolute/unwrapped；有符号比率与误差相位；新增连续角状态和引用生命周期。不能只移植 Bullet 现有速度比行。

**单位：** ratio 无量纲，角 rad。

**更新时机：** 步间修改参数并使缓存失效；拓扑变更先从 world/scene 移除。

**源码：** [PxGearJoint.h:35](https://github.com/NVIDIA-Omniverse/PhysX/blob/7845321d31fa3619917ebe127ab5e08e73de0bdb/physx/include/extensions/PxGearJoint.h#L35), [ExtGearJoint.cpp:37](https://github.com/NVIDIA-Omniverse/PhysX/blob/7845321d31fa3619917ebe127ab5e08e73de0bdb/physx/source/physxextensions/src/ExtGearJoint.cpp#L37)

## RACK

**默认/构造：** 引用=null；ratio=1；累计坐标与 resetError 初始化。

**合法性/坐标/子模式：** hinge/prismatic 引用、setRatio 与 setData(nbRackTeeth,nbPinionTeeth,rackLength)；齿数和齿条长度须>0；符号/方向/相位由 prep 规则确定。

**单位：** ratio rad/L 或其等效定义，以源公式为准；rackLength L。

**更新时机：** 步间修改参数并使缓存失效；拓扑变更先从 world/scene 移除。

**源码：** [PxRackAndPinionJoint.h:35](https://github.com/NVIDIA-Omniverse/PhysX/blob/7845321d31fa3619917ebe127ab5e08e73de0bdb/physx/include/extensions/PxRackAndPinionJoint.h#L35), [ExtRackAndPinionJoint.cpp:38](https://github.com/NVIDIA-Omniverse/PhysX/blob/7845321d31fa3619917ebe127ab5e08e73de0bdb/physx/source/physxextensions/src/ExtRackAndPinionJoint.cpp#L38)

## ART

**默认/构造：** drive k=c=maxForce=0,type=NONE；armature=0；per-axis static/dynamic/viscous friction=0；performance envelope 四项=0。关节 core 默认与 API 全声明见附录。 core: type=UNDEFINED，motion全LOCKED，limit=(0,0)，q/v/targetP/targetV=0，legacy frictionCoefficient=.05，maxJointVelocity各轴=100。

**合法性/坐标/子模式：** 轴顺序 TWIST/SWING1/SWING2/X/Y/Z，与 D6 不同；FIX/PRISMATIC/REVOLUTE/REVOLUTE_UNWRAPPED/SPHERICAL；UNDEFINED 是占位值。prismatic 仅一线轴，revolute 仅一角轴；spherical 仅角轴。q 由 Gp*Lp*J=Gc*Lc 定义。spherical 旋转向量，不能直接复用 Bullet quaternion 分量。

**单位：** q:线轴 L/角轴 rad；armature:平动 M/转动 M L²；friction:effort(力/力矩)、viscous effort/velocity；频率 T^-1。

**更新时机：** type/motion/topology 在 scene 外修改；参数需 simulation 之外安全更新。setMotion checked validator 与文档差异见测试。

**源码：** [PxArticulationJointReducedCoordinate.h:37](https://github.com/NVIDIA-Omniverse/PhysX/blob/7845321d31fa3619917ebe127ab5e08e73de0bdb/physx/include/PxArticulationJointReducedCoordinate.h#L37), [PxSolverDefs.h:45](https://github.com/NVIDIA-Omniverse/PhysX/blob/7845321d31fa3619917ebe127ab5e08e73de0bdb/physx/include/solver/PxSolverDefs.h#L45), [PxArticulationFlag.h:36](https://github.com/NVIDIA-Omniverse/PhysX/blob/7845321d31fa3619917ebe127ab5e08e73de0bdb/physx/include/PxArticulationFlag.h#L36), [NpArticulationJointReducedCoordinate.cpp:35](https://github.com/NVIDIA-Omniverse/PhysX/blob/7845321d31fa3619917ebe127ab5e08e73de0bdb/physx/source/physx/src/NpArticulationJointReducedCoordinate.cpp#L35), [DyFeatherstoneArticulation.cpp:59](https://github.com/NVIDIA-Omniverse/PhysX/blob/7845321d31fa3619917ebe127ab5e08e73de0bdb/physx/source/lowleveldynamics/src/DyFeatherstoneArticulation.cpp#L59), [DyArticulationJointCore.h:107](https://github.com/NVIDIA-Omniverse/PhysX/blob/7845321d31fa3619917ebe127ab5e08e73de0bdb/physx/source/lowleveldynamics/include/DyArticulationJointCore.h#L107)

## MIMIC

**默认/构造：** create: naturalFrequency=0,dampingRatio=0；joints/axes/gearRatio/offset 由调用者提供。

**合法性/坐标/子模式：** 同 articulation 的两个不同 DOF；ratio 非零；qA+ratio*qB+offset=0，速度同一梯度；frequency<=0 或 dampingRatio<=0 为硬约束，两者>0 为柔顺约束；不能按普通两个独立刚体响应计算。

**单位：** offset 需与选定广义坐标一致；混合线角时 ratio 带单位；frequency s^-1，dampingRatio 无量纲。

**更新时机：** 步间修改参数并使缓存失效；拓扑变更先从 world/scene 移除。

**源码：** [PxArticulationMimicJoint.h:38](https://github.com/NVIDIA-Omniverse/PhysX/blob/7845321d31fa3619917ebe127ab5e08e73de0bdb/physx/include/PxArticulationMimicJoint.h#L38), [PxArticulationReducedCoordinate.h:45](https://github.com/NVIDIA-Omniverse/PhysX/blob/7845321d31fa3619917ebe127ab5e08e73de0bdb/physx/include/PxArticulationReducedCoordinate.h#L45), [NpArticulationMimicJoint.cpp:35](https://github.com/NVIDIA-Omniverse/PhysX/blob/7845321d31fa3619917ebe127ab5e08e73de0bdb/physx/source/physx/src/NpArticulationMimicJoint.cpp#L35), [DyArticulationMimicJoint.cpp:33](https://github.com/NVIDIA-Omniverse/PhysX/blob/7845321d31fa3619917ebe127ab5e08e73de0bdb/physx/source/lowleveldynamics/src/DyArticulationMimicJoint.cpp#L33)

## TENDON

**默认/构造：** base k=0,c=0,offset=0,limitStiffness=0；fixed lowLimit=PX_MAX_F32,highLimit=-PX_MAX_F32（禁用区间），restLength=0；spatial attachment 默认见其构造初始化证据。

**合法性/坐标/子模式：** fixed 测量 coefficient 与响应 recipCoefficient 独立，可为0；多关节树求和。spatial: link 局部点、根到叶子子腱，端点受力，中间点参与长度几何；不可误写成每中间点都承受滑轮力。拓扑同 articulation，叶节点删除、根节点与 link 关系需校验。

**单位：** spatial 长度 L；fixed 混合 q 时 coefficient 带单位以统一 tendon length；offset/restLength/limits 同 length；刚度与阻尼按合成坐标。

**更新时机：** 步间修改参数并使缓存失效；拓扑变更先从 world/scene 移除。

**源码：** [PxArticulationTendon.h:38](https://github.com/NVIDIA-Omniverse/PhysX/blob/7845321d31fa3619917ebe127ab5e08e73de0bdb/physx/include/PxArticulationTendon.h#L38), [PxArticulationTendonData.h:37](https://github.com/NVIDIA-Omniverse/PhysX/blob/7845321d31fa3619917ebe127ab5e08e73de0bdb/physx/include/PxArticulationTendonData.h#L37), [NpArticulationTendon.cpp:35](https://github.com/NVIDIA-Omniverse/PhysX/blob/7845321d31fa3619917ebe127ab5e08e73de0bdb/physx/source/physx/src/NpArticulationTendon.cpp#L35), [ScArticulationTendonCore.h:35](https://github.com/NVIDIA-Omniverse/PhysX/blob/7845321d31fa3619917ebe127ab5e08e73de0bdb/physx/source/simulationcontroller/include/ScArticulationTendonCore.h#L35), [DyArticulationTendon.h:42](https://github.com/NVIDIA-Omniverse/PhysX/blob/7845321d31fa3619917ebe127ab5e08e73de0bdb/physx/source/lowleveldynamics/include/DyArticulationTendon.h#L42)

## CONTACT

**默认/构造：** 普通材料三基础值由 createMaterial 明确传入；combine 默认 AVERAGE；shape/contact/scene/body 的逐成员 defaults/ranges 见 API 附录和构造证据。

**合法性/坐标/子模式：** normal 单边，切向 static/dynamic Coulomb；PATCH 为该版唯一 friction enum；restitution<0 代表 compliant stiffness；combine 优先级 MAX>MULTIPLY>MIN>AVERAGE；mixed rigid/compliant、force/acceleration compliant 有专门组合规则；target velocity/maxImpulse/接触法线等 callback 覆盖也须消费。

**单位：** penetration/contactOffset/restOffset/torsionalRadius L；restitution/friction 无量纲；negative restitution 的 compliant 解释需映射 stiffness；maxImpulse ML/T。

**更新时机：** 步间修改参数并使缓存失效；拓扑变更先从 world/scene 移除。

**源码：** [PxMaterial.h:35](https://github.com/NVIDIA-Omniverse/PhysX/blob/7845321d31fa3619917ebe127ab5e08e73de0bdb/physx/include/PxMaterial.h#L35), [PxShape.h:39](https://github.com/NVIDIA-Omniverse/PhysX/blob/7845321d31fa3619917ebe127ab5e08e73de0bdb/physx/include/PxShape.h#L39), [PxContactModifyCallback.h:38](https://github.com/NVIDIA-Omniverse/PhysX/blob/7845321d31fa3619917ebe127ab5e08e73de0bdb/physx/include/PxContactModifyCallback.h#L38), [PxSceneDesc.h:43](https://github.com/NVIDIA-Omniverse/PhysX/blob/7845321d31fa3619917ebe127ab5e08e73de0bdb/physx/include/PxSceneDesc.h#L43), [PxRigidBody.h:37](https://github.com/NVIDIA-Omniverse/PhysX/blob/7845321d31fa3619917ebe127ab5e08e73de0bdb/physx/include/PxRigidBody.h#L37), [PxsMaterialCombiner.h:34](https://github.com/NVIDIA-Omniverse/PhysX/blob/7845321d31fa3619917ebe127ab5e08e73de0bdb/physx/source/lowlevel/software/include/PxsMaterialCombiner.h#L34), [DyContactPrep.cpp:36](https://github.com/NVIDIA-Omniverse/PhysX/blob/7845321d31fa3619917ebe127ab5e08e73de0bdb/physx/source/lowleveldynamics/src/DyContactPrep.cpp#L36)

## MOBILITY

**默认/构造：** dynamic lock flags=0；kinematic 和 articulation FIX_BASE/DISABLE_SELF_COLLISION 各为独立配置。

**合法性/坐标/子模式：** 六轴 lock、kinematic target、fixed base、sleep/autowake、质量惯量缩放、速度/穿透纠正上限是约束支撑规则；不能把改 inverse mass 误作完整锁轴。

**单位：** 各轴速度和effort，frame 与 COMMON/ART 一致。

**更新时机：** 步间修改参数并使缓存失效；拓扑变更先从 world/scene 移除。

**源码：** [PxRigidDynamic.h:36](https://github.com/NVIDIA-Omniverse/PhysX/blob/7845321d31fa3619917ebe127ab5e08e73de0bdb/physx/include/PxRigidDynamic.h#L36), [PxRigidBody.h:37](https://github.com/NVIDIA-Omniverse/PhysX/blob/7845321d31fa3619917ebe127ab5e08e73de0bdb/physx/include/PxRigidBody.h#L37), [PxArticulationReducedCoordinate.h:45](https://github.com/NVIDIA-Omniverse/PhysX/blob/7845321d31fa3619917ebe127ab5e08e73de0bdb/physx/include/PxArticulationReducedCoordinate.h#L45), [NpArticulationReducedCoordinate.cpp:44](https://github.com/NVIDIA-Omniverse/PhysX/blob/7845321d31fa3619917ebe127ab5e08e73de0bdb/physx/source/physx/src/NpArticulationReducedCoordinate.cpp#L44)

## ROW

**默认/构造：** Px1DConstraint 由调用方初始化：Jacobian、geometricError、velocityTarget、min/maxImpulse、spring/restitution modifiers、flags、solveHint；没有可假设的全局零初始化契约。

**合法性/坐标/子模式：** SPRING 与 RESTITUTION 互斥；ACCELERATION_SPRING 仅 spring；KEEPBIAS 在 spring 不生效；OUTPUT_FORCE 决定反馈/断裂累加；HAS_DRIVE_LIMIT 配合 dt 与 force/impulse 配置；ANGULAR_CONSTRAINT 标识角行。solveHint 用于预处理而非物理新类别。

**单位：** Jv、位置误差、速度目标、冲量界；力限乘 dt 恰好一次；ERP/CFM 从目标语义推导，不能生搬 PhysX row memory。

**更新时机：** 步间修改参数并使缓存失效；拓扑变更先从 world/scene 移除。

**源码：** [PxConstraintDesc.h:40](https://github.com/NVIDIA-Omniverse/PhysX/blob/7845321d31fa3619917ebe127ab5e08e73de0bdb/physx/include/PxConstraintDesc.h#L40), [PxConstraint.h:39](https://github.com/NVIDIA-Omniverse/PhysX/blob/7845321d31fa3619917ebe127ab5e08e73de0bdb/physx/include/PxConstraint.h#L39), [ExtConstraintHelper.h:40](https://github.com/NVIDIA-Omniverse/PhysX/blob/7845321d31fa3619917ebe127ab5e08e73de0bdb/physx/source/physxextensions/src/ExtConstraintHelper.h#L40), [DyConstraintSetup.cpp:43](https://github.com/NVIDIA-Omniverse/PhysX/blob/7845321d31fa3619917ebe127ab5e08e73de0bdb/physx/source/lowleveldynamics/src/DyConstraintSetup.cpp#L43)

## CUSTOM

**默认/构造：** 自定义常量块由用户定义；PulleyJoint ctor接受body0/1、localFrames和world attachment0/1；distance=1,ratio=1，后续setter修改。

**合法性/坐标/子模式：** solverPrep 产生 scalar rows，connector 管理 actors、外部引用、serialization、release；Pulley 是 custom snippet，不是第9个 PxJointConcreteType；保留绳长比率与单边性/双边性实际源码条件。 此snippet totalDistance=distanceA+distanceB，ratio只缩放第二端Jacobian；这不是完整的比例加权绳长几何。误差正负选择相反单边冲量界，不能描述成始终只受拉的松绳。实现前需在兼容模式与物理一致改进间明确区分。

**单位：** row 单位见 ROW；pulley segment L、ratio 无量纲。

**更新时机：** 步间修改参数并使缓存失效；拓扑变更先从 world/scene 移除。

**源码：** [PxConstraintDesc.h:40](https://github.com/NVIDIA-Omniverse/PhysX/blob/7845321d31fa3619917ebe127ab5e08e73de0bdb/physx/include/PxConstraintDesc.h#L40), [PxConstraint.h:39](https://github.com/NVIDIA-Omniverse/PhysX/blob/7845321d31fa3619917ebe127ab5e08e73de0bdb/physx/include/PxConstraint.h#L39), [PxConstraintExt.h:36](https://github.com/NVIDIA-Omniverse/PhysX/blob/7845321d31fa3619917ebe127ab5e08e73de0bdb/physx/include/extensions/PxConstraintExt.h#L36), [PulleyJoint.cpp:33](https://github.com/NVIDIA-Omniverse/PhysX/blob/7845321d31fa3619917ebe127ab5e08e73de0bdb/physx/snippets/snippetcustomjoint/PulleyJoint.cpp#L33), [PulleyJoint.h:46](https://github.com/NVIDIA-Omniverse/PhysX/blob/7845321d31fa3619917ebe127ab5e08e73de0bdb/physx/snippets/snippetcustomjoint/PulleyJoint.h#L46)

## VEHICLE

**默认/构造：** params/states 由车辆配置与状态更新填充；不读取未初始化 struct；setToDefault/validity 与更新函数见附录。

**合法性/坐标/子模式：** suspensionLimit 方向 SUSPENSION/ROAD_GEOMETRY_NORMAL/NONE；restitution [0,1]；sticky longitudinal/lateral activation 受速度阈值、持续时间与轮接触状态决定。

**单位：** 距离 L、速度 L/T、activation 时间 T、restitution 无量纲。

**更新时机：** 步间修改参数并使缓存失效；拓扑变更先从 world/scene 移除。

**源码：** [PxVehiclePhysXConstraintParams.h:35](https://github.com/NVIDIA-Omniverse/PhysX/blob/7845321d31fa3619917ebe127ab5e08e73de0bdb/physx/include/vehicle2/physxConstraints/PxVehiclePhysXConstraintParams.h#L35), [PxVehiclePhysXConstraintStates.h:42](https://github.com/NVIDIA-Omniverse/PhysX/blob/7845321d31fa3619917ebe127ab5e08e73de0bdb/physx/include/vehicle2/physxConstraints/PxVehiclePhysXConstraintStates.h#L42), [VhPhysXConstraintFunctions.cpp:44](https://github.com/NVIDIA-Omniverse/PhysX/blob/7845321d31fa3619917ebe127ab5e08e73de0bdb/physx/source/physxvehicle/src/physxConstraints/VhPhysXConstraintFunctions.cpp#L44), [VhPhysXConstraintHelpers.cpp:44](https://github.com/NVIDIA-Omniverse/PhysX/blob/7845321d31fa3619917ebe127ab5e08e73de0bdb/physx/source/physxvehicle/src/physxConstraints/VhPhysXConstraintHelpers.cpp#L44)

## PBD

**默认/构造：** material create 参数含调用者值和 signature 默认；per-particle pos+invMass、velocity、phase、rest data 由 buffer 上传；弹簧/充气/cluster 描述不得假定通用默认。详见 API 和实现构造附录。

**合法性/坐标/子模式：** density/granular/spring/inflatable/rigid-cluster 不同物理算子；phase group、fluid/selfCollision/filter、particle mass=0、buffer active counts/dirty flags 独立校验；material 所有 friction/damping/adhesion/cohesion/viscosity/surfaceTension/vorticity/gravity/CFL 参数登记到同一配置层。 shape densityForFluid默认800，文档>0，setter实际只检查finite；粒子-rigid合同必须登记该参数，不能仅凭setter承诺完整浮力模型。

**单位：** 位置/半径 L、invMass M^-1、密度 M/L³；其余以 API 参数定义及对应算子量纲转换。

**更新时机：** 步间修改参数并使缓存失效；拓扑变更先从 world/scene 移除。

**源码：** [PxPBDParticleSystem.h:42](https://github.com/NVIDIA-Omniverse/PhysX/blob/7845321d31fa3619917ebe127ab5e08e73de0bdb/physx/include/PxPBDParticleSystem.h#L42), [PxPBDMaterial.h:36](https://github.com/NVIDIA-Omniverse/PhysX/blob/7845321d31fa3619917ebe127ab5e08e73de0bdb/physx/include/PxPBDMaterial.h#L36), [PxParticleBuffer.h:43](https://github.com/NVIDIA-Omniverse/PhysX/blob/7845321d31fa3619917ebe127ab5e08e73de0bdb/physx/include/PxParticleBuffer.h#L43), [PxParticleSystem.h:36](https://github.com/NVIDIA-Omniverse/PhysX/blob/7845321d31fa3619917ebe127ab5e08e73de0bdb/physx/include/PxParticleSystem.h#L36), [PxParticleSystemFlag.h:36](https://github.com/NVIDIA-Omniverse/PhysX/blob/7845321d31fa3619917ebe127ab5e08e73de0bdb/physx/include/PxParticleSystemFlag.h#L36), [PxParticleClothCooker.h:36](https://github.com/NVIDIA-Omniverse/PhysX/blob/7845321d31fa3619917ebe127ab5e08e73de0bdb/physx/include/extensions/PxParticleClothCooker.h#L36), [particlesystem.cu:73](https://github.com/NVIDIA-Omniverse/PhysX/blob/7845321d31fa3619917ebe127ab5e08e73de0bdb/physx/source/gpusimulationcontroller/src/CUDA/particlesystem.cu#L73), [particleSystem.cuh:40](https://github.com/NVIDIA-Omniverse/PhysX/blob/7845321d31fa3619917ebe127ab5e08e73de0bdb/physx/source/gpusimulationcontroller/src/CUDA/particleSystem.cuh#L40), [ExtParticleClothCooker.cpp:37](https://github.com/NVIDIA-Omniverse/PhysX/blob/7845321d31fa3619917ebe127ab5e08e73de0bdb/physx/source/physxextensions/src/ExtParticleClothCooker.cpp#L37), [NpPBDMaterial.cpp:34](https://github.com/NVIDIA-Omniverse/PhysX/blob/7845321d31fa3619917ebe127ab5e08e73de0bdb/physx/source/physx/src/NpPBDMaterial.cpp#L34), [PxShape.h:459](https://github.com/NVIDIA-Omniverse/PhysX/blob/7845321d31fa3619917ebe127ab5e08e73de0bdb/physx/include/PxShape.h#L459), [NpShape.cpp:637](https://github.com/NVIDIA-Omniverse/PhysX/blob/7845321d31fa3619917ebe127ab5e08e73de0bdb/physx/source/physx/src/NpShape.cpp#L637)

## FEM

**默认/构造：** create material signature: Young/Poisson/friction/thickness/bending 等见 API 精确默认；FEMParameter legacy ctor velocityDamping=.05,settlingThreshold=.1,sleepThreshold=.05,sleepDamping=10,selfCollisionFilterDistance=.1,selfCollisionStressTolerance=.9。

**合法性/坐标/子模式：** surface 膜内三角形应变/三角对弯曲及两种阻尼；ENABLE_FLATTENING 将rest bending angle设0；USE_ANISOTROPIC_MODEL有公开flag但该commit全仓库未找到消费引用，单列仅声明。volume corotational/NeoHookean；模拟网格和碰撞网格映射、质量分配、倒置/退化单元保护、kinematic targets、self contacts 和 attachment 均独立登记。

**单位：** Young 模量 M/(LT²)，Poisson 无量纲，thickness L，体积 L³；bending/damping 转换以能量定义为准，不能等同 Bullet spring 系数。

**更新时机：** 步间修改参数并使缓存失效；拓扑变更先从 world/scene 移除。

**源码：** [PxDeformableBody.h:37](https://github.com/NVIDIA-Omniverse/PhysX/blob/7845321d31fa3619917ebe127ab5e08e73de0bdb/physx/include/PxDeformableBody.h#L37), [PxDeformableMaterial.h:36](https://github.com/NVIDIA-Omniverse/PhysX/blob/7845321d31fa3619917ebe127ab5e08e73de0bdb/physx/include/PxDeformableMaterial.h#L36), [PxDeformableSurface.h:39](https://github.com/NVIDIA-Omniverse/PhysX/blob/7845321d31fa3619917ebe127ab5e08e73de0bdb/physx/include/PxDeformableSurface.h#L39), [PxDeformableSurfaceFlag.h:37](https://github.com/NVIDIA-Omniverse/PhysX/blob/7845321d31fa3619917ebe127ab5e08e73de0bdb/physx/include/PxDeformableSurfaceFlag.h#L37), [PxDeformableSurfaceMaterial.h:35](https://github.com/NVIDIA-Omniverse/PhysX/blob/7845321d31fa3619917ebe127ab5e08e73de0bdb/physx/include/PxDeformableSurfaceMaterial.h#L35), [PxDeformableVolume.h:37](https://github.com/NVIDIA-Omniverse/PhysX/blob/7845321d31fa3619917ebe127ab5e08e73de0bdb/physx/include/PxDeformableVolume.h#L37), [PxDeformableVolumeFlag.h:36](https://github.com/NVIDIA-Omniverse/PhysX/blob/7845321d31fa3619917ebe127ab5e08e73de0bdb/physx/include/PxDeformableVolumeFlag.h#L36), [PxDeformableBodyFlag.h:36](https://github.com/NVIDIA-Omniverse/PhysX/blob/7845321d31fa3619917ebe127ab5e08e73de0bdb/physx/include/PxDeformableBodyFlag.h#L36), [PxDeformableVolumeMaterial.h:35](https://github.com/NVIDIA-Omniverse/PhysX/blob/7845321d31fa3619917ebe127ab5e08e73de0bdb/physx/include/PxDeformableVolumeMaterial.h#L35), [PxFEMParameter.h:37](https://github.com/NVIDIA-Omniverse/PhysX/blob/7845321d31fa3619917ebe127ab5e08e73de0bdb/physx/include/PxFEMParameter.h#L37), [FEMCloth.cu:34](https://github.com/NVIDIA-Omniverse/PhysX/blob/7845321d31fa3619917ebe127ab5e08e73de0bdb/physx/source/gpusimulationcontroller/src/CUDA/FEMCloth.cu#L34), [FEMClothConstraintPrep.cu:50](https://github.com/NVIDIA-Omniverse/PhysX/blob/7845321d31fa3619917ebe127ab5e08e73de0bdb/physx/source/gpusimulationcontroller/src/CUDA/FEMClothConstraintPrep.cu#L50), [FEMClothExternalSolve.cu:55](https://github.com/NVIDIA-Omniverse/PhysX/blob/7845321d31fa3619917ebe127ab5e08e73de0bdb/physx/source/gpusimulationcontroller/src/CUDA/FEMClothExternalSolve.cu#L55), [softBody.cu:65](https://github.com/NVIDIA-Omniverse/PhysX/blob/7845321d31fa3619917ebe127ab5e08e73de0bdb/physx/source/gpusimulationcontroller/src/CUDA/softBody.cu#L65), [softBodyGM.cu:66](https://github.com/NVIDIA-Omniverse/PhysX/blob/7845321d31fa3619917ebe127ab5e08e73de0bdb/physx/source/gpusimulationcontroller/src/CUDA/softBodyGM.cu#L66), [PxgSimulationController.cpp:83](https://github.com/NVIDIA-Omniverse/PhysX/blob/7845321d31fa3619917ebe127ab5e08e73de0bdb/physx/source/gpusimulationcontroller/src/PxgSimulationController.cpp#L83)

## ATTACH

**默认/构造：** actor[2]=null,type[2]=UNDEFINED,pose[2]=identity；indices/coords由PxTypedBoundedData构造，使用时明确提供合法count/stride/data；不能直接创建默认descriptor。

**合法性/坐标/子模式：** 11组创建支持与6组显式拒绝分开；triangle/tet coords 是重心坐标，rigid/world 是 Cartesian；元素数量/索引范围/重心权重/pose 必须校验；deformable-deformable 的 pose 不可动态更新；attachment 与 collision filter 独立管理。

**单位：** Cartesian L，barycentric 无量纲且归一；cone angle rad；重心权重也用于双向冲量/力分配。

**更新时机：** 步间修改参数并使缓存失效；拓扑变更先从 world/scene 移除。

**源码：** [PxDeformableAttachment.h:41](https://github.com/NVIDIA-Omniverse/PhysX/blob/7845321d31fa3619917ebe127ab5e08e73de0bdb/physx/include/PxDeformableAttachment.h#L41), [PxDeformableElementFilter.h:40](https://github.com/NVIDIA-Omniverse/PhysX/blob/7845321d31fa3619917ebe127ab5e08e73de0bdb/physx/include/PxDeformableElementFilter.h#L40), [PxConeLimitedConstraint.h:37](https://github.com/NVIDIA-Omniverse/PhysX/blob/7845321d31fa3619917ebe127ab5e08e73de0bdb/physx/include/PxConeLimitedConstraint.h#L37), [PxDeformableVolume.h:37](https://github.com/NVIDIA-Omniverse/PhysX/blob/7845321d31fa3619917ebe127ab5e08e73de0bdb/physx/include/PxDeformableVolume.h#L37), [NpDeformableAttachment.cpp:45](https://github.com/NVIDIA-Omniverse/PhysX/blob/7845321d31fa3619917ebe127ab5e08e73de0bdb/physx/source/physx/src/NpDeformableAttachment.cpp#L45), [attachments.cuh:34](https://github.com/NVIDIA-Omniverse/PhysX/blob/7845321d31fa3619917ebe127ab5e08e73de0bdb/physx/source/gpusimulationcontroller/src/CUDA/attachments.cuh#L34)
