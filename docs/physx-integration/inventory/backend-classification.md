# 物理类别、标量行、执行后端的边界

物理类别是 Fixed、Distance、mimic、tendon、contact、density、FEM能量或附件等。D6轴模式和材料规则是其子模式/参数合同。Px1DConstraintFlag 是标量行语义。DySolverConstraintType 的 RB/EXT/STATIC/BLOCK 是对象和布局分派。这四层不能合并成一个 Bullet 约束枚举。

| 路径 | 复用/扩展位置 | 移植后的开放条件 |
|---|---|---|
| RB joint/contact/friction | btTypedConstraint + SequentialImpulse | 新参数确实参与行构建、反馈、上下界 |
| 含MB-link的joint/contact | btMultiBodyConstraintSolver | 广义坐标响应、同树相关性、固定/浮动base正确 |
| 多DOF mimic/tendon | MultiBody数据/行提供者/response | 多端Jacobian、状态和生命周期实现后开放 |
| RB MLCP | btMLCPSolver | 能表达同一行合同才开放，不能将所有SI新字段默认为支持 |
| 粒子/PBD/FEM/附件 | BulletSoftBody/deformable operators/world | 能量/密度/体积/重心耦合有明确消费者 |
| PhysX PGS/TGS/GPU/BLOCK | 只记录来源 | 不移植求解调度/布局；物理语义在Bullet实现 |

目前“开放条件”均为待实施，未通过引擎测试。新类getInfo2或createConstraintRows不够表达的功能必须补相应模块；不得空实现、忽略flag、或返回0行后标记支持。用户选到不支持后端时明确报错，或仅在明确记录且物理语义相同的情况下fallback。
