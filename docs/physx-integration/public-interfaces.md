# 第3步：公共描述与Bullet接入接口

完成日期：2026-10-14。生产代码仅依赖 Bullet，不包含 PhysX actor、scene、SDK 类型或库。

本步完成可编译的公共描述、参数、坐标读取协议、帧快照、数学辅助函数、描述图保存/加载、RB/MB硬行适配，以及非刚体独立算子接口。具体八类关节、mimic/tendon、接触策略、PBD/FEM算子的完整移植仍按后续步骤实施；分类描述可保存不表示其运行后端已经实现。

## 文件与职责

| 文件 | 已实现职责 |
|---|---|
| `src/BulletDynamics/ConstraintSolver/btConstraintExtension.h/.cpp` | 独立类别ID；6对象域、轴/运动模式；公共/限位/驱动/摩擦/envelope参数；坐标provider、帧快照、角unwrap、frame与冲量换算；测量/响应分离的行；描述图验证、事务加载、二进制codec；运行工厂协议 |
| `src/BulletDynamics/ConstraintSolver/btRowDescriptorConstraint.h/.cpp` | `btTypedConstraint`刚体适配：真实 `getInfo1/getInfo2`，复制并验证帧行数据 |
| `src/BulletDynamics/Featherstone/btMultiBodyRowDescriptorConstraint.h/.cpp` | `btMultiBodyConstraint`适配：真实 `createConstraintRows`，显式广义Jacobian与现有Bullet响应 |
| `src/BulletSoftBody/btConstraintElementOperator.h` | node/triangle/tet/rigid/world点、重心坐标检查、附件参数；独立算子和可批量回写的context接口 |
| `test/ConstraintInterface/` | 8个接口测试组，纳入 `BUILD_UNIT_TESTS`/CTest |

头文件随Bullet正常安装。新增源码纳入BulletDynamics；其C++最低要求为C++11。原有 Bullet 具体约束ID不变；追加 `ROW_DESCRIPTOR_CONSTRAINT_TYPE=13` 和 `MULTIBODY_CONSTRAINT_ROW_DESCRIPTOR=11`，各自MAX哨兵后移。公共描述的 `0x1001..0x8002` 类别ID属于独立schema，禁止强转为原求解器类型枚举。

## 参数与坐标合同

共享轴顺序是 X/Y/Z/TWIST/SWING1/SWING2；PhysX articulation的轴序不同，未来provider/构造器显式映射。参数使用btScalar。长度L、质量M、时间T、角rad；线速度L/T，角速度rad/T；线冲量ML/T，角冲量ML²/T。公开结构使用中性、确定的默认值，不能直接当作每个PhysX类的全部原生默认；后续类别构造器按[参数清单](inventory/parameter-contracts.md)赋值。

`btConstraintObjectRef` 保存应用分配的64位稳定ID、domain、link索引。ID=0仅表示world；MultiBody同一树的各link共用对象ID，base link=-1；禁止保存裸指针作为ID。坐标provider读出 actor/COM 世界变换、线角速度和广义q/v。Bullet刚体的world transform通常是COM变换，actor偏置由provider显式提供。

`btConstraintLocalFrame`明确COM、actor或world空间。所有刚体行的linear/angular Jacobian均为世界基、关于COM测量的速度。广义Jacobian顺序沿Bullet：base角速度3项、base线速度3项、joint DOFs。描述保留测量和施力响应两套Jacobian及各端局部质量/惯量缩放，供fixed tendon等模型使用；当前硬行适配拒绝不同响应或非1缩放，消费这些字段属于第4步。

行符号约定：`q = geometricError`，行计算 `J*v`；ERP模式目标为 `targetVelocity - ERP*q/dt`。适配器不预先再减一次Jv，Bullet求解器执行这个减法。force界乘dt恰好一次变成impulse界；传入impulse不再缩放。上下界是有限btScalar，`SIMD_INFINITY`使用Bullet的大有限哨兵，不接受IEEE NaN/Inf。

spring/restitution/bias、angular/outputForce、owner/row ID、累计冲量/帧号均在描述中保留。spring和restitution不能同时启用；原生硬行适配只接受无spring、无bounce、NO_BIAS/ERP_BIAS、固定bounds、零导入warmstart及对称响应。动态摩擦界、performance envelope消费、加速度式弹簧、逐行反馈和缓存迁移属于第4步。

## 每帧调用流程

1. 应用在安全步间更新参数，完成对象/引用有效性检查。
2. 用 `btConstraintFrameSnapshot::capture(step,dt,objects,provider)` 捕获只读状态。失败不修改旧快照；重复对象、无效frame/数值受控报错。
3. 类别builder根据快照生成行，并在此阶段更新持续角等状态。`btConstraintAngleTracker::prepare` 对同一step相同输入幂等；同step不同角或旧step报错。wrapped角unwrap要求相邻采样真实转角小于π；连续转动的大跨步坐标应直接由provider提供。
4. 对适配器调用 `prepare(frame,rows)` 并检查返回状态；成功后再加入world/执行step。每个接受的step必须递增。失败会禁用已准备行，不能继续把旧行当成功。
5. `getInfo1/getInfo2/createConstraintRows`只读已复制行；反复调用不修改角状态。求解dt不匹配时返回错误状态，不生成有效冲量。应用负责每次实际simulation step重新prepare；旧Bullet回调没有step ID，接口无法自动识别调用者重复模拟却未更新快照。
6. 在对象或拓扑移除前先移除适配器并invalidate；适配器不拥有body。调用者负责对象存活、ID解析、graph到world的注册和移除。

接口用法可直接参考 [测试源](../../test/ConstraintInterface/test_constraint_interface.cpp) 的 `testRigidWorld` 和 `runMultiBody`。其中对质量2、力上限2、dt=0.1的单行实际仿真，速度变化应为0.1；跨树两端反作用分别按质量缩放。

## 当前适配能力

| 输入/语义 | 第3步结果 |
|---|---|
| RB/RB、RB/world，对称硬标量行 | 可运行，现有Bullet SI求解；同一owner和统一反馈开关 |
| MB/world、不同MB树，显式广义硬行 | 可运行；维度校验、固定/浮动响应由Bullet提供 |
| RB/MB混合行、同MB树多端耦合 | 未开放；需第4步响应适配和同树交叉项，不能按两个独立体近似 |
| spring/bounce/KEEPBIAS、局部质量缩放、独立响应、动态bounds、导入累计冲量 | 当前适配明确返回UNSUPPORTED；描述和保存合同保留字段 |
| MB空间力反馈重建 | 未开放，outputForce=true受控拒绝；当前广义行不假装提供完整空间反馈 |
| PBD/FEM/附件 | 公共元素/附件数据及独立context/operator接口已建立；物理projector在第11步实施 |
| MLCP/批处理/多线程 | 本步未验收；以后按能力矩阵开放或明确fallback，不凭继承关系承诺支持 |

非刚体context的 `applyDeltas` 要先验证全部端点，再原子应用一批位置修正/冲量；必须实现重心分配和RB/MB反作用。operator是纯虚接口，不存在默认成功或空project实现。element point的通用校验只验证类型/权重，真实网格索引和topologyGeneration由context检查；11组支持与6组原生拒绝由后续附件factory按清单实施。

## 保存、加载与引用图

详见[二进制与引用规范](description-format.md)。`btConstraintDescriptionImporter`验证全图并事务复制/加载，不创建动力学对象。类别工厂通过 `canCreate/create/remove`明确宣告能力；未知或未实现payload不能静默忽略后创建成功。

新描述图是独立文件协议，不是旧 `.bullet` world快照。RB适配器的旧serializer只写合法的基类元数据和新type ID，避免零字节/空类型记录；不包含行和category payload，旧world importer不能重建新约束。要保存/恢复扩展必须使用描述图与应用的运行对象resolver；完整运行状态/类别payload codec在各类别实现中补齐。旧Bullet对象的序列化路径没有修改。

## 验证范围

清单/原源码验证与接口测试结果集中记录在[本次验证报告](validation/README.md)。接口测试包含格式版本/魔数/截断/非法引用/事务性、单位和坐标、帧重放、硬行实际求解、跨树反作用、明确拒绝和重心权重。它们不代替193项后续物理功能验收。

非刚体坐标补充：rigid/link点的m_localPoint使用COM/link局部坐标，world点使用世界坐标；actor局部点先经公共frame helper转换。triangle/tet由仿真网格重心权重定义，不把局部笛卡尔点混作重心坐标。
