# PhysX Distance → Bullet：第4步

实现基于固定 PhysX 源码 `7845321d31fa3619917ebe127ab5e08e73de0bdb` 的 `physx/source/physxextensions/src/ExtDistanceJoint.cpp`，保存于 Bullet 的 `feature/physx-constraint-integration`。源代码文件中保留原版权与许可文本。Bullet 原有约束类型及默认路径不变。

## 功能

`btDistanceConstraint` 接入刚体顺序冲量求解，`btMultiBodyDistanceConstraint` 接入 Bullet MultiBody 约束求解；共享参数、几何行生成和保存格式。`btDistanceConstraintFactory` 将应用分配的稳定 ID 解析为世界中现有的端点，并负责注册、更新和先于端点销毁的移除。生产代码仅使用 Bullet 类型。

| PhysX 行为 | Bullet 实现 |
|---|---|
| 构造默认 | min=max=0，tolerance=0.025×长度比例；默认只启用 max；stiffness/damping=0。`btDistanceConstraintParameters(lengthScale)` 显式接受长度比例。 |
| 端点 | RB/RB、RB/world、RB/static/kinematic、MB-link/world、MB-link/RB、同树或跨树 MB-link 对。至少一端为非静态刚体语义；运动学端点可零响应。 |
| 帧 | COM-local、actor-local、world；actor→COM 偏置由 resolver 提供。距离始终取两个世界锚点之间的模。 |
| min/max | 独立启用。只启用一边产生单边行；两边不等时即使未越界也同时产生两行；两边相等产生带容差带的双向一行；均禁用时零行。 |
| 容差 | 与原 `ExtDistanceJoint.cpp` 行误差公式一致；`setTolerance`、stiffness、damping 接受有限负数，因为固定 PhysX setter 仅检查 finite。可能导致奇异响应的参数在生成求解系数时受控报错。 |
| 方向退化 | 两点距离小于 `1.192092896e-7` 时使用世界 X 轴。 |
| spring | 依据固定源码 `DyCpuGpu1dConstraint.h` 的 PGS force-spring 系数，按当前单位响应、`dt²k+dt·d` 构造 Bullet 行 `diag/rhs/cfm`；不调用 PhysX solver。 |
| 响应 | RB 局部质量/惯量缩放只作用于该约束；MB 广义测量和施力分别计算。同树两端的广义响应先相加，再用一棵树的惯量响应，保留交叉项；MB/RB 混合端有双向冲量。 |
| 反馈、断裂 | 本类输出两个端点的线力与关于 COM 的力矩；合成行冲量除以 dt 与 breakForce 比较。距离行在关节锚点没有独立扭矩，所以 breakTorque 不参与本类断裂判定。 |
| 碰撞与生命周期 | Factory 根据 collisionEnabled 管理成对碰撞忽略；删除约束或 `removeObject` 后撤销。卸载对象前必须先从 factory 移除引用它的约束。 |
| 保存/加载 | 分类描述图 payload v1：版本字节、5个 binary64 参数、4字节 flags、broken 字节。公共 BTCXGR01 图格式保存稳定 ID/帧/公共参数。Factory 显式恢复运行约束。 |

普通旧 `.bullet` world importer 不识别新增类型的类别 payload。保存和恢复本扩展须使用公共描述图与应用的对象 resolver；原 Bullet world 文件路径未因此变更。一般的 `btRowDescriptorConstraint` 与 `btMultiBodyRowDescriptorConstraint` 仍保持各自第3步能力边界，Distance 使用自己的类别适配。

PhysX 不同求解器的子步时间、误差处理和数值收敛与 Bullet 不要求逐位一致。`KEEPBIAS` 标记对应的双限位行和几何误差已保留，Bullet 在自身 ERP/冲量迭代中消费；本移植不复制 PhysX 的独立位置/速度求解阶段。负刚度/阻尼虽然符合固定 setter 的 finite 合同，但不保证稳定仿真；零或奇异单位响应的行不产生无效冲量。

## 用法要点

1. 构造端点或实现 `btDistanceConstraintResolver`。可通过 `btDistanceConstraintState::writeDescription` 生成类别 payload；引用必须为应用持久 ID，不能把指针保存为 ID。
2. 设置 `btDistanceConstraintParameters` 的 min/max/flags、tolerance、stiffness/damping，并设置 `btConstraintCommonParameters` 的局部缩放、反馈、碰撞及断裂属性。
3. 直接使用类时由调用者管理 world 注册和对象生命周期。Factory 会注册并提供 `create/update/remove/removeObject/find`；调用者销毁对象前先执行 `removeObject`。
4. 动态更新使用完整 description。对象引用不可在 `update` 中偷换；若要更换端点，先移除再创建。对象变更后重新取参数/状态，不假设旧指针仍可用。

测试文件为 [`test/DistanceConstraint/test_distance_constraint.cpp`](../../../test/DistanceConstraint/test_distance_constraint.cpp)。它验证模式公式、实际刚体与 MultiBody 仿真、同树耦合、混合端、偏心力矩、spring、质量缩放、断裂、碰撞开关、图格式和非法输入。构建/测试与环境信息见 [验证结果](validation.md)。
