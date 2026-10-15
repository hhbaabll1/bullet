# Bullet / PhysX约束融合进度

截至2026-10-15，步骤2与步骤3完成；Distance 作为第一种具体 PhysX 约束分类已经实现并验证。开发位于 `feature/physx-constraint-integration`。

- [步骤1基线](baseline/README.md)：固定源码快照；float/double核心库构建及各3个既有测试通过。
- [步骤2完整清单](inventory/README.md)：193条覆盖义务、逐项源码/参数/对象/Bullet落点/测试ID；支持、仅声明及原生拒绝分开登记。
- [步骤3公共接口](public-interfaces.md)：描述、公共参数、坐标/快照、行表示、RB/MB硬行桥接、非刚体operator协议及描述图codec。
- [数据与引用规范](description-format.md)，[验证记录](validation/README.md)。
- [第4步 Distance 距离约束](distance/README.md)：完整分类实现、刚体与 MultiBody 运行适配，以及专项测试。

范围保持：扩展 Bullet 分类和必要支撑函数，保留原有具体类型编号及默认物理行为；继续使用 Bullet 动力学和求解流程。Distance 所需的局部响应、MultiBody 同树与混合端点能力已随类别完成。其余分类按清单逐项移植；公共接口的存在不代表其余物理类别已经完成。
