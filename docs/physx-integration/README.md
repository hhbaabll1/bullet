# Bullet / PhysX约束融合进度

截至2026-10-14，步骤2与步骤3完成；开发位于 `feature/physx-constraint-integration`。

- [步骤1基线](baseline/README.md)：固定源码快照；float/double核心库构建及各3个既有测试通过。
- [步骤2完整清单](inventory/README.md)：193条覆盖义务、逐项源码/参数/对象/Bullet落点/测试ID；支持、仅声明及原生拒绝分开登记。
- [步骤3公共接口](public-interfaces.md)：描述、公共参数、坐标/快照、行表示、RB/MB硬行桥接、非刚体operator协议及描述图codec。
- [数据与引用规范](description-format.md)，[验证记录](validation/README.md)。

范围保持：扩展Bullet分类和必要支撑函数，保留原有具体类型编号及默认物理行为；继续使用Bullet动力学和求解流程。后续步骤4补响应与处理能力，步骤5–11逐类实现，步骤12全量集成审计。清单中的物理类别尚未因公共接口存在而标记完成。
