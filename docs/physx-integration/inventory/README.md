# PhysX约束分类 → Bullet：第2步完整清单

日期：2026-10-13。第2步完成源码清点与验收登记；尚未进行类别移植、引擎构建或数值验收。

| 基准 | 固定版本 |
|---|---|
| Bullet移植起点 | `a3ebe727e0bc8d4e780e9c8fb90f512769ca4299`，`feature/physx-constraint-integration` |
| 原Bullet源代码 | `63c4d67e337017f9d8b298c900e9aabdb69296e7`；工作基线只多安装smoke源和ignore |
| PhysX | `7845321d31fa3619917ebe127ab5e08e73de0bdb`，SDK 5.6.1 |
| 远端原路径 | Bullet `/cnic/work/wangky/bullet3`；PhysX `/cnic/work/wangky/PhysX` |

共 **193条覆盖义务**、**193条专项测试设计**、**20组参数合同**、**1099条API合同摘录**、**194条枚举登记**。总数包括子模式、参数、后端与排除项，不能称为193种独立约束。普通joint仍是8类，articulation内部joint是5类；新附件11组可创建、6组源码拒绝。

推荐阅读顺序：

1. [完整分类/复用/新增矩阵](classification-matrix.md)：逐ID的源码、Bullet对接起点、参数/对象profile、测试ID和差异。
2. [参数合同](parameter-contracts.md)与[对象组合](object-matrix.md)：默认、单位、frames、有效范围、更新时机、RB/MB/非刚体、同树/跨树和生命周期。
3. [测试计划](test-plan.md)：各条目验收条件和有限组合矩阵；[已知差异与范围更正](issues.md)。
4. [求解后端边界](backend-classification.md)：哪些是分类、哪些是行语义/执行分派，及Bullet所需支撑函数。

机器可读主表为[inventory.json](inventory.json)。参数profiles、API声明和实现构造/校验均有源码证据。source-index.json固定commit、path、line、symbol、git blob SHA1，可在H200离线核对来源；外部GitHub源码链接仅方便阅读，验证依据为本地固定Git对象。

实现原则：保留Bullet原有分类/默认/接口，追加PhysX独有类别与模式；以Bullet类风格实现参数及必要求解行/响应/非刚体算子支持；继续使用Bullet动力学与求解流程。拟新增类名是设计起点，尚未创建。

第1步补做记录：原来中断的干净构建与回归已在本次继续工作中补做；固定a3ebe727源码快照的float/double Release构建及每种3个核心CTest测试通过，配置/日志见[基线记录](../baseline/README.md)。这不等于新PhysX类别通过验收。后续实现以每个ID从not_started→implemented→verified流转，不按“类名已经存在”直接验收。

运行清单验证：`python3 validate_inventory.py --physx /cnic/work/wangky/PhysX --bullet /cnic/work/wangky/bullet3`。只验证清单完整关联和固定源码对象，不运行引擎测试。

源码引用按原仓库许可证使用。API/实现摘录的许可证见[来源许可](SOURCE-LICENSES.md)；后续复制或改写源码需保留每个来源文件的具体版权/许可声明。

后续状态：步骤3公共接口已建立，见[公共接口](../public-interfaces.md)。本清单 implementation_status 指每个物理类别/语义的完整落地状态，公共基础设施完成不将这些验收项提前关闭。
