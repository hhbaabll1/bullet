# 公共描述格式与引用图 v1

此格式保存公共描述及类别模块的opaque payload；它不保存指针、PhysX对象、GPU资源或Bullet内部ABI布局。

## 稳定编号与扩展

类别分组：0x1001–0x1008普通八类关节，0x2001–0x2005 articulation/轴限位/驱动/摩擦/速度限位，0x3001–0x3004 mimic/tendons/loop，0x4001–0x4004接触/摩擦/torsion/mobility，0x5001–0x5004 custom/pulley/vehicle，0x6001–0x6006 particle/cloth/volume/cluster/node，0x7001–0x7003 FEM，0x8001–0x8002附件和跨域接触。确切名称见公共头；已有ID不可重排或复用。

对象域0–5依次world、rigid body、MultiBody link、particle、surface、volume。对象ID由应用分配，world是0；对象ID与constraint ID属于独立名字空间。同一MultiBody对象允许不同link引用，不能让相同对象ID对应不同domain。constraint references按ID解析，全图读取后再验证，允许用于闭环描述的引用图；创建顺序和类别特有的禁止自引用/拓扑检查由运行factory处理。

## 字节布局

所有整数槽为小端uint64；bool只能0/1；标量为IEEE-754 binary64，以`memcpy`转位模式，不reinterpret未对齐指针。link=-1编码UINT64_MAX，其余为非负32位索引。float读取不能表示的数值时拒绝，不截断或饱和。实际结构的padding和btScalar编译宽度不进入文件。

顺序：

1. 8字节ASCII魔数 `BTCXGR01`，u64 schemaVersion=1，u64 objectCount，u64 constraintCount。
2. objectCount个 ObjectRef：u64 id，u64 domain，u64 link。
3. 每个ConstraintDescription：u64 id，u64 category；u64 objectRefCount及ObjectRef数组；u64 constraintRefCount及u64引用ID数组。
4. frameA、frameB：各含u64 frameSpace、3个origin标量、按行排列的3×3 basis标量。
5. common：4个bool(enabled/collisionEnabled/outputForce/autoWake)，breakForce、breakTorque两个标量；对A、B依次写inverseMassScale、inverseInertiaScale。
6. 六轴（X/Y/Z/TWIST/SWING1/SWING2），每轴28个8字节槽：motion；limit lower/upper/restitution/bounceThreshold/stiffness/damping；drive position/velocity/stiffness/damping/maxEffort、springMode、effortUnit、enabled/freeSpin/outputForce；friction static/dynamic/viscous/legacy、useLegacy；envelope四参数；armature/maxVelocity。
7. u64 payloadVersion、u64 payloadByteCount及原始payload字节。非空payload必须有非零版本；具体类别模块负责解释和拒绝未知版本。

末尾不得有未消费字节。最多4096个顶层对象和4096个约束；单约束引用数最多4096，单payload最多1MiB，全图上限16MiB。越界计数、非法枚举/数值、非刚性frame矩阵、重复ID、缺失引用或截断均返回明确状态；destination保持原值。

改变字段顺序、单位或意义必须增加schemaVersion。`readBinary`当前只接受v1，不猜测旧版本；payloadVersion由类别独立维护。新增类别ID需要同步公共枚举、validator、factory与清单。不能把“成功解析通用payload字节”描述为“已经支持其物理功能”。

## 运行状态边界

row ownerId/rowId/accumulatedImpulse/lastStep是运行行合同；图codec保存构造描述，尚不保存每次求解的行缓存。后续类别模块要定义持续角、摩擦状态、腱拓扑及warmstart版本并放入明确的payload/状态记录，且由factory验证。当前适配器拒绝非零导入累计冲量，新的帧准备会清理MB缓存。对象销毁顺序：移除引用它的约束/operator，移出world，最后释放对象。
