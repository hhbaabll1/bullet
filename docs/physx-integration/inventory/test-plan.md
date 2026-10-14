# 验收测试完整登记

本文件是测试设计，不是测试通过报告。tests.json 与 inventory.json 一一关联；所有状态 planned_not_run。

## G-VALIDATION：共用数值与工程矩阵

每个数值用例使用无量纲化单位记录位置/角度/速度/冲量/能量残差；fixtures固定随机种子、初始条件、dt、迭代数、质量惯量和激活flag。单体/单行解析解优先，原PhysX只作行语义或物理特性参考，不要求逐帧轨迹一致。

候选验收门槛：单位尺度的单行代数相对/绝对误差float 1e-5、double 1e-10；稳定小型场景收敛后位置误差<=1e-3 L、角误差<=1e-3 rad；约束违反应随迭代增加下降；被动约束不产生无解释能量，双向作用的总线/角动量误差<=1e-4归一量。复杂FEM/PBD场景使用网格/dt/迭代收敛和独立能量梯度验证，不能套一个关节阈值。上述为后续fixture起点，若更改须记录理由及对应基线数据，不能为掩盖失败随意放宽。

共用取值：float/double；dt=1/30、1/60、1/240；质量比1、1e2、1e4；非identity frame/COM；固定/浮动base；静态/运动学端；正/负/零目标；force/impulse dt；开启/关闭/warmstart清空；禁用/断裂/删除/序列化往返/唤醒；无效NaN/Inf、越界索引、零response均受控处理。

有限组合必须枚举：D6 3^6=729 motion配置 × 三种angular drive config（过滤非法drive）；Distance 2^3=8 flags；Revolute 2^3=8 flags；六轴刚体锁2^6=64；combine优先级4×4=16再叠加材料性质；articulation每joint type的6轴三态合法性（包括拒绝样本）；附件11支持+6拒绝以及两端交换。非有限浮点空间采用边界、符号、退化与尺度代表样本，不宣称穷举。

| 测试ID | 对象profile | 专项验收标准 |
|---|---|---|
| T-J-FIX | O-JOINT | 6个相对自由度残差受控，移动 frame/COM 后一致。 |
| T-J-REV | O-JOINT | 仅绕 joint X 旋转自由，转轴正负和非零 frame 正确。 |
| T-J-PRISM | O-JOINT | 仅 joint X 平移自由，其余5 DOF受限。 |
| T-J-SPHERE | O-JOINT | 球心重合、三角自由；旋转锥不会生成 twist 锁。 |
| T-J-DIST | O-JOINT | 上下界开关、相等界、零距离方向和弹簧连续。 |
| T-J-D6 | O-JOINT | 729种 motion 配置合法性/行数/自由方向符合契约。 |
| T-J-GEAR | O-JOINT | 正负比率及跨圈累计角的速度和相位误差受控。 |
| T-J-RACK | O-JOINT | 旋转和平移比率/方向/相位正确，变更 ratio 重置规则一致。 |
| T-J-REV-LIMIT | O-JOINT | 上下界分别测试 k/c=0 与非零及 bounce velocity 阈值，跨π不反向。 |
| T-J-REV-DRIVE | O-JOINT | 正负目标速度、gearRatio、forceLimit 与 dt 换算符合目标。 |
| T-J-REV-FREESPIN | O-JOINT | 被动超速不制动；反向速度仍受驱动正确加速。 |
| T-J-PRISM-LIMIT | O-JOINT | 上下界、相等界及高速接近界限的启用条件符合 source prep。 |
| T-J-SPHERE-CONE | O-JOINT | 两不同半角、接近π边界、锥角法向与回弹响应正确。 |
| T-J-DIST-MIN | O-JOINT | 仅 min 标志时不限制上方，零距离处理无NaN。 |
| T-J-DIST-MAX | O-JOINT | 仅 max 标志时绳长内自由，外侧拉回。 |
| T-J-DIST-RANGE | O-JOINT | min/max同时启用及相等界，tolerance 内外没有符号跳变。 |
| T-J-DIST-SPRING | O-JOINT | spring 开关、k=0/c>0、阻尼和质量比例行为通过解析一维对照。 |
| T-D6-X-LOCKED | O-JOINT | 纳入全部3^6组合；锁定方向为双边行，自由方向无锁定行，limited 依选定几何界限启用。 |
| T-D6-X-LIMITED | O-JOINT | 纳入全部3^6组合；锁定方向为双边行，自由方向无锁定行，limited 依选定几何界限启用。 |
| T-D6-X-FREE | O-JOINT | 纳入全部3^6组合；锁定方向为双边行，自由方向无锁定行，limited 依选定几何界限启用。 |
| T-D6-Y-LOCKED | O-JOINT | 纳入全部3^6组合；锁定方向为双边行，自由方向无锁定行，limited 依选定几何界限启用。 |
| T-D6-Y-LIMITED | O-JOINT | 纳入全部3^6组合；锁定方向为双边行，自由方向无锁定行，limited 依选定几何界限启用。 |
| T-D6-Y-FREE | O-JOINT | 纳入全部3^6组合；锁定方向为双边行，自由方向无锁定行，limited 依选定几何界限启用。 |
| T-D6-Z-LOCKED | O-JOINT | 纳入全部3^6组合；锁定方向为双边行，自由方向无锁定行，limited 依选定几何界限启用。 |
| T-D6-Z-LIMITED | O-JOINT | 纳入全部3^6组合；锁定方向为双边行，自由方向无锁定行，limited 依选定几何界限启用。 |
| T-D6-Z-FREE | O-JOINT | 纳入全部3^6组合；锁定方向为双边行，自由方向无锁定行，limited 依选定几何界限启用。 |
| T-D6-TWIST-LOCKED | O-JOINT | 纳入全部3^6组合；锁定方向为双边行，自由方向无锁定行，limited 依选定几何界限启用。 |
| T-D6-TWIST-LIMITED | O-JOINT | 纳入全部3^6组合；锁定方向为双边行，自由方向无锁定行，limited 依选定几何界限启用。 |
| T-D6-TWIST-FREE | O-JOINT | 纳入全部3^6组合；锁定方向为双边行，自由方向无锁定行，limited 依选定几何界限启用。 |
| T-D6-SWING1-LOCKED | O-JOINT | 纳入全部3^6组合；锁定方向为双边行，自由方向无锁定行，limited 依选定几何界限启用。 |
| T-D6-SWING1-LIMITED | O-JOINT | 纳入全部3^6组合；锁定方向为双边行，自由方向无锁定行，limited 依选定几何界限启用。 |
| T-D6-SWING1-FREE | O-JOINT | 纳入全部3^6组合；锁定方向为双边行，自由方向无锁定行，limited 依选定几何界限启用。 |
| T-D6-SWING2-LOCKED | O-JOINT | 纳入全部3^6组合；锁定方向为双边行，自由方向无锁定行，limited 依选定几何界限启用。 |
| T-D6-SWING2-LIMITED | O-JOINT | 纳入全部3^6组合；锁定方向为双边行，自由方向无锁定行，limited 依选定几何界限启用。 |
| T-D6-SWING2-FREE | O-JOINT | 纳入全部3^6组合；锁定方向为双边行，自由方向无锁定行，limited 依选定几何界限启用。 |
| T-D6-LIMIT-DISTANCE | O-JOINT | 1/2/3 limited 线轴对应距离维数、近零距离和与 axis limit 共存。 |
| T-D6-LIMIT-LINEAR-PAIR | O-JOINT | 三个axis分别测试 lower/upper/相等界，和径向限位同时开启。 |
| T-D6-LIMIT-TWIST | O-JOINT | 正负角、跨π、extended limits 和上下界法向。 |
| T-D6-LIMIT-CONE | O-JOINT | swing1/2独立状态，锥面法向及退化边界。 |
| T-D6-LIMIT-PYRAMID | O-JOINT | 四界独立、非对称姿态、同时设置 cone 的调用顺序。 |
| T-D6-DRIVE-X | O-JOINT | 三个 angular config × 729 motion 配置过滤合法组合；验证锁轴抑制、SLERP优先级、pose/velocity target、饱和及反馈。 |
| T-D6-DRIVE-Y | O-JOINT | 三个 angular config × 729 motion 配置过滤合法组合；验证锁轴抑制、SLERP优先级、pose/velocity target、饱和及反馈。 |
| T-D6-DRIVE-Z | O-JOINT | 三个 angular config × 729 motion 配置过滤合法组合；验证锁轴抑制、SLERP优先级、pose/velocity target、饱和及反馈。 |
| T-D6-DRIVE-TWIST | O-JOINT | 三个 angular config × 729 motion 配置过滤合法组合；验证锁轴抑制、SLERP优先级、pose/velocity target、饱和及反馈。 |
| T-D6-DRIVE-SWING | O-JOINT | 三个 angular config × 729 motion 配置过滤合法组合；验证锁轴抑制、SLERP优先级、pose/velocity target、饱和及反馈。 |
| T-D6-DRIVE-SLERP | O-JOINT | 三个 angular config × 729 motion 配置过滤合法组合；验证锁轴抑制、SLERP优先级、pose/velocity target、饱和及反馈。 |
| T-D6-DRIVE-SWING1 | O-JOINT | 三个 angular config × 729 motion 配置过滤合法组合；验证锁轴抑制、SLERP优先级、pose/velocity target、饱和及反馈。 |
| T-D6-DRIVE-SWING2 | O-JOINT | 三个 angular config × 729 motion 配置过滤合法组合；验证锁轴抑制、SLERP优先级、pose/velocity target、饱和及反馈。 |
| T-D6-ANGULAR-CONFIG | O-JOINT | 切换配置重置角驱动，非法drive ID受控报错，linear drives不受误伤。 |
| T-D6-ACCELERATION | O-JOINT | 两种质量与惯量比例场景检验 acceleration drive 响应而非固定力弹簧。 |
| T-D6-OUTPUT | O-JOINT | flag清零不累加drive反馈，flag置位力矩/force与断裂阈值一致。 |
| T-D6-TARGET | O-JOINT | 非identity actor/frame，autowake true/false，target velocity坐标转换。 |
| T-A-FIX | O-TREE | 枚举type × 6轴 × motion合法性；父子frame/COM变换、q/v映射、固定/浮动基座均覆盖。 |
| T-A-PRISMATIC | O-TREE | 枚举type × 6轴 × motion合法性；父子frame/COM变换、q/v映射、固定/浮动基座均覆盖。 |
| T-A-REVOLUTE | O-TREE | 枚举type × 6轴 × motion合法性；父子frame/COM变换、q/v映射、固定/浮动基座均覆盖。 |
| T-A-REVOLUTE_UNWRAPPED | O-TREE | 枚举type × 6轴 × motion合法性；父子frame/COM变换、q/v映射、固定/浮动基座均覆盖。 |
| T-A-SPHERICAL | O-TREE | 枚举type × 6轴 × motion合法性；父子frame/COM变换、q/v映射、固定/浮动基座均覆盖。 |
| T-A-AXIS-MOTION | O-TREE | 轴序显式映射；FIX无自由轴；prismatic/revolute最多一匹配轴；spherical角轴组合包括单轴边界验证。 |
| T-A-LIMIT | O-TREE | prismatic/revolute/spherical/unwrapped 界限及 equality 文档/validator差异；每个启用轴验证。 |
| T-A-PREDICTIVE | O-TREE | 到界前预测和已越界纠正分开；高速、小dt、不等惯量；不可用外部关节限位公式推断内部规则。 |
| T-A-DRIVE-FORCE | O-TREE | position+velocity PD、maxForce饱和、force/impulse flag dt换算。 |
| T-A-DRIVE-ACCEL | O-TREE | 不同response mass的跟踪曲线与解析目标符号一致。 |
| T-A-DRIVE-NONE | O-TREE | 关闭后无遗留warmstart驱动力，不影响限位/摩擦。 |
| T-A-FRICTION-STATIC | O-TREE | 低于阈值保持，高于阈值启滑；static>=dynamic。 |
| T-A-FRICTION-DYNAMIC | O-TREE | 正负运动阻力方向正确，零速度切换不增能。 |
| T-A-FRICTION-VISCOUS | O-TREE | effort与joint velocity成比例；线/角单位各自检查。 |
| T-A-FRICTION-LEGACY | O-TREE | 未设置per-axis新参数时旧值生效；已设置轴不会被旧setter覆盖。 |
| T-A-ARMATURE | O-TREE | 零值回归；ABA/response/forward/inverse dynamics采用同一惯量定义。 |
| T-A-VELOCITY | O-TREE | 双方向饱和、各轴独立、legacy precedence。 |
| T-A-ENVELOPE | O-TREE | effort-velocity两条不等式四参数，各零值/非零与drive,maxvelocity叠加。 |
| T-A-STATE | O-TREE | 直接修改状态、目标和cache输入次序，rotvec与quaternion转换不混淆。 |
| T-C-MIMIC-HARD | O-TREE-COUPLED | 正负ratio/offset、同树跨分支、同DOF拒绝；广义坐标残差受控。 |
| T-C-MIMIC-SOFT | O-TREE-COUPLED | frequency/damping两者正才为soft；任一<=0为hard；解析阻尼响应。 |
| T-C-FIXED-LENGTH | O-TREE-COUPLED | 多分支广义长度求和与独立计算一致，混合线角量纲明确。 |
| T-C-FIXED-RESPONSE | O-TREE-COUPLED | coefficient与recipCoefficient不同/为0时，测量和冲量通道分别生效。 |
| T-C-FIXED-LIMIT | O-TREE-COUPLED | restLength/offset/上下界及k/c/limitStiffness独立控制。 |
| T-C-SPATIAL-GEOMETRY | O-TREE-COUPLED | root-leaf各路径长度，局部点移动、branch和系数符号。 |
| T-C-SPATIAL-FORCE | O-TREE-COUPLED | 根/叶受力与response一致；中间几何点不误施滑轮力。 |
| T-C-SPATIAL-LIMIT | O-TREE-COUPLED | 每叶rest/上下限，预张力、松弛、零长度保护。 |
| T-C-LOOP | O-LOOP | 同树环/不同树/树到RB固定点的耦合响应，不双计同树质量。 |
| T-K-NORMAL | O-CONTACT | 法向冲量>=0、非穿透/分离、动态和静态接触路径。 |
| T-K-FRICTION-STATIC | O-CONTACT | 斜面静止阈值、载荷比例和多点patch。 |
| T-K-FRICTION-DYNAMIC | O-CONTACT | 滑动阻力与方向，静动切换和高摩擦。 |
| T-K-RESTITUTION | O-CONTACT | 阈值上/下，0/1，负值进入compliant而非负回弹。 |
| T-K-COMPLIANT-FORCE | O-CONTACT | 负restitution与damping映射，材料混合，弹簧阻尼解析对照。 |
| T-K-COMPLIANT-ACCEL | O-CONTACT | 质量比例对照，mixed acceleration/force material组合。 |
| T-K-COMBINE | O-CONTACT | 4×4优先级和四种数值运算；rigid/compliant与mixed acceleration分支。 |
| T-K-DISABLE-FRICTION | O-CONTACT | 开关后切向响应消失/恢复，无旧warmstart。 |
| T-K-STRONG-FRICTION | O-CONTACT | 持久patch锚误差与滑移阈值；不同接触流形更新。 |
| T-K-TORSIONAL | O-CONTACT | radius/minRadius、点/面接触和角速度，不能直接等价 rolling friction。 |
| T-K-OFFSETS | O-CONTACT | offset/restOffset正负组合、分离预测、几何尺度与高速场景。 |
| T-K-TARGET | O-CONTACT | 传送带切向速度和法向目标，frame正确。 |
| T-K-MODIFY | O-CONTACT | 修改后Jacobian和有效质量同步更新；invalid normal拒绝。 |
| T-K-IMPULSE | O-CONTACT | 0忽略、有限上限、maxContactImpulse组合。 |
| T-K-MASS-SCALE | O-CONTACT | 四缩放独立，不改变原始质量；反馈记录实际施加量。 |
| T-K-CCD | O-CONTACT | 现有Bullet CCD下保持法向/摩擦/回弹合同；离散和speculative路径区别登记。 |
| T-K-FILTER | O-CONTACT | collision solving/contact reporting/trigger分离；过滤不误生成行。 |
| T-M-LOCKS | O-MOBILITY | 六单轴及64组合、惯量旋转后世界锁轴定义，外力和约束响应均一致。 |
| T-M-KINEMATIC | O-MOBILITY | 运动学输入速度推动动态端，inverseMass=0且不接受反冲量。 |
| T-M-BASE | O-MOBILITY | 相同关节场景固定和浮动root响应正确，切换遵守生命周期。 |
| T-M-SELF-COLLISION | O-MOBILITY | 父子过滤与非相邻link开关分别覆盖。 |
| T-M-VELOCITY-CAPS | O-MOBILITY | 逐项上限、角线速度、正负边界、与限位和接触叠加。 |
| T-M-SLEEP | O-MOBILITY | 修改目标/参数、禁用再启用、接触后唤醒和autowake=false。 |
| T-X-CUSTOM | O-JOINT | 自定义1/多行、行数变化、0行、不可满足行、Jacobian/冲量上下界及connector生命周期。 |
| T-X-PULLEY | O-JOINT | ratio=1/2/-1；比较几何误差导数与J；正/负/零误差冲量界；兼容原snippet规则与改进模型需不同配置和测试基准。 |
| T-V-SUSPENSION | O-VEHICLE | 悬架三方向配置、rest/contact状态；sticky低速持续时间/解除条件和正反速度；三个行可独立开关。 |
| T-V-STICKY-LONG | O-VEHICLE | 悬架三方向配置、rest/contact状态；sticky低速持续时间/解除条件和正反速度；三个行可独立开关。 |
| T-V-STICKY-LAT | O-VEHICLE | 悬架三方向配置、rest/contact状态；sticky低速持续时间/解除条件和正反速度；三个行可独立开关。 |
| T-P-DENSITY | O-PARTICLE | 静态液柱密度误差、自由面及压力无无故吸引。 |
| T-P-GRANULAR | O-PARTICLE | 堆积、不同phase、粒径/质量和自碰撞filter。 |
| T-P-SPRING | O-PARTICLE | 拉伸、压缩、剪切/弯曲网络拓扑，restLength和stiffness/damping。 |
| T-P-INFLATABLE | O-PARTICLE | 闭合网格体积、pressure/constraintScale，破洞/退化输入拒绝。 |
| T-P-CLUSTER | O-PARTICLE | 刚体平移/旋转不引入形变能，cluster重叠/权重与退化分布。 |
| T-P-FIXED | O-PARTICLE | inverseMass=0受约束力不移动，目标更新和解除固定。 |
| T-P-MATERIAL | O-PARTICLE | friction/damping/adhesion/gravity/viscosity/vorticity/surfaceTension/cohesion/CFL逐项开关与量纲。 |
| T-P-PHASE | O-PARTICLE | group隔离、fluid flag与selfCollision过滤、rest位置过滤。 |
| T-P-RIGID-CONTACT | O-PARTICLE | 法向/摩擦、双向动量、static/kinematic和link反作用。 |
| T-P-SURFACE-CONTACT | O-PARTICLE | 粒子落在三角面，重心分配、穿透与双向响应。 |
| T-P-VOLUME-CONTACT | O-PARTICLE | 粒子对体碰撞表面响应及仿真mesh映射一致。 |
| T-P-RIGID-ATTACH | O-PARTICLE | particle-rigid固定点、局部坐标、索引复用、双向响应与setRigidFilters独立开关。 |
| T-P-BUFFER | O-PARTICLE | activeCount/容量、buffer销毁、索引复用、GPU到CPU语义数据转换。 |
| T-F-SURFACE-MEMBRANE | O-FEM | 单三角stretch/shear、刚体运动零能、网格收敛。 |
| T-F-SURFACE-BEND | O-FEM | 静止弯角、弯曲方向、thickness/bending stiffness。 |
| T-F-SURFACE-INPLANE-DAMP | O-FEM | 纯刚体运动不耗应变能，形变振动衰减。 |
| T-F-SURFACE-BEND-DAMP | O-FEM | 弯曲模态衰减、不抑制无关刚体运动。 |
| T-F-VOLUME-COROT | O-FEM | 单tet拉伸/剪切/旋转，接近不可压缩Poisson边界。 |
| T-F-VOLUME-NEO | O-FEM | 大形变/能量梯度、倒置/近零体积的保护。 |
| T-F-VOLUME-DAMP | O-FEM | 两阻尼分离、刚体平移/旋转与应变模态。 |
| T-F-KINEMATIC | O-FEM | 局部顶点目标、invMass、释放后的速度一致。 |
| T-F-MESH | O-FEM | 不同网格映射、tet材料索引、碰撞力回传和质量总和。 |
| T-F-SURFACE-CONTACT | O-FEM | 顶点/边/面接触、两面/自接触filter、MB反作用。 |
| T-F-VOLUME-CONTACT | O-FEM | tet碰撞表面、self filter、双向力、不同仿真网格。 |
| T-F-SURFACE-VOLUME | O-FEM | 膜落在软体上，接触重心和双向动量一致。 |
| T-F-FILTER | O-FEM | 禁用指定元素对、销毁恢复、attachment不隐式漏开filter。 |
| T-F-SLEEP | O-FEM | 旧参数到新body setter逐项映射，睡眠/唤醒/自碰撞应力容忍。 |
| T-AT-SURFACE_VTX_RIGID | O-ATTACH | 两端位置残差/权重/双向反作用；valid/invalid索引、重心归一、交换端点、pose更新和销毁。 |
| T-AT-SURFACE_TRI_RIGID | O-ATTACH | 两端位置残差/权重/双向反作用；valid/invalid索引、重心归一、交换端点、pose更新和销毁。 |
| T-AT-SURFACE_VTX_WORLD | O-ATTACH | 两端位置残差/权重/双向反作用；valid/invalid索引、重心归一、交换端点、pose更新和销毁。 |
| T-AT-SURFACE_TRI_WORLD | O-ATTACH | 两端位置残差/权重/双向反作用；valid/invalid索引、重心归一、交换端点、pose更新和销毁。 |
| T-AT-VOLUME_VTX_RIGID | O-ATTACH | 两端位置残差/权重/双向反作用；valid/invalid索引、重心归一、交换端点、pose更新和销毁。 |
| T-AT-VOLUME_TET_RIGID | O-ATTACH | 两端位置残差/权重/双向反作用；valid/invalid索引、重心归一、交换端点、pose更新和销毁。 |
| T-AT-VOLUME_VTX_WORLD | O-ATTACH | 两端位置残差/权重/双向反作用；valid/invalid索引、重心归一、交换端点、pose更新和销毁。 |
| T-AT-VOLUME_TET_WORLD | O-ATTACH | 两端位置残差/权重/双向反作用；valid/invalid索引、重心归一、交换端点、pose更新和销毁。 |
| T-AT-SURFACE_TRI_SURFACE_TRI | O-ATTACH | 两端位置残差/权重/双向反作用；valid/invalid索引、重心归一、交换端点、pose更新和销毁。 |
| T-AT-VOLUME_TET_VOLUME_TET | O-ATTACH | 两端位置残差/权重/双向反作用；valid/invalid索引、重心归一、交换端点、pose更新和销毁。 |
| T-AT-VOLUME_TET_SURFACE_TRI | O-ATTACH | 两端位置残差/权重/双向反作用；valid/invalid索引、重心归一、交换端点、pose更新和销毁。 |
| T-EX-AT-SURFACE_TRI_SURFACE_VTX | O-REJECT | 所有6组创建必须明确报unsupported，不创建半有效对象、无资源泄漏。 |
| T-EX-AT-SURFACE_VTX_SURFACE_VTX | O-REJECT | 所有6组创建必须明确报unsupported，不创建半有效对象、无资源泄漏。 |
| T-EX-AT-VOLUME_TET_VOLUME_VTX | O-REJECT | 所有6组创建必须明确报unsupported，不创建半有效对象、无资源泄漏。 |
| T-EX-AT-VOLUME_TET_SURFACE_VTX | O-REJECT | 所有6组创建必须明确报unsupported，不创建半有效对象、无资源泄漏。 |
| T-EX-AT-VOLUME_VTX_VOLUME_VTX | O-REJECT | 所有6组创建必须明确报unsupported，不创建半有效对象、无资源泄漏。 |
| T-EX-AT-VOLUME_VTX_SURFACE_VTX | O-REJECT | 所有6组创建必须明确报unsupported，不创建半有效对象、无资源泄漏。 |
| T-AT-LEGACY-PARTICLE-TET | O-ATTACH-LEGACY | deprecated入口handle创建/删除、cone参数、索引/坐标和双向响应；别名不重复求解。 |
| T-AT-LEGACY-VERTEX-RIGID | O-ATTACH-LEGACY | deprecated入口handle创建/删除、cone参数、索引/坐标和双向响应；别名不重复求解。 |
| T-AT-LEGACY-TET-RIGID | O-ATTACH-LEGACY | deprecated入口handle创建/删除、cone参数、索引/坐标和双向响应；别名不重复求解。 |
| T-AT-LEGACY-TET-TET | O-ATTACH-LEGACY | deprecated入口handle创建/删除、cone参数、索引/坐标和双向响应；别名不重复求解。 |
| T-R-SPRING | O-ROW | 单标志和互斥/依赖组合的数值行检查；力/冲量/角线单位，预处理前后含义一致。 |
| T-R-ACCELERATION_SPRING | O-ROW | 单标志和互斥/依赖组合的数值行检查；力/冲量/角线单位，预处理前后含义一致。 |
| T-R-RESTITUTION | O-ROW | 单标志和互斥/依赖组合的数值行检查；力/冲量/角线单位，预处理前后含义一致。 |
| T-R-KEEPBIAS | O-ROW | 单标志和互斥/依赖组合的数值行检查；力/冲量/角线单位，预处理前后含义一致。 |
| T-R-OUTPUT_FORCE | O-ROW | 单标志和互斥/依赖组合的数值行检查；力/冲量/角线单位，预处理前后含义一致。 |
| T-R-HAS_DRIVE_LIMIT | O-ROW | 单标志和互斥/依赖组合的数值行检查；力/冲量/角线单位，预处理前后含义一致。 |
| T-R-ANGULAR_CONSTRAINT | O-ROW | 单标志和互斥/依赖组合的数值行检查；力/冲量/角线单位，预处理前后含义一致。 |
| T-H-NONE | O-ROW | 相同物理输入映射为正确行类型；Bullet无该hint时明确不使用它的排序含义。 |
| T-H-SLERP_SPRING | O-ROW | 相同物理输入映射为正确行类型；Bullet无该hint时明确不使用它的排序含义。 |
| T-H-ROTATIONAL_EQUALITY | O-ROW | 相同物理输入映射为正确行类型；Bullet无该hint时明确不使用它的排序含义。 |
| T-H-ROTATIONAL_INEQUALITY | O-ROW | 相同物理输入映射为正确行类型；Bullet无该hint时明确不使用它的排序含义。 |
| T-H-EQUALITY | O-ROW | 相同物理输入映射为正确行类型；Bullet无该hint时明确不使用它的排序含义。 |
| T-H-INEQUALITY | O-ROW | 相同物理输入映射为正确行类型；Bullet无该hint时明确不使用它的排序含义。 |
| T-B-NONE | O-BACKEND | 分类到RB/MB/contact/friction路径，后端无能力时拒绝或明确fallback，不能静默丢行。 |
| T-B-RB_CONTACT | O-BACKEND | 分类到RB/MB/contact/friction路径，后端无能力时拒绝或明确fallback，不能静默丢行。 |
| T-B-RB_1D | O-BACKEND | 分类到RB/MB/contact/friction路径，后端无能力时拒绝或明确fallback，不能静默丢行。 |
| T-B-EXT_CONTACT | O-BACKEND | 分类到RB/MB/contact/friction路径，后端无能力时拒绝或明确fallback，不能静默丢行。 |
| T-B-EXT_1D | O-BACKEND | 分类到RB/MB/contact/friction路径，后端无能力时拒绝或明确fallback，不能静默丢行。 |
| T-B-STATIC_CONTACT | O-BACKEND | 分类到RB/MB/contact/friction路径，后端无能力时拒绝或明确fallback，不能静默丢行。 |
| T-B-BLOCK_RB_CONTACT | O-BACKEND | 分类到RB/MB/contact/friction路径，后端无能力时拒绝或明确fallback，不能静默丢行。 |
| T-B-BLOCK_STATIC_RB_CONTACT | O-BACKEND | 分类到RB/MB/contact/friction路径，后端无能力时拒绝或明确fallback，不能静默丢行。 |
| T-B-BLOCK_1D | O-BACKEND | 分类到RB/MB/contact/friction路径，后端无能力时拒绝或明确fallback，不能静默丢行。 |
| T-B-FRICTION | O-BACKEND | 分类到RB/MB/contact/friction路径，后端无能力时拒绝或明确fallback，不能静默丢行。 |
| T-B-STATIC_FRICTION | O-BACKEND | 分类到RB/MB/contact/friction路径，后端无能力时拒绝或明确fallback，不能静默丢行。 |
| T-B-EXT_FRICTION | O-BACKEND | 分类到RB/MB/contact/friction路径，后端无能力时拒绝或明确fallback，不能静默丢行。 |
| T-B-BLOCK_FRICTION | O-BACKEND | 分类到RB/MB/contact/friction路径，后端无能力时拒绝或明确fallback，不能静默丢行。 |
| T-B-BLOCK_STATIC_FRICTION | O-BACKEND | 分类到RB/MB/contact/friction路径，后端无能力时拒绝或明确fallback，不能静默丢行。 |
| T-S-FLAGS | O-ENGINEERING | broken/collision/disable/visualization/extended limits/SLERP/preprocess/alwaysUpdate/drive unit标志逐项消费或说明无算法等价。 |
| T-S-FRAME | O-ENGINEERING | 非identity COM、相反轴、坐标平移旋转、角unwrap、原点平移。 |
| T-S-LIFETIME | O-ENGINEERING | 释放actor/joint、被引用gear/mimic/tendon、world移除无悬垂引用。 |
| T-S-FEEDBACK | O-ENGINEERING | force/torque阈值分别换算，OUTPUT_FORCE和drive flag，break事件只一次。 |
| T-S-SERIALIZE | O-ENGINEERING | 追加类型ID、float/double、flags/targets/拓扑/持久状态保存；旧Bullet文件仍可读。 |
| T-S-CACHE | O-ENGINEERING | 行增删/重排/参数突变清除失效缓存；zero response无除零。 |
| T-EX-ANISOTROPY | O-REJECT | 粒子表面重建与FEM公开anisotropic flag独立；不能用重建代码冒充FEM算子。 |
| T-F-SURFACE-ANISOTROPIC | O-FEM | 在固定commit全仓库搜索消费点；若仍无实现则明确unsupported，不把设置flag当功能已实现。 |
| T-F-SURFACE-FLATTENING | O-FEM | 同一弯曲rest mesh切换flag，静止角来自rest shape或0，两条初始化路径均验证。 |
| T-EX-OLD-FRICTION | O-REJECT | 当前版API枚举仅PATCH；不会为已删除的eONE_DIRECTIONAL/eTWO_DIRECTIONAL/eIMPROVED_PATCH_FRICTION虚构当前接口。 |
