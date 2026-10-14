# 源码差异、历史更正和实施决策

这些问题已纳入清单，不以缺少默认值或“后续再看”掩盖边界。接口/数学设计阶段需为各项给出明确实现选择与相应测试。

1. D6轴序与articulation轴序不同。D6有8个drive枚举名称而实现复用6个槽；三个angularDriveConfig的合法通道不同，切换会重置角驱动。radial/axis limit与cone/pyramid各有独立启用状态。
2. 普通limit没有旧版contactDistance参数。linearPair hard构造的bounceThreshold源码是2*scale.length，量纲命名容易误读，清单保留原式并要求单位测试。Distance tolerance/k/c setter实际仅校验finite；不要把建议物理范围伪装成现有validator。
3. Articulation limit文档要求low<high，而Np校验接受<=；相等界必须专测。spherical文档的自由度说法与isValidMotion单次检查不能代替完整创建合法性验证。UNDEFINED是未配置状态。
4. 新per-axis摩擦与legacy joint frictionCoefficient不同；初值新三项为0，legacy系数.05。新增参数不能悄悄改变原Bullet对象行为。armature要同时进入动力学与约束响应。
5. Fixed tendon的recipCoefficient独立于coefficient，并非强制倒数。Spatial tendon中间点参与几何，根/叶承担端点力；不能当普通多滑轮绳模拟。
6. Pulley snippet存在语义特殊点：ratio只缩放第二端Jacobian，geometricError仍用未加权长度和；正负误差使用不同单边冲量界，零误差分支没有显式重设这些字段。应保留来源对照，物理一致修正以显式模式或新合同实现。
7. 新attachment描述可识别17组，parseAttachment真正创建11组，明确拒绝6组。旧particle-tet、cone-limited附件接口另列；旧SoftBody/FEM命名是兼容别名，不能重复计数。
8. FEM surface公开USE_ANISOTROPIC_MODEL flag存在，但固定commit全仓库git grep仅命中声明，未发现消费实现；单列为“仅声明”，不从粒子anisotropy.cu借用表面重建算法充数。ENABLE_FLATTENING有真实消费，单独登记。额外实现各向异性材料需独立数学合同。
9. 当前PxFrictionType只有PATCH；PxMaterialFlag没有旧eIMPROVED_PATCH_FRICTION。不能把历史1/2-direction friction开关说成该版本现存类别。
10. 同名Bullet类只表示复用起点；D6 Euler与swing/twist、齿轮速度比与相位约束、rolling friction与torsional patch、soft-body弹簧与FEM能量均不能直接判定等价。
11. 本清单纳入所有物理类别及必要参数支撑，不移植PhysX求解循环、GPU调度、PCM窄相、渲染、profiling、debugger协议。CCD/过滤/车辆整套模型只在与约束输入合同有关的部分纳入；其完整产品功能不因本清单完成而自动具备。
12. 新附件SURFACE_TRI_GLOBAL_POSE与VOLUME_TET_GLOBAL_POSE创建有效，但源码updatePose和removeAttachment分支漏列；Bullet必须处理释放，更新pose若补齐须记录显式扩展。该缺口不能笼统写为“所有rigid/world附件均完整支持动态更新”。
