# Distance 约束验证记录

2026-10-15，在 H200（Ubuntu 22.04、aarch64）以原版 Bullet 基线 `63c4d67e337017f9d8b298c900e9aabdb69296e7` 和固定 PhysX 源码 `7845321d31fa3619917ebe127ab5e08e73de0bdb` 为依据，验证本分支的 Distance 移植。所有 Release 构建均先重新配置，再使用 `cmake --build <build-dir> --clean-first -j 8`，避免服务器文件时间戳偏差导致复用旧对象。

| 核验 | 结果 |
|---|---|
| float Release 全部 CTest | 27/27 通过：3 个原 Bullet、8 个公共接口、16 个 Distance 用例 |
| double Release 全部 CTest | 27/27 通过，`USE_DOUBLE_PRECISION=ON` |
| Debug ASan + UBSan | 24/24 公共接口及 Distance 用例通过，启用泄漏检测 |
| 安装后独立调用 | float/double 均安装并通过独立 C++11 程序对公开头文件、Distance 行生成与系数函数的编译、链接、运行 |
| 固定源码清单 | 193 个覆盖项、193 个测试设计、1449 处源码引用、134 个固定 Git 文件验证通过 |
| `git diff --check` | 通过 |

专项测试文件是 [`test_distance_constraint.cpp`](../../../test/DistanceConstraint/test_distance_constraint.cpp)，注册的 16 个测试涵盖最小/最大/区间/等距模式、容差、spring、刚体/世界/静态/运动学、偏心锚点、局部质量缩放、断裂与反馈、MultiBody 基座/连杆/同树/混合端点、工厂生命周期、碰撞开关、描述图往返和非法输入。实际求解仍为 Bullet 顺序冲量或 Bullet MultiBody 路径。

Debug Sanitizer 配置和 Distance/公共接口目标已构建成功。执行不加筛选的 `cmake --build` 时，仓库既有 `Test_Collision` 测试目标在链接阶段报告 `typeinfo for btCapsuleShape` 等未定义引用；该目标与本次 Distance 源码无关，因此 Sanitizer 运行范围限定为 `ctest -R '^(ConstraintInterface_|DistanceConstraint_)'`，结果 24/24。不能把此结果表述为 Sanitizer 下全部 27 个测试通过。

复现命令（以 `/cnic/work/wangky/bullet3` 为源码目录）：

```sh
cmake -S . -B /cnic/work/wangky/bullet3_baseline/2026-10-14-interfaces/build-verified-float -DBUILD_UNIT_TESTS=ON -DBUILD_BULLET2_DEMOS=OFF -DBUILD_EXTRAS=OFF -DBUILD_BULLET3=OFF
cmake --build /cnic/work/wangky/bullet3_baseline/2026-10-14-interfaces/build-verified-float --clean-first -j 8
ctest --test-dir /cnic/work/wangky/bullet3_baseline/2026-10-14-interfaces/build-verified-float --output-on-failure
# double 目录同理，加 -DUSE_DOUBLE_PRECISION=ON
# sanitize 目录使用 Debug、-fsanitize=address,undefined -fno-omit-frame-pointer；
# 运行 ASAN_OPTIONS=detect_leaks=1:halt_on_error=1 UBSAN_OPTIONS=halt_on_error=1
# ctest --test-dir <sanitize-dir> -R '^(ConstraintInterface_|DistanceConstraint_)' --output-on-failure
```

原始配置、构建、CTest 与安装日志保存在 H200 的 `/cnic/work/wangky/bullet3_baseline/2026-10-14-interfaces/distance-*.log`。本目录的三个 CTest 日志副本保存了本次提交所针对的成功测试输出。清单中的 `J-DIST`、`J-DIST-MIN`、`J-DIST-MAX`、`J-DIST-RANGE`、`J-DIST-SPRING` 由本次分类实现覆盖；步骤2的原始清单仍作为固定的移植前快照，其 `planned_not_run` 字段不用于表示当前实现状态。
