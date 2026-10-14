# 原Bullet基线核验

基线提交：`a3ebe727e0bc8d4e780e9c8fb90f512769ca4299`；原始核心源码提交：`63c4d67e337017f9d8b298c900e9aabdb69296e7`。

2026-10-13 已从固定提交 git archive 建立独立源码快照，完成 float/double 两套 Release 干净构建。每种配置的 Test_Collision_PASS、Test_btKinematicCharacterController_PASS、Test_BulletDynamics_PASS 共3个 CTest 目标均通过。详细命令、退出状态和日志在本目录。

所选配置：BUILD_BULLET3=OFF，BUILD_UNIT_TESTS=ON，BUILD_SHARED_LIBS=OFF，关闭图形/demo/extras；保留 BulletDynamics、BulletCollision、LinearMath、BulletSoftBody 等核心库。此结果不覆盖未启用的Bullet3/OpenCL、图形示例或PhysX类别测试。

原配置和编译器/平台/版本记录见 initial-state.json；执行记录见 original-checks.json；原构建目录为 `/cnic/work/wangky/bullet3_baseline/2026-10-13/build-original-{float,double}`，源码为同目录 source-bullet-a3ebe727。现有用户 build 与安装测试生成目录未改动。

基线标签：baseline/bullet-original-63c4d67e、baseline/h200-20261013；后续开发分支 feature/physx-constraint-integration。

仓库内日志副本仅统一换行并移除行尾空白；H200 基线目录保留原始日志。
