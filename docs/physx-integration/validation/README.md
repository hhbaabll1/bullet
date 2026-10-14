# 步骤2、3验证记录

2026-10-14，H200 / aarch64 / GCC 11.4 / CMake 3.22.1。

| 核验 | 结果 |
|---|---|
| 清单结构与固定Git对象 | 193条覆盖项，193条测试设计；1449处引用、134个源码文件的commit/blob/行/符号核验通过 |
| float Release | 11/11：3个原Bullet测试、8个公共接口测试组 |
| double Release | 11/11：3个原Bullet测试、8个公共接口测试组 |
| Debug ASan + UBSan | 8/8接口测试组；启用泄漏检测，没有内存/未定义行为报告 |
| 安装后外部调用 | float/double均完成安装、独立C++11 consumer编译、链接、运行 |

测试组：math、snapshot、graph、rigid_rows、rigid_world、multibody_world、rejected、element。实际动力学覆盖RB/world、MB/world、不同MB树反作用、固定基座prismatic joint DOF；另验零行切换、重复prepare/capture/import、描述图往返与损坏输入、明确拒绝、重心坐标。

系统/NFS存在文件时间戳偏差，曾造成增量构建复用旧对象。最终核验全部使用独立目录和`--clean-first`强制重建；本目录日志是最终执行结果。测试源码与接口源码的SHA256见summary.json，便于核对验证对应的实际文件。

原始基线配置/结果见[baseline](../baseline/README.md)。本次配置使用Ninja，关闭图形/demo/extras及Bullet3，启用核心单元测试；没有执行GPU或完整PhysX类别验收。sanitizer测试单独编译并链接受检测的Bullet依赖库。

复现命令已写入各日志第一行，两个runner脚本也随本目录保存。它们的绝对路径针对本H200工作区；其他机器需要调整根路径。旧失败输出保留在服务器工作目录，本交付记录最终成功结果。

本步明确未开放：同树多端/混合RB-MB广义响应、非对称measurement/response、局部质量缩放、动态界、完整spring/bounce、非刚体物理projector。这些均有描述或接口，当前硬行适配会明确拒绝，后续按步骤4及各类别实施。

仓库内日志副本仅统一换行并移除行尾空白；H200 基线目录保留原始日志。
