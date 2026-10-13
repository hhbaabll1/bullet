cmake_minimum_required(VERSION 3.10)
project(BulletTestProject)

set(CMAKE_CXX_STANDARD 11)

# ================= 强制指定具体路径 =================

# 1. 指定头文件所在的具体目录
set(BULLET_INCLUDE_DIRS "/cnic/work/wangky/bullet3_install/include/bullet")

# 2. 定义库文件所在的目录（方便下面引用）
set(BULLET_LIB_DIR "/cnic/work/wangky/bullet3_install/lib")

# 3. 指定具体的库文件绝对路径（注意链接顺序：Dynamics 依赖 Collision 依赖 LinearMath）
# 如果你之前编译的是动态库（SHARED），后缀是 .so；如果是静态库（STATIC），后缀是 .a
set(BULLET_LIBRARIES 
    ${BULLET_LIB_DIR}/libBulletDynamics.so
    ${BULLET_LIB_DIR}/libBulletCollision.so
    ${BULLET_LIB_DIR}/libLinearMath.so
)
# ====================================================

# 添加可执行文件
add_executable(BulletTest bullet_test.cpp)

# 包含 Bullet 头文件目录
target_include_directories(BulletTest PRIVATE ${BULLET_INCLUDE_DIRS})

# 链接 Bullet 库
target_link_libraries(BulletTest ${BULLET_LIBRARIES})