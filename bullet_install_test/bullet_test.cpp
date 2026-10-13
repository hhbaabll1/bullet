#include <iostream>
#include <btBulletDynamicsCommon.h>

int main() {
    // 1. 配置碰撞检测算法
    btDefaultCollisionConfiguration* collisionConfiguration = new btDefaultCollisionConfiguration();

    // 2. 碰撞分发器
    btCollisionDispatcher* dispatcher = new btCollisionDispatcher(collisionConfiguration);

    // 3. 宽相碰撞检测
    btBroadphaseInterface* overlappingPairCache = new btDbvtBroadphase();

    // 4. 约束求解器
    btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver;

    // 5. 实例化动态世界
    btDiscreteDynamicsWorld* dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, overlappingPairCache, solver, collisionConfiguration);

    // 设置重力
    dynamicsWorld->setGravity(btVector3(0, -10, 0));

    std::cout << "Bullet 物理引擎初始化成功！当前重力为: " 
              << dynamicsWorld->getGravity().getY() << std::endl;

    // 清理内存
    delete dynamicsWorld;
    delete solver;
    delete overlappingPairCache;
    delete dispatcher;
    delete collisionConfiguration;

    return 0;
}