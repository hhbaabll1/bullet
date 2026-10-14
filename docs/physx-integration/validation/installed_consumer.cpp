#include <BulletDynamics/ConstraintSolver/btConstraintExtension.h>
#include <BulletDynamics/ConstraintSolver/btRowDescriptorConstraint.h>
#include <BulletDynamics/Featherstone/btMultiBodyRowDescriptorConstraint.h>
#include <BulletSoftBody/btConstraintElementOperator.h>
int main(){btConstraintDescriptionGraph g;btConstraintElementPoint p;return btConstraintDescriptionImporter::validate(g).isOk() && p.validate().isOk()?0:1;}
