
#ifndef TEST_UTILS_H
#define TEST_UTILS_H
#include <box2d/b2_shape.h>
#include <box2d/b2_body.h>
namespace keith
{
class Utils
{
public:
  static bool hasCollision(b2Shape* shapeA, const b2Transform& start_transA, const b2Transform& end_transA, b2Shape* shapeB, const b2Transform& transB);


};
}

#endif //TEST_UTILS_H
