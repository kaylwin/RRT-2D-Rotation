#include "rrt_2d/Utils.h"
#include <algorithm>

using namespace keith;

// Check the degrees as the shape goes around
auto wrap_rad = [](float&& theta){
    if (theta < 0)
    {
        return theta += 2*M_PI;
    }
    else if (theta > 2*M_PI)
    {
        return theta -= 2*M_PI;
    }
    else{
        return theta;
    }
};

/**
 * Basic collision checking yielding whether two bodies are touching. Checks distance by 10cm blocks
 * and checks rotation by 10 degree increments
 * @param shape
 * @param pos
 * @return
 */
bool Utils::hasCollision(b2Shape* shapeA, const b2Transform& start_transA, const b2Transform& end_transA, b2Shape* shapeB, const b2Transform& transB)
{

    // Figure out which one has more steps
    float dist_x = end_transA.p.x - start_transA.p.x;
    float dist_y = end_transA.p.y - start_transA.p.y;

    // Wrap the distance of theta
    float dist_theta = end_transA.q.GetAngle() - start_transA.q.GetAngle();
    if(dist_theta > M_PI){dist_theta -= M_PI;}else if(dist_theta < -M_PI){dist_theta += M_PI;}

    int count_x = abs(static_cast<int>(dist_x / 0.1));
    int count_y = abs(static_cast<int>(dist_y / 0.1));
    int count_theta = abs(static_cast<int>(dist_theta / 0.174));  // Every 10 degrees of rotation
    size_t max_count = std::max({count_x, count_y, count_theta});

    float dx = dist_x / max_count;
    float dy = dist_y / max_count;
    float dtheta = dist_theta / max_count;

    b2Transform inter_trans{};
    for (size_t i = 0; i < max_count; i++)
    {
        auto pos_x = start_transA.p.x + dx * i;
        auto pos_y = start_transA.p.y + dy * i;
        auto pos_theta = wrap_rad(start_transA.q.GetAngle() + dtheta * i);
        inter_trans.Set(b2Vec2{pos_x, pos_y}, pos_theta);
        if(b2TestOverlap(shapeA, 0, shapeB, 0, inter_trans, transB))
        {
            return true;  // There was a collision
        }
    }

    return false;
}
