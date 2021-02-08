//
// Created by Keith Aylwin on 2/11/21.
//

#ifndef TEST_OCTREE_H
#define TEST_OCTREE_H
#include <algorithm>
#include <queue>
#include <cmath>

namespace keith
{
template <typename UserData>
class Octree {
public:
    enum class Quadrant
    {
        bbl = 0,
        bbr = 1,
        bfl = 2,
        bfr = 3,
        tbl = 4,
        tbr = 5,
        tfr = 6,
        tfl = 7
    };

    class Point
    {
    public:
        double x = -1;
        double y = -1;
        double z = -1;
        bool operator==(const Point& p)
        {
            return abs(p.x - this->x) < std::numeric_limits<double>::epsilon() &&
                   abs(p.y - this->y) < std::numeric_limits<double>::epsilon() &&
                   abs(p.z - this->z) < std::numeric_limits<double>::epsilon();
        }
    };

    Point center;
    double half_width = 0;
    double half_height = 0;
    double half_length = 0;

    bool delete_user_data = false;

    // Manage user data at a point
    Point data_point;
    UserData* data_user = nullptr;
    bool is_leaf = true;

    Octree* children[8] = {nullptr};

    /**
     *
     * @param initialPoint This tree needs to be initialized with at least one point
     * @param data Set to what you want stored in this tree
     * @param width x axis of the tree
     * @param length  y axis of the tree
     * @param height  z axis of the tree
     */
    Octree(Point initialPoint, UserData* data, Point new_center, double width, double length, double height):data_user(data),
    data_point(initialPoint)
    {
        half_height = height / 2;
        half_width = width / 2;
        half_length = length / 2;
        center = new_center;
    }

    Octree(Point initialPoint, UserData* data, double width, double length, double height)
            :data_user(data),
             data_point(initialPoint)
    {
        half_height = height / 2;
        half_width = width / 2;
        half_length = length / 2;
        center = {width / 2, length/ 2, height/2};
    }

    ~Octree()
    {
        // Clean up the user data
        if (data_user != nullptr && delete_user_data)
        {
            delete data_user;
            data_user = nullptr;
        }

        for (auto& child : children)
        {
            if (child == nullptr){continue;}
            delete child;
            child = nullptr;
        }
    }

    /**
     * Returns the quadrant that a given point belongs to
     */
    Octree::Quadrant findQuadrant(const Point& point)
    {
        Quadrant quad;
        // Split the middle
        if (point.z > center.z)
        {
            // Point is on the top
            if (point.x > center.x)
            {
                // Point is on the right
                if (point.y > center.y)
                {
                    // Point is in the back
                    quad = Quadrant::tbr;
                }else{
                    // Point is in the front
                    quad = Quadrant::tfr;
                }
            }
            else
            {
                if (point.y > center.y)
                {
                    quad = Quadrant::tbl;
                } else{
                    quad = Quadrant::tfl;
                }
                // Point is on the left or equal
            }
        }
        else
        {
            // Point on the bottom or equal
            if(point.x > center.x)
            {
                // Point is on the right
                if (point.y > center.y)
                {
                    // Point is in the back
                    quad = Quadrant::bbr;
                }
                else{
                    // Point is in the front
                    quad = Quadrant::bfr;
                }
            }
            else{
                // Point is on the left
                if (point.y > center.y)
                {
                    // Point is in the back
                    quad = Quadrant::bbl;
                } else{
                    // Point is in the front
                    quad = Quadrant::bfl;
                }
            }
        }
        return quad;
    }


    template <typename POINT_TYPE>
    void insert(POINT_TYPE&& point, UserData* data) {
        // Handle initial case
        Point userdata;
        UserData* currentData;
        if (find(point,userdata, currentData)){return;}

        Quadrant quad = findQuadrant(point);
        auto index = static_cast<int>(quad);

        // If the node is already a leaf node then we need to drop its value out into the children
        if (is_leaf)
        {
            is_leaf = false;
            insert(data_point, data_user);
        }

        // Does this exist at the current node
        if (children[index] == nullptr) {
            Point new_center{};
            switch (quad) {
                case Quadrant::bbl:
                    new_center = {center.x - half_width / 2, center.y + half_length / 2, center.z - half_height / 2};
                    break;
                case Quadrant::bbr:
                    new_center = {center.x + half_width / 2, center.y + half_length / 2, center.z - half_height / 2};
                    break;
                case Quadrant::bfl:
                    new_center = {center.x - half_width / 2, center.y - half_length / 2, center.z - half_height / 2};
                    break;
                case Quadrant::bfr:
                    new_center = {center.x + half_width / 2, center.y - half_length / 2, center.z - half_height / 2};
                    break;
                case Quadrant::tbl:
                    new_center = {center.x - half_width / 2, center.y + half_length / 2, center.z + half_height / 2};
                    break;
                case Quadrant::tbr:
                    new_center = {center.x + half_width / 2, center.y + half_length / 2, center.z + half_height / 2};
                    break;
                case Quadrant::tfr:
                    new_center = {center.x + half_width / 2, center.y - half_length / 2, center.z + half_height / 2};
                    break;
                case Quadrant::tfl:
                    new_center = {center.x - half_width / 2, center.y - half_length / 2, center.z + half_height / 2};
                    break;
            }
            children[index] = new Octree(point, data, new_center, half_width, half_length, half_height);
            is_leaf = false;


        } else {
            children[index]->insert(point, data);  // Recursively insert new data
        }
    }

    // Locate user data in the closest octant at a point
    bool find(const Point& point, Point& foundPoint, UserData*& userdata)
    {
        if (is_leaf){
            if (data_point == point){
                foundPoint = data_point;
                userdata = data_user;
                return true;
            }
            userdata = nullptr;
            return false;
        }
        Quadrant quad = findQuadrant(point);
        auto index = static_cast<int>(quad);
        if (children[index] == nullptr)
        {
            return false;
        }
        else{
            return children[index]->find(point, foundPoint, userdata);
        }
    }

    void nearestNeighbor(const Point& point, double& bestDistance, Point& bestPoint, UserData*& bestUserData)
    {

        // End case
        if (is_leaf)
        {
            auto dx = data_point.x - point.x;
            auto dy = data_point.y - point.y;
            auto dz = fabs(data_point.z - point.z);
            // Handle identification, distance can't be greater than PI
            if (dz > M_PI){dz -= M_PI;}
            bestDistance = dx * dx + dy * dy + dz;
            bestPoint = data_point;
            bestUserData = data_user;
            return;
        }

        auto min_dist = [](const Octree<UserData>* tree, const Point& p)
        {
            // Sqrt(2) * 2 rad is the distance from center to corner of cube
           auto max_rad = std::max({tree->half_height, tree->half_width, tree->half_length});
           auto rad =  1.41421356237 * max_rad;

            // Euclidian Distance to the target point
           auto dx = p.x - tree->center.x;
           auto dy = p.y - tree->center.y;
           auto dz = fabs(p.z - tree->center.z);
           // Wrap theta
           if (dz > M_PI){dz -= M_PI;}
           auto euc_dist = dx * dx + dy * dy + dz;

           // If this point is within the cube then min_dist is 0
           return (euc_dist > rad * rad) ? euc_dist - rad * rad : 0;
        };


        // Want the smallest distance to come first
        auto cmp = [&, this](const Octree<UserData>* a, const Octree<UserData>* b)
        {
            if (a == nullptr && b != nullptr){return true;}
            if (a != nullptr && b == nullptr){return false;}

            if (a == nullptr && b == nullptr){return false;}
            auto dis_a = min_dist(a, point);
            auto dis_b = min_dist(b, point);
            return dis_a > dis_b;
        };

        // Build the queue
        std::priority_queue<Octree*, std::vector<Octree*>, decltype(cmp)> queue(cmp);
        for (auto& child : children){queue.push(child);}

        bestDistance = -1;
        while(!queue.empty())
        {
            auto child = queue.top();
            queue.pop();
            if(child == nullptr){break;}
            auto min_possible = min_dist(child, point);
            if (min_possible > bestDistance && bestDistance > 0){break;}

            double child_distance;
            Point childPoint;
            UserData* childData;

            child->nearestNeighbor(point, child_distance, childPoint, childData);
            if (bestDistance < 0 || child_distance < bestDistance)
            {
                bestDistance = child_distance;
                bestUserData = childData;
                bestPoint = childPoint;
            }
        }
    }
};

}  // keith
#endif //TEST_OCTREE_H
