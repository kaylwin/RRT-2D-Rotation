#include <exception>
#include <algorithm>
#include <math.h>
#include "rrt_2d/GridMap.h"

using namespace keith;
GridMap::GridMap()
{
    // Initialize cell count
    count_w = static_cast<int>(width / w_resolution);
    count_h = static_cast<int>(height / h_resolution);
    count_theta = static_cast<int>(2 * M_PI / theta_resolution);

    num_cells = count_w * count_h * count_theta;

    // Reserve enough cells up front for the tree
    cell_grid_ = allocator.allocate(num_cells);
}

GridMap::~GridMap()
{
    allocator.deallocate(cell_grid_, num_cells);
}

bool GridMap::computeCellCoords(double x, double y, double theta, int& cell_x, int& cell_y, int& cell_theta)
{
    // Check the width of the incoming data
    if ( x > this->width || x < 0){return false;}
    if (y > this->height || y < 0){return false;}

    // Wrap theta
    if (theta > 2*M_PI){theta -= 2*M_PI;} else if (theta < 0){theta+= 2*M_PI;}

    // Cast down to find the cell index that this point belongs in
    cell_x = static_cast<int>(x / w_resolution);
    cell_y = static_cast<int>(y / h_resolution);
    cell_theta = static_cast<int>(theta / theta_resolution);
    return true;
}

GridMap::Cell& GridMap::getCell(double x, double y, double theta)
{
    int x_cell, y_cell, theta_cell;
    computeCellCoords(x, y, theta, x_cell, y_cell, theta_cell);
    return getValue(x_cell, y_cell, theta_cell);
}

GridMap::Cell& GridMap::getValue(int x, int y, int theta)
{
    auto offset_theta = (theta < 0) ? count_theta - theta : theta;  // Handle identification of theta
    auto offset_y = count_theta * y;
    auto offset_x = x * count_h * count_theta;
    return cell_grid_[offset_x + offset_y + offset_theta];
}
// Return a reference to a cell in the pre allocated Grid block
// Goes along x->y->theta axis
//GridMap::Cell& GridMap::operator[](int x, int y, int theta)
//{
//    return GetValue(x, y, theta);
//}

GridMap::Point* GridMap::insertPoint(const double& x, const double& y, const double& theta)
{
    // Find the cell this belongs in
    if ( x < 0 || y < 0 || theta < 0 || theta > 2 * M_PI)
    {
        throw std::runtime_error("Cannot insert point that's not in the map coordinates");
    }

    // Cast down to find the cell index that this point belongs in
    int cell_x = static_cast<int>(x / w_resolution);
    int cell_y = static_cast<int>(y / h_resolution);
    int cell_theta = static_cast<int>(theta / theta_resolution);

    Cell& cell = getValue(cell_x, cell_y, cell_theta);
    return cell.addPoint(Point{.x = x, .y = y, .theta = theta});
}

/**
 * Determine if point a is less than another point using a metric space on se2
 */
bool GridMap::order_se2(const GridMap::Point* a, const GridMap::Point* b)
{
    // Check the trivial cases ordering null pointers as greater
    if (a == nullptr && b == nullptr){return true;}
    if (a != nullptr && b == nullptr){return true;}
    if (a == nullptr && b != nullptr){return false;}

    // TODO: Compute Se2 metric
    return true;  // Return true if a is less than b
}

/**
 * Look through a cell and determine which point is the best
 * @param cell
 * @return
 */
GridMap::Point* GridMap::get_best_point(GridMap::Cell& cell)
{
    // Search through all points in a cell using the se2_mteric
    GridMap::Point* best_point = nullptr;
    for(int i = 0; i < cell.num_points; i++)
    {
        if(order_se2(cell[i], best_point)){best_point = cell[i];}  // This point cost less
    }
    return best_point;
}

GridMap::Point* GridMap::nearestPoint(double x, double y, double theta)
{
    // Find the index of this in the array

    // Cast down to find the cell index that this point belongs in
    int cell_x = static_cast<int>(x / w_resolution);
    int cell_y = static_cast<int>(y / h_resolution);
    int cell_theta = static_cast<int>(theta / theta_resolution);

    int max_count = std::max({count_w, count_h, count_theta});
    int level = 1;

    // Search in 3D layer around the center
    Point* current_best_point = nullptr;
    double metric = 0;  // Current Best metric

    auto clamp_x = [this](int index)->int{
        if (index > count_w-1){return count_w-1;}
        else if (index < 0){return 0;}
        return index;
    };
    auto clamp_y = [this](int index)->int{
        if (index > count_h - 1) {return count_h - 1;}
        else if (index < 0) { return 0;}
        return index;
    };
    auto clamp_theta = [this](int index)->int{
        int theta_index = index;
        if (theta_index < 0){
            return count_theta - (abs(theta_index) % count_theta);
        }
        return theta_index % count_theta;
    };


    // Search the entire space for the nearest point. If level is greater than the maximum count then the entire space
    // has been searched and nothing has been found :(
    while(current_best_point == nullptr && level < max_count)
    {
        // Search a given level of the cube
        int i;
        int j;

        // TODO: This is over searching -- the edges will be searched by each face twice
        for(i = -level; i < level; i++)
        {
            for(j = -level; j < level; j++)
            {
                // Top Face and bottom face -- fix theta
                int theta_index_top = clamp_theta(cell_theta + level);
                int theta_index_bot = clamp_theta(cell_theta - level);
                int x_index_top_bottom = clamp_x(i + cell_x);
                int y_index_top_bottom = clamp_y(j + cell_y);
                Point* top_face = get_best_point(getValue(x_index_top_bottom, y_index_top_bottom, theta_index_top));
                Point* bot_face = get_best_point(getValue(x_index_top_bottom, y_index_top_bottom, theta_index_bot));

                // Front Face and Rear Face -- fix y
                auto y_level_front = clamp_y(cell_y + level);
                auto y_level_rear = clamp_y(cell_y - level);
                auto x_index_front_rear = clamp_x(cell_x + i);
                auto theta_index_front_rear = clamp_theta(cell_theta + j);
                Point* front_face = get_best_point(getValue(x_index_front_rear, y_level_front, theta_index_front_rear));
                Point* rear_face = get_best_point(getValue(x_index_front_rear, y_level_rear, theta_index_front_rear));

                // Left Face and Right Face -- Fix x
                auto x_level_right = (cell_x + level > count_w - 1) ? count_w - 1 : cell_x + level;
                auto x_level_left = (cell_x - level < 0) ? 0 : cell_x - level;
                auto y_index_right_left = clamp_y(cell_y + i);
                auto theta_index_right_left = clamp_theta(cell_theta + j);
                Point* left_face = get_best_point(getValue(x_level_left, y_index_right_left,  theta_index_right_left));
                Point* right_face = get_best_point(getValue(x_level_right,y_index_right_left, theta_index_right_left));

                // Figure out what the best point is between the current and the existing
                current_best_point = std::min({current_best_point, top_face, bot_face, front_face, rear_face, left_face, right_face}, order_se2);
            }
        }

        // Search x
        level++;
    }

    return current_best_point;
}


///////////////////////////////////////////////////////////
///                     Cell
///////////////////////////////////////////////////////////
GridMap::Point* GridMap::Cell::addPoint(Point&& new_point)
{
    if (num_points < 20)
    {
        stack_buffer[num_points] = new_point;
        num_points++;
        return &stack_buffer[num_points - 1];
    }
    return nullptr;
}
