#ifndef TEST_GRIDMAP_H
#define TEST_GRIDMAP_H

#include <cmath>
#include <memory>

#include "GridMap.h"

namespace keith
{
/**
 * Class to optimize nearest neighbor search
 */
    class GridMap
    {
    public:

        struct Point
        {
            double x = 0;
            double y = 0;
            double theta = 0;
        };

        class Cell
        {
        private:
            Point stack_buffer[20];  // Max 20 points in 10cm

        public:
            Point* addPoint(Point&& new_point);
            int num_points = 0;

            // Retrieve points from a cell
            Point* operator[](int index)
            {
                if (index < 20){return &(stack_buffer[index]);};
                return nullptr;
            }
        };

        bool computeCellCoords(double x, double y, double theta, int& cell_x, int& cell_y, int& cell_theta);



        /**
         * Create a new grid tree with specified resolution. Bulk allocate objects
         * @return
         */
        GridMap();
        ~GridMap();

        inline Cell& getValue(int x, int y, int theta);

        // Return a reference to a cell in the pre allocated Grid block
        //Cell& operator[](int x, int y, int theta);

        Point* insertPoint(const double& x, const double& y, const double& theta);

        GridMap::Point* nearestPoint(double x, double y, double theta);

        static bool order_se2(const GridMap::Point* a, const GridMap::Point* b);
        GridMap::Point* get_best_point(GridMap::Cell& cell);

        GridMap::Cell& getCell(double x, double y, double theta);

        const double& getWidth(){return width;}
        const double& getHeight(){return height;}
        const double& getWidthResolution(){return w_resolution;}
        const double& getHeightResolution(){return h_resolution;}
        const double& getThetaResolution(){return theta_resolution;}
    private:
        Cell* cell_grid_ = nullptr;
        int count_theta;
        int count_w;
        int count_h;
        int num_cells = 0;
        std::allocator<Cell> allocator;
        double w_resolution = 0.1;  // 10cm
        double h_resolution = 0.1;  // 10cm
        double theta_resolution = 0.1745329;
        double height = 10;  // meters
        double width = 20;  // meters

    };
}  // keith


#endif //TEST_GRIDMAP_H
