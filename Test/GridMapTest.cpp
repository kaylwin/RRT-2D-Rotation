#include <math.h>
#include <gtest/gtest.h>
#include "rrt_2d/GridMap.h"


using namespace keith;
using namespace std;

using Cell = GridMap::Cell;
using Point = GridMap::Point;

TEST(BasicFunctionality, basicInsert)
{
    GridMap gridmap;

    // Comprehensively check the insertion and deletion of the space
    // i / j / k are offset a tad to push them evenly into cells
    for (double i = gridmap.getWidthResolution() / 2; i < gridmap.getWidth(); i += gridmap.getWidthResolution())
    {
        for ( double j = gridmap.getHeightResolution() / 2; j < gridmap.getHeight(); j = j + gridmap.getHeightResolution())
        {
            for (double k = gridmap.getThetaResolution() / 2; k < 2*M_PI - gridmap.getThetaResolution(); k += gridmap.getThetaResolution())
            {
                gridmap.insertPoint(i, j, k);
                Cell& cell = gridmap.getCell(i, j, k);
                Point* first_point = cell[0];
                EXPECT_DOUBLE_EQ(first_point->x, i);
                EXPECT_DOUBLE_EQ(first_point->y, j);
                EXPECT_DOUBLE_EQ(first_point->theta, k);

            }
        }
    }

    // This should already have a point in it from the above loop
    gridmap.insertPoint(4.05, 4, M_PI_2);
    Cell& cell = gridmap.getCell(4.05, 4, M_PI_2);
    Point* second_cell = cell[1];
    EXPECT_DOUBLE_EQ(second_cell->x, 4.05);
    EXPECT_DOUBLE_EQ(second_cell->y, 4);
    EXPECT_DOUBLE_EQ(second_cell->theta, M_PI_2);
}


TEST(TestSearch, nearbysearch)
{
    GridMap map;
    // Shoudl be dead center in the space
    map.insertPoint(5.2, 5, M_PI / 2);
    Point* point = map.nearestPoint(5, 5, M_PI/ 2);
    ASSERT_NE(point, nullptr);
    EXPECT_DOUBLE_EQ(point->x, 5.2);
    EXPECT_DOUBLE_EQ(point->y, 5);
    EXPECT_DOUBLE_EQ(point->theta, M_PI / 2);

    map.insertPoint(4.9, 4.3, M_PI / 2);
    point = map.nearestPoint(4, 4, M_PI/ 2);
    ASSERT_NE(point, nullptr);
    EXPECT_DOUBLE_EQ(point->x, 4.9);
    EXPECT_DOUBLE_EQ(point->y, 4.3);
    EXPECT_DOUBLE_EQ(point->theta, M_PI/2);
}

TEST(TestSearch, ThetaWrap)
{
    GridMap map;
    // So go one less than the max theta
    map.insertPoint(5, 5, 2 * M_PI - map.getThetaResolution());
    map.insertPoint(5, 5, M_PI_4);
    Point* point = map.nearestPoint(5, 5, M_PI + M_PI_2);

    ASSERT_NE(point, nullptr);
    EXPECT_DOUBLE_EQ(point->x, 5);
    EXPECT_DOUBLE_EQ(point->y, 5);
    EXPECT_DOUBLE_EQ(point->theta, 2 * M_PI - map.getThetaResolution());
}

TEST(TestSearch, CrossMapConnection)
{
    GridMap map;
    map.insertPoint(.5,.5,M_PI_4);
    Point* point = map.nearestPoint(19,9,2 * M_PI);
    ASSERT_NE(point, nullptr);
    EXPECT_DOUBLE_EQ(point->x, .5);
    EXPECT_DOUBLE_EQ(point->y, .5);
    EXPECT_DOUBLE_EQ(point->theta, M_PI_4);
}



