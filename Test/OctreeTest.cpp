//
// Created by Keith Aylwin on 2/11/21.
//
#include <gtest/gtest.h>
#include "rrt_2d/Octree.h"
#include <random>
#include <chrono>

using namespace keith;

struct UserData
{
    int64_t id = 0;
};
using Point = Octree<UserData>::Point;

/// Create an octree and find the point that just went
TEST(OctreeBasicTests, basicInsertion)
{
    Point point{ 0, 0, 0};
    UserData data{.id=3};
    Point foundPoint;
    UserData* userdata;
    Octree<UserData> tree(point, &data, 20, 10, 5);
    EXPECT_EQ(tree.find(point, foundPoint, userdata), true);
    ASSERT_NE(userdata, nullptr);
    EXPECT_EQ(userdata->id, data.id);
}

/// bbl = 0,
/// bbr = 1,
/// bfl = 2,
/// bfr = 3,
/// tbl = 4,
/// tbr = 5,
/// tfr = 6,
/// tfl = 7
TEST(OctreeBasicTests, insertServeralPoints)
{
    Point points[8];
    // Insert points into bottom of cube
    points[0] = {.25, .75, .25};  // bbl
    points[1] = {.75, .75,.25};  // bbr
    points[2] = {.25, .25, .25};  // bfl
    points[3] = {.75, .25,.25};  // bfr

    // Insert points into top of cube
    points[4] = {.25, .75, .75};  // tbl
    points[5] = {.75, .75,.75};  // tbr
    points[6] = {.75, .25,.75};  // tfr
    points[7] = {.25, .25, .75};  // tfl

    int id = 0;
    // Set the ids for the search
    UserData data[8];
    for(int i = 0; i < 8; i++)
    {
        data[i].id= id++;
    }

    Octree<UserData> tree(points[0], &(data[0]),Point{.5, .5, .5}, 1, 1, 1);
    for(int i = 1; i < 8; i++)
    {
        tree.insert(points[i], &(data[i]));
    }

    // Inspect the id's according to the expected locations of the children
    for(int i = 0; i < 8; i++)
    {
        EXPECT_EQ(tree.children[i]->data_user->id, data[i].id);
        UserData* data_point;
        Point found_point;
        tree.find(points[i], found_point, data_point);
        ASSERT_NE(data_point, nullptr);
        EXPECT_EQ(data_point->id, data[i].id);
    }

    // This function relies upon only 8 values being inserted thus far
    EXPECT_EQ(id, 8);

    UserData child_data[64];
    id = 100;
    // Sub divide each tree again
    for (int i = 0; i < 8; i++)
    {
        // Go through each of the sub points and subdivide again
        Point sub_points[8];
        // bbl
        sub_points[0].x = tree.children[i]->center.x - tree.children[i]->half_width / 2;
        sub_points[0].y = tree.children[i]->center.y + tree.children[i]->half_length / 2;
        sub_points[0].z = tree.children[i]->center.z - tree.children[i]->half_height / 2;

        // bbr = 1,
        sub_points[1].x = tree.children[i]->center.x + tree.children[i]->half_width / 2;
        sub_points[1].y = tree.children[i]->center.y + tree.children[i]->half_length / 2;
        sub_points[1].z = tree.children[i]->center.z - tree.children[i]->half_height / 2;

        // bfl = 2,
        sub_points[2].x = tree.children[i]->center.x - tree.children[i]->half_width / 2;
        sub_points[2].y = tree.children[i]->center.y - tree.children[i]->half_length / 2;
        sub_points[2].z = tree.children[i]->center.z - tree.children[i]->half_height / 2;

        // bfr = 3,
        sub_points[3].x = tree.children[i]->center.x + tree.children[i]->half_width / 2;
        sub_points[3].y = tree.children[i]->center.y - tree.children[i]->half_length / 2;
        sub_points[3].z = tree.children[i]->center.z - tree.children[i]->half_height / 2;

        // tbl = 4,
        sub_points[4].x = tree.children[i]->center.x - tree.children[i]->half_width / 2;
        sub_points[4].y = tree.children[i]->center.y + tree.children[i]->half_length / 2;
        sub_points[4].z = tree.children[i]->center.z + tree.children[i]->half_height / 2;

        // tbr = 5,
        sub_points[5].x = tree.children[i]->center.x + tree.children[i]->half_width / 2;
        sub_points[5].y = tree.children[i]->center.y + tree.children[i]->half_length / 2;
        sub_points[5].z = tree.children[i]->center.z + tree.children[i]->half_height / 2;

        // tfr = 6,
        sub_points[6].x = tree.children[i]->center.x + tree.children[i]->half_width / 2;
        sub_points[6].y = tree.children[i]->center.y - tree.children[i]->half_length / 2;
        sub_points[6].z = tree.children[i]->center.z + tree.children[i]->half_height / 2;

        // tfl = 7
        sub_points[7].x = tree.children[i]->center.x - tree.children[i]->half_width / 2;
        sub_points[7].y = tree.children[i]->center.y - tree.children[i]->half_length / 2;
        sub_points[7].z = tree.children[i]->center.z + tree.children[i]->half_height / 2;

        // Add all the points and insert with the correct data format
        for(int j = 0; j < 8; j++)
        {
            UserData* new_child_data = &(child_data[i*8 +j]);
            new_child_data->id = id++;
            tree.insert(sub_points[j], new_child_data);
        }

        // Check that the children are where we expect them to be
        for(int j = 0; j < 8; j++)
        {
           ASSERT_NE(tree.children[i], nullptr);
           ASSERT_NE(tree.children[i]->children[j], nullptr);

           // If it bumps the node down a level then that is fine
           if (tree.children[i]->children[j]->is_leaf)
           {
               EXPECT_EQ(tree.children[i]->children[j]->data_user->id, 100 + 8*i + j);
           }

           UserData* data;
           Point found_point;
           EXPECT_EQ(tree.find(sub_points[j], found_point, data), true);
           ASSERT_NE(data, nullptr);
        }

    }
}


TEST(OctreeSearch, OctreeBasicSearch)
{
    Point points[8];
    // Insert points into bottom of cube
    points[0] = {.25, .75, .25};  // bbl
    points[1] = {.75, .75,.25};  // bbr
    points[2] = {.25, .25, .25};  // bfl
    points[3] = {.75, .25,.25};  // bfr

    // Insert points into top of cube
    points[4] = {.25, .75, .75};  // tbl
    points[5] = {.75, .75,.75};  // tbr
    points[6] = {.75, .25,.75};  // tfr
    points[7] = {.25, .25, .75};  // tfl

    int id = 0;
    // Set the ids for the search
    UserData data[8];
    for(int i = 0; i < 8; i++)
    {
        data[i].id= id++;
    }

    Octree<UserData> tree(points[0], &(data[0]),Point{.5, .5, .5}, 1, 1, 1);
    for(int i = 1; i < 8; i++)
    {
        tree.insert(points[i], &(data[i]));
    }

    // Check in each quadrant that the search found the point
    Point point{.3, .75, .25};

    auto cmp_points = [](Point& a, Point&& b){
        EXPECT_DOUBLE_EQ(a.x, b.x);
        EXPECT_DOUBLE_EQ(a.y, b.y);
        EXPECT_DOUBLE_EQ(a.z, b.z);
    };

    auto search_compare = [&](Point&& p, Point&& actual)
    {
        Point nearestPoint;
        double bestWeight;
        Point searchPoint = p;
        UserData* nearestData = nullptr;
        tree.nearestNeighbor(searchPoint, bestWeight, nearestPoint, nearestData);
        cmp_points(nearestPoint, std::move(actual));
    };

    // bbl
    search_compare(Point{.3, .75, .25}, Point{.25, .75, .25});
    search_compare(Point{.3, .8, .25}, Point{.25, .75, .25});
    search_compare(Point{.3, .8, .3}, Point{.25, .75, .25});

    search_compare(Point{.8, .75, .25}, Point{.75, .75,.25});  // bbr
    search_compare(Point{.75, .8, .25}, Point{.75, .75, .25});  // bbr
    search_compare(Point{.75, .75, .3}, Point{.75, .75, .25});  // bbr

    search_compare(Point{.3, .25, .25}, Point{.25, .25, .25});  // bfl
    search_compare(Point{.25, .3, .25}, Point{.25, .25, .25});  // bfl
    search_compare(Point{.25, .25, .3}, Point{.25, .25, .25});  // bfl
    search_compare(Point{.3, .3, .3}, Point{.25, .25, .25});  // bfl

    search_compare(Point{.8, .25, .25}, Point{.75, .25, .25});
    search_compare(Point{.75, .3, .25}, Point{.75, .25, .25});
    search_compare(Point{.75, .25, .3}, Point{.75, .25, .25});
    search_compare(Point{.8, .3, .3}, Point{.75, .25, .25});

    search_compare(Point{.3, .75, .75}, Point{.25, .75, .75});
    search_compare(Point{.25, .8, .75}, Point{.25, .75, .75});
    search_compare(Point{.25, .75, .8}, Point{.25, .75, .75});
    search_compare(Point{.3, .8, .8}, Point{.25, .75, .75});

    search_compare(Point{.8, .75, .75}, Point{.75, .75, .75});
    search_compare(Point{.75, .8, .75}, Point{.75, .75, .75});
    search_compare(Point{.75, .75, .8}, Point{.75, .75, .75});
    search_compare(Point{.8, .8, .8}, Point{.75, .75, .75});

    search_compare(Point{.8, .25, .75}, Point{.75, .25, .75});
    search_compare(Point{.75, .3, .75}, Point{.75, .25, .75});
    search_compare(Point{.75, .25, .8}, Point{.75, .25, .75});
    search_compare(Point{.8, .3, .8}, Point{.75, .25, .75});

    search_compare(Point{.3, .25, .75}, Point{.25, .25, .75});
    search_compare(Point{.25, .3, .75}, Point{.25, .25, .75});
    search_compare(Point{.25, .25, .8}, Point{.25, .25, .75});
    search_compare(Point{.3, .3, .8}, Point{.25, .25, .75});



    // Now time to insert points on top of eachother
    UserData data2[64];
    tree.insert(Point{.8, .8, .8}, nullptr);
    search_compare(Point{.9, .9, .9}, Point{.8, .8, .8});

    // Pile a ton of points right next to eachother
    tree.insert(Point{.81, .8, .8}, nullptr);
    tree.insert(Point{.82, .8, .8}, nullptr);
    tree.insert(Point{.83, .8, .8}, nullptr);
    tree.insert(Point{.84, .8, .8}, nullptr);
    search_compare(Point{.9, .9, .9}, Point{.84, .8, .8});
}


TEST(RandomInsertion, randomFind)
{

    Point center{.1, .2, .3};
    Octree<UserData> tree(center, nullptr ,Point{10, 5, M_PI}, 20, 10, 2*M_PI);;

    auto rando = std::minstd_rand(1);
    // Create a random number
    auto rand_point = [&]()->Point{
        Point p;
        p.x = (rando() % 10'000) / 500.0f;
        p.y = (rando() % 10'000) / 1000.0f;
        p.z = (rando() % 10'000) / 10'000.0f * 2*M_PI;

        return p;
    };
    for (int i = 0; i < 1e6; i++)
    {
        Point p = rand_point();
        tree.insert(p, nullptr);
    }

    using Clock = std::chrono::high_resolution_clock;


    // This is to evaluate performance
    double sum = 0;
    auto total_entries = 10;
    for (int i = 0; i < total_entries; i++)
    {
        auto t1 = Clock::now();
        Point p = rand_point();
        Point bestPoint;
        UserData* userdata;
        double bestDistance;
        tree.nearestNeighbor(p, bestDistance, bestPoint, userdata);
        auto t2 = Clock::now();
        sum += (t2 - t1).count();
    }

    std::cerr << "Average lookup time: " << total_entries / sum << std::endl;

}
