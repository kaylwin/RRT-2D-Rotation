#include <rrt_2d/Utils.h>
#include <gtest/gtest.h>
#include <box2d/b2_polygon_shape.h>

using namespace keith;
class CollisionTestFixture : public ::testing::Test
{
protected:
  b2PolygonShape* sa;
  b2Transform ta1;
  b2Transform ta2;

  b2PolygonShape* sb;
  b2Transform tb;

  virtual void SetUp()
  {
      sa = new b2PolygonShape();
      sb = new b2PolygonShape();

      // Changing these will break test cases
      sa->SetAsBox(0.5, 5);
      sb->SetAsBox(0.5, 0.5);

      // Set transforms
      ta1.Set(b2Vec2{0.0, 0.0}, 0);
      ta2.Set(b2Vec2{0.0, 0.0}, 0);
      tb.Set(b2Vec2{5.0, 0.0}, 0);
  }

  bool check(){
      return keith::Utils::hasCollision(sa, ta1, ta2, sb, tb);
  }

  virtual void TearDown()
  {
      delete sa;
      delete sb;
      sa = sb = nullptr;
  }
};

// Sweep straight into the start object
TEST_F(CollisionTestFixture, TestDirectCollsion)
{
    // Throw the objects at eachother
    ta2.Set(b2Vec2{6,0}, 0);
    ASSERT_EQ(check(), true);
}

// Sweep a collision by rotating
TEST_F(CollisionTestFixture, TestNoCollsion)
{
    ta2.Set(b2Vec2{-6, 0}, 0);
    ASSERT_EQ(check(), false);
}

// Check Glancing Collision
TEST_F(CollisionTestFixture, TestGlancingCollision)
{
    ta2.Set(b2Vec2{7, 3}, 0);
    ASSERT_EQ(check(), true);
}

// Check Collision -- tripping
TEST_F(CollisionTestFixture, TestRotatingCollision)
{
    // This should sweep and kick the obstacle
    ta2.Set(b2Vec2{10, 5}, M_PI_2);
    ASSERT_EQ(check(), true);
}

// Check Collision -- Not Tripping
TEST_F(CollisionTestFixture, NoCollision)
{
    // This should rotate over the obstacle
    ta2.Set(b2Vec2{10, 5}, -M_PI_2);
    ASSERT_EQ(check(), false);
}

TEST_F(CollisionTestFixture, BarelyGlancingCollision)
{
    // This should barely collide
    // The vector from the center point to point of collision @ 45 degrees
    // (-1.4064, -2.11) meters
    // Center point of collision be offset from point of contact (4.5, .5)
    // (5.9064, 2.61) meters is the cut off for a collision
    // The point of collision should be 4.5 + 1.4064

    // First check that at 10cm above it doesn't hit
    ta2.Set(b2Vec2{5.9064 * 2, 2.61*2 + 0.1}, -M_PI_2);
    EXPECT_EQ(check(), false);

    // Second check that at 10cm below it does hit
    ta2.Set(b2Vec2{5.9064 * 2, 2.61*2 - 0.1}, -M_PI_2);
    EXPECT_EQ(check(), true);


}

