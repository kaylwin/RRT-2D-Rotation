#include <memory>
#include <random>
#include <iostream>
#include <box2d/b2_world.h>
#include <box2d/b2_body.h>
#include <box2d/b2_polygon_shape.h>
#include <box2d/b2_fixture.h>
#include <box2d/b2_joint.h>
#include <box2d/b2_distance_joint.h>
#include <box2d/b2_gear_joint.h>
#include <box2d/b2_time_of_impact.h>
#include "rrt_2d/Octree.h"
#include "helper.h"
#include "rrt_2d/Utils.h"

// Define window height in meters
#define WINDOW_HEIGHT 10.0f


float aspect_ratio;
float timeStep = 1.0f / 60.0f;
int32 velocityIterations = 10;
int32 positionIterations = 5;

bool initialized = false;
b2Vec2* gravity = nullptr;
b2World* world = nullptr;


b2Body* groundBody;

// The walls that the object is to avoid
b2Body* leftWall;
b2Body* rightWall;
b2Body* topWall;
b2Body* centerWall;
b2Body* center2Wall;
b2Body* center3Wall;


// Used for searching collision

std::vector<b2Body*> bodies;
b2Body* goalBody;

b2PolygonShape collisionShape;
b2Transform collisionTransform;

b2PolygonShape robotShape;
b2Transform robotTransform;

template <typename THETA_TYPE>
void wrap_theta(THETA_TYPE&& theta)
{
   if (theta < 0)
   {
       theta += 2 * M_PI;
   }
   else if (theta > 2 * M_PI)
   {
      theta -= 2 * M_PI;
   }
}


struct Vertex
{
    double x = 0;  // m
    double y = 0;  // m
    double theta = 0;  // rad
    uint32_t id;
    Vertex* p_vertex = nullptr;  // Previous vertex

    Vertex(double x, double y, double theta, Vertex* p_vertex):x(x), y(y), 
        theta(theta), p_vertex(p_vertex)
    {
       id = m_id++;
    }
    Vertex()
    {
        id= m_id++;
    }
    static void reset_id()
    {
        m_id = 0;
    }
    double theta_deg()
    {
        return theta * 180.0 / M_PI;
    }

private:
    static uint32_t m_id;
};

uint32_t Vertex::m_id = 0; // Definition for Vertex ID

std::vector<Vertex*> vertices{};
std::vector<Vertex*> goal_points{};

using Kpoint = keith::Octree<Vertex>::Point;
keith::Octree<Vertex>* tree = nullptr;

Vertex* createNewVertex()
{
    static std::linear_congruential_engine<std::uint_fast32_t, 48271, 0, 2147483647> engine(0);
    auto grand = [&]()->double{ return engine() / static_cast<double>(RAND_MAX);};
    Vertex* v = new Vertex();
    v->x = grand() * WINDOW_HEIGHT * aspect_ratio;
    v->y = grand() * WINDOW_HEIGHT;
    v->theta = grand() * 2 * M_PI;
    v->p_vertex = nullptr;
    return v;
}

/**
 * Function will make sure that no more than 10cm will be covered
 * between two vertices, this will make space exploration more efficient
 * at least in R2 for simplicity S1 is not covered (e.g no explicit max on rotation)
 * Function will set previous vertices for prev vertex
 * @param new_vertex New Vertex
 * @param prev_vertex Vertex already in tree
 */
void insert_intermediary_vertices(Vertex* new_vertex, Vertex* prev_vertex)
{
    double dist_sq = (new_vertex->x - prev_vertex->x) * (new_vertex->x - prev_vertex->x)
            + (new_vertex->y - prev_vertex->y) * (new_vertex->y - prev_vertex->y);
    auto v_count = dist_sq / 1;
    if (v_count < 1){
        new_vertex->p_vertex = prev_vertex;
        return;
    }

    // Wrap theta
    auto new_theta = (new_vertex->theta > M_PI) ? new_vertex->theta - 2*M_PI : new_vertex->theta;
    auto prev_theta = (prev_vertex->theta > M_PI) ? new_vertex->theta - 2*M_PI : prev_vertex->theta;
    auto total_theta = new_theta - prev_theta;
    if (total_theta > M_PI){total_theta -= M_PI;} else if(total_theta < -M_PI){total_theta += M_PI;}

    Vertex* previous = prev_vertex;
    Vertex* int_vertex;
    for (int i = 0; i < v_count; i++)
    {
        int_vertex = new Vertex();
        double x = previous->x + (new_vertex->x - prev_vertex->x) * 1 / v_count;
        double y = previous->y + (new_vertex->y - prev_vertex->y) * 1 / v_count;
        double theta = previous->theta + (total_theta) * 1 / v_count;

        if (theta > 2 * M_PI)
        {
            theta -= 2 * M_PI;
        }
        else if(theta < 0){
            theta += 2 * M_PI;
        }

        int_vertex->x = x;
        int_vertex->y = y;
        int_vertex->theta = theta;
        int_vertex->p_vertex = previous;

        // Don't do these out of order
        previous = int_vertex;

        tree->insert(Kpoint{int_vertex->x, int_vertex->y, int_vertex->theta}, int_vertex);
        vertices.emplace_back(int_vertex);
    }

    // Set the previous on the last vertex
    new_vertex->p_vertex = int_vertex;

}


/**
  *  Create all fixutres for animation, some of these aren't used yet and I left them 
  *  so that I would have some boilerplate code later.
  */
void setUpWorld(const QRect& screenDims){
    gravity = new b2Vec2(0.0f, -10.0f);
    world = new b2World(*gravity);
    aspect_ratio = static_cast<float>(screenDims.width()) / static_cast<float>(screenDims.height());

    // Create the ground and walls
    b2BodyDef groundBodyDef;
    groundBodyDef.type = b2_staticBody;
    groundBodyDef.position.Set(0.0f, 0.0f);
    groundBody = world->CreateBody(&groundBodyDef);
    b2PolygonShape groundBox;
    groundBox.SetAsBox( WINDOW_HEIGHT * aspect_ratio,1.0f);
    groundBody->CreateFixture(&groundBox, 0.0f);

    // Create a dynamic body
    b2BodyDef bodyDef;
    b2PolygonShape dynamicBox;
    dynamicBox.SetAsBox(1.0f, 1.0f);
    b2FixtureDef fixtureDef;


    // Left Wall
    b2BodyDef leftDef;
    b2PolygonShape leftBox;
    b2FixtureDef leftFixture;
    leftDef.type = b2_staticBody;
    leftDef.position.Set(0.0f, 5.0f);
    leftWall = world->CreateBody(&leftDef);
    leftBox.SetAsBox(0.5f, 5.0f);
    fixtureDef.restitution = .01;
    fixtureDef.shape = &leftBox;
    fixtureDef.density = 1.0f;
    fixtureDef.friction = 0.1f;
    leftWall->CreateFixture(&fixtureDef);

    // Right Wall
    b2BodyDef rightDef;
    b2PolygonShape rightBox;
    b2FixtureDef rightFixture;
    rightDef.type = b2_staticBody;
    rightDef.position.Set(WINDOW_HEIGHT * aspect_ratio, 5);
    rightWall = world->CreateBody(&rightDef);
    rightBox.SetAsBox(0.5f, 5.0f);
    fixtureDef.restitution = .01;
    fixtureDef.shape = &rightBox;
    fixtureDef.density = 1.0f;
    fixtureDef.friction = 0.1f;
    rightWall->CreateFixture(&fixtureDef);

    // top Wall
    b2BodyDef topDef;
    b2PolygonShape topBox;
    b2FixtureDef topFixture;
    topDef.type = b2_staticBody;
    topDef.position.Set(WINDOW_HEIGHT * aspect_ratio/2.0, WINDOW_HEIGHT);
    topWall = world->CreateBody(&topDef);
    topBox.SetAsBox(WINDOW_HEIGHT * aspect_ratio / 2.0, 0.5f);
    fixtureDef.restitution = .01;
    fixtureDef.shape = &topBox;
    fixtureDef.density = 1.0f;
    fixtureDef.friction = 0.1f;
    topWall->CreateFixture(&fixtureDef);

    // Center Wall
    b2BodyDef centerDef;
    b2PolygonShape centerBox;
    b2FixtureDef centerFixture;
    centerDef.type = b2_staticBody;
    centerDef.position.Set(WINDOW_HEIGHT * aspect_ratio * .2, WINDOW_HEIGHT*.25);
    centerWall = world->CreateBody(&centerDef);
    centerBox.SetAsBox(1.0, 2);
    fixtureDef.restitution = .01;
    fixtureDef.shape = &centerBox;
    fixtureDef.density = 1.0f;
    fixtureDef.friction = 0.1f;
    centerWall->CreateFixture(&fixtureDef);

    // Second center wall
    b2BodyDef center2Def;
    b2PolygonShape center2Box;
    b2FixtureDef center2Fixture;
    center2Def.type = b2_staticBody;
    center2Def.position.Set(WINDOW_HEIGHT * aspect_ratio * 0.4f, WINDOW_HEIGHT*.75);
    center2Wall = world->CreateBody(&center2Def);
    center2Box.SetAsBox(1.0, 3);
    fixtureDef.restitution = .01;
    fixtureDef.shape = &center2Box;
    fixtureDef.density = 1.0f;
    fixtureDef.friction = 0.1f;
    center2Wall->CreateFixture(&fixtureDef);

    // Third center wall
    b2BodyDef center3Def;
    b2PolygonShape center3Box;
    b2FixtureDef center3Fixture;
    center3Def.type = b2_staticBody;
    center3Def.position.Set(WINDOW_HEIGHT * aspect_ratio * 0.6f, WINDOW_HEIGHT*.25);
    center3Wall = world->CreateBody(&center3Def);
    center3Box.SetAsBox(1.0, 3);
    fixtureDef.restitution = .01;
    fixtureDef.shape = &center3Box;
    fixtureDef.density = 1.0f;
    fixtureDef.friction = 0.1f;
    center3Wall->CreateFixture(&fixtureDef);

    // Goal
    b2BodyDef goalDef;
    b2PolygonShape goalBox;
    b2FixtureDef goalFixture;
    goalDef.type = b2_staticBody;
    goalDef.position.Set(WINDOW_HEIGHT * aspect_ratio * 0.90, WINDOW_HEIGHT/2);
    goalBody = world->CreateBody(&goalDef);
    goalBox.SetAsBox(1.0, 1.0);
    goalFixture.restitution = .01;
    goalFixture.shape = &goalBox;
    goalFixture.density = 1.0f;
    goalFixture.friction = 0.1f;
    goalBody->CreateFixture(&goalFixture);


    robotShape.SetAsBox(2.0, 0.5);
    robotTransform.Set(b2Vec2{2.0, WINDOW_HEIGHT / 2.0f}, 0);

    // Make sure the bodies list is populated
    bodies = {centerWall, center2Wall, center3Wall, groundBody, leftWall, rightWall, topWall};

    auto* initialVertex = new Vertex(2, 4, M_PI_2,nullptr);
    vertices.push_back(initialVertex);
    tree = new keith::Octree<Vertex>{Kpoint{2,4,M_PI_2}, initialVertex, Kpoint{10, 5, M_PI_2}, 20, 10, 2 * M_PI};
    tree->delete_user_data = true;  // Need to clean up verticies

    // Don't run this twice
    initialized = true;
}

Helper::Helper()
{
    QLinearGradient gradient(QPointF(50, -20), QPointF(80, 20));
    gradient.setColorAt(0.0, Qt::white);
    gradient.setColorAt(1.0, QColor(0xa6, 0xce, 0x39));

    background = QBrush(QColor(64, 32, 64));
    circleBrush = QBrush(gradient);
    circlePen = QPen(Qt::black);
    circlePen.setWidth(1);
    textPen = QPen(Qt::white);
    textFont.setPixelSize(50);
}

Helper::~Helper()
{
    if (tree != nullptr)
    {
        delete tree;
        vertices.clear();
        goal_points.clear();
    }
    delete world;
    delete gravity;
}


void drawBox(QPainter* painter, const b2Shape& draw_shape, const b2Transform& transform,
             const QColor& color = QColor(255, 255, 255))
{
    painter->save();
    const auto& polygonShape = dynamic_cast<const b2PolygonShape&>(draw_shape);
    QPointF points[b2_maxPolygonVertices];
    auto body_x = transform.p.x;
    auto body_y = transform.p.y;
    painter->translate(body_x, body_y);
    auto rotation = transform.q.GetAngle() * 180.0f / M_PI;
    if (rotation < 0){rotation += 360;}
    if (rotation > 360){rotation -= 360;}
    painter->rotate(rotation);
    for (int i = 0; i < polygonShape.m_count; i++)
    {
        points[i] = QPointF(polygonShape.m_vertices[i].x , polygonShape.m_vertices[i].y);
    }
    QPen pen;
    pen.setWidth(0.1f);
    pen.setColor(color);
    painter->setRenderHint(QPainter::Antialiasing);
    painter->setPen(pen);
    painter->setBrush(color);
    painter->drawConvexPolygon(points, polygonShape.m_count);
    painter->restore();
}

/**
  *  From the new to the previous point check if there is an unobstructed path between them. This
  *  is not optimized at all, more a blunt trauma check. The Check collion proceeds 10cm increments
  *  with 10 degrees rotation each step, whichever covers less distance.
  */
bool checkCollision(Vertex* new_vertex, Vertex* nearest_vertex)
{

    b2Transform new_vertex_transform{b2Vec2{static_cast<float>(new_vertex->x), static_cast<float>(new_vertex->y)},
                                     b2Rot{static_cast<float>(new_vertex->theta)}};

    b2Transform nearest_vertex_transform{b2Vec2{static_cast<float>(nearest_vertex->x), static_cast<float>(nearest_vertex->y)},
                                         b2Rot{static_cast<float>(nearest_vertex->theta)}};

    for (auto& body : bodies)
    {
        auto body_shape = body->GetFixtureList()[0].GetShape();
        auto body_transform = body->GetTransform();
        if(keith::Utils::hasCollision(&robotShape, new_vertex_transform, nearest_vertex_transform, body_shape,
                body_transform)){
           return true;
        }
    }

    return false;
}


/**
  *  This function takes the position of the slider, figures out how far along the 
  *  goal path the object is then animates it at the correct position
  */
void animateGoalPath(QPainter *painter, int percent)
{
    if(goal_points.empty()){return;}
    Vertex* goal_point = goal_points[0];
    Vertex* curr_point = goal_point;
    float distance_sq = 0;
    auto cmp_dist = [&](Vertex* a, Vertex* b){
        auto dx = a->x - b->x;
        auto dy = a->y - b->y;
        return dx * dx + dy * dy;
    };

    // Add all the elements to the list
    std::list<Vertex*> goal_path;
    while(curr_point->p_vertex != nullptr) {
        goal_path.push_back(curr_point);
        distance_sq += cmp_dist(curr_point, curr_point->p_vertex);
        curr_point = curr_point->p_vertex;
    }
    goal_path.push_back(curr_point); // Get the beginning point

    float target_dist = distance_sq * (static_cast<float>(percent) / 100.0f);
    float current_dist = 0;
    auto curr_vertex_it = goal_path.rbegin();
    while(current_dist < target_dist && std::next(curr_vertex_it) != goal_path.rend()){
        auto curr = *curr_vertex_it;
        auto next = *std::next(curr_vertex_it);
        auto new_dist = cmp_dist(curr, next);
        if (current_dist + new_dist >= target_dist- .001) {
            // Solve for interpolation and draw
            auto t = (target_dist - current_dist) / new_dist;
            auto dx = (next->x - curr->x) * t;
            auto dy = (next->y - curr->y) * t;
            auto next_theta= next->theta;
            auto curr_theta = curr->theta;

            // Make sure theta values do not overflow
            if(next_theta > M_PI){next_theta -= 2*M_PI;} 
            if(curr_theta > M_PI){curr_theta-= 2*M_PI;}
            auto total_theta = (next_theta - curr_theta);
            if (total_theta > M_PI){total_theta -= M_PI;} else if(total_theta < -M_PI){total_theta += M_PI;}
            auto dtheta = total_theta * t;
            auto new_theta = curr->theta + dtheta;
            if(new_theta > 2*M_PI){new_theta -= 2 * M_PI;}else if (new_theta < 0){new_theta+=2*M_PI;}

            // Helpful debug
            // std::cout << "Distance x: " << dx << " y: " << dy << " theta: " << new_theta << std::endl;

            b2Transform trans{b2Vec2(curr->x + dx, curr->y + dy),
                              b2Rot(new_theta)};

            b2Transform trans2{b2Vec2{(float) curr->x, (float)curr->y}, b2Rot{(float) (curr->theta)}};

            drawBox(painter, robotShape, trans, QColor(127, 0, 255));
        }
        current_dist += new_dist;
        curr_vertex_it++;
    }
}

void addNewPoint()
{

    // Generate a random vertex
    Vertex* vertex = createNewVertex();

    double closestDistance;
    Kpoint nearestNeighborPoint{vertex->x, vertex->y, vertex->theta};
    Vertex* nearest;
    Kpoint closestPoint;
    tree->nearestNeighbor(nearestNeighborPoint, closestDistance, closestPoint, nearest);
    assert(nearest != nullptr);

    if (!checkCollision(vertex, nearest))
    {

        insert_intermediary_vertices(vertex, nearest);
        tree->insert(Kpoint{vertex->x, vertex->y, vertex->theta}, vertex);
        if (goalBody->GetFixtureList()[0].TestPoint( b2Vec2{static_cast<float>(vertex->x),
                                                            static_cast<float>(vertex->y)}))
        {
            goal_points.emplace_back(vertex);
        }
        else{
            vertices.emplace_back(vertex);
        }
    }
    else{
        delete vertex;
    }
}

void Helper::paint(QPainter *painter, QPaintEvent *event, int elapsed)
{

    // Calculate time steps -- for dynamics
    static std::chrono::time_point last_time = std::chrono::high_resolution_clock::now();
    std::chrono::time_point current_time = std::chrono::high_resolution_clock::now();
    auto step = (current_time - last_time).count() / 1e9;
    last_time = current_time;
    if(!initialized){setUpWorld(event->rect()); return;}  // Skip the initialization pass

    // Save the painter state
    painter->save();

    //world->Step(step, velocityIterations, positionIterations);

    painter->fillRect(event->rect(), background);

    float height_ratio =  event->rect().height() / 10.0f;
    float width_ratio = event->rect().width() / (10.0f * aspect_ratio);

    // Set the window to use world coordinates with a window height of 10m
    QTransform new_coordinates(width_ratio, 0, 0, -1.0f*height_ratio, 0, event->rect().height());
    painter->setTransform(new_coordinates);

    drawBox(painter, *groundBody->GetFixtureList()[0].GetShape(), groundBody->GetTransform());
    drawBox(painter, *leftWall->GetFixtureList()[0].GetShape(), leftWall->GetTransform());
    drawBox(painter, *rightWall->GetFixtureList()[0].GetShape(), rightWall->GetTransform());
    drawBox(painter, *topWall->GetFixtureList()[0].GetShape(), topWall->GetTransform());
    drawBox(painter, *centerWall->GetFixtureList()[0].GetShape(), centerWall->GetTransform());
    drawBox(painter, *goalBody->GetFixtureList()[0].GetShape(), goalBody->GetTransform(), QColor(0,255,0));
    drawBox(painter, *center2Wall->GetFixtureList()[0].GetShape(), center2Wall->GetTransform());
    drawBox(painter, *center3Wall->GetFixtureList()[0].GetShape(), center3Wall->GetTransform());

    // Add new points if we don't have a path
    if(goal_points.empty())
    {
        for (int i = 0; i < 1e4; i++){addNewPoint();}
    }

    painter->save();

    // Draw All Vertices
    for (const auto& vertex : vertices)
    {
        if (vertex->p_vertex == nullptr){continue;}
        QPen pen;
        pen.setWidth(0.1f);
        pen.setColor(Qt::cyan);
        painter->setPen(pen);
        painter->drawLine(QPointF(vertex->x, vertex->y), QPointF(vertex->p_vertex->x, vertex->p_vertex->y));
    }

    // Simple smoothing of goal points -- see if points can be skipped
    if (!goal_points.empty())
    {
        auto* curr_search = goal_points[0];
        while(curr_search != nullptr && curr_search->p_vertex != nullptr)
        {
            auto* curr = curr_search->p_vertex;
            while(curr->p_vertex != nullptr && !checkCollision(curr->p_vertex,curr_search))
            {
                curr = curr->p_vertex;
            }
            curr_search->p_vertex = curr;
            curr_search = curr;
        }
    }

    // Draw the goal path
    for (const Vertex* vertex : goal_points)
    {
        auto curr_vertex = vertex;
        while(curr_vertex->p_vertex != nullptr) {
            QPen pen;
            pen.setWidth(0.1f);
            pen.setColor(Qt::green);
            painter->setPen(pen);
            painter->drawLine(QPointF(curr_vertex->x, curr_vertex->y),
                    QPointF(curr_vertex->p_vertex->x, curr_vertex->p_vertex->y));
            curr_vertex = curr_vertex->p_vertex;
        }
    }

    // Animate the objects path
    animateGoalPath(painter, commands->g_percent);

    painter->restore();
    painter->restore();
}
