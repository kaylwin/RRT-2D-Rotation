#include "helper.h"
#include <box2d/b2_world.h>
#include <box2d/b2_body.h>
#include <box2d/b2_polygon_shape.h>
#include <box2d/b2_fixture.h>
#include <iostream>
#include <box2d/b2_joint.h>
#include <box2d/b2_distance_joint.h>
#include <box2d/b2_gear_joint.h>
#include <box2d/b2_time_of_impact.h>
#include <memory>
#include <random>

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

b2Body* platform;
b2Body* leftWall;
b2Body* rightWall;
b2Body* topWall;
b2Body* centerWall;

b2Body* goalBody;

b2PolygonShape collisionShape;
b2Transform collisionTransform;

b2PolygonShape robotShape;
b2Transform robotTransform;

struct Vertex
{
    double x = 0;  // m
    double y = 0;  // m
    double theta = 0;  // rad
    uint32_t id;
    Vertex* p_vertex = nullptr;  // Previous vertex

    Vertex(double x, double y, double theta, Vertex* p_vertex):x(x), y(y), theta(theta), p_vertex(p_vertex)
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
private:
    static uint32_t m_id;
};
uint32_t Vertex::m_id = 0; // Definition for Vertex ID

std::vector<std::unique_ptr<Vertex>> vertices{};


void createNewVertex()
{
    std::linear_congruential_engine<std::uint_fast32_t, 48271, 0, 2147483647> engine;
    engine.seed(0);
    auto grand = [&engine]()->double{ return engine() / static_cast<double>(INT_MAX);};
    Vertex v{};
    v.x = engine() / grand() * WINDOW_HEIGHT * aspect_ratio;
    v.y = engine() / grand() * WINDOW_HEIGHT;
    v.theta = grand() * 2 * M_PI;
}

/**
 * Compares to verticies in SE(2) and establishes a distance metric
 * @param a Vertex 1
 * @param b Vertex  2
 * @return
 */
double compute_se2_metric(const Vertex& a, const Vertex& b)
{
    // Normalize the distance w/r to max distance on the map
    double norm_eu_dist = (a.x - b.x) * (a.x - b.x)
            + ((a.y - b.y) * (a.y - b.y));

    // Metric assumes robot pivots on 1m axis and that this is w/r to a point on the head of the robot

    // Obtain a Euclidean metric on SE(2)
    double diff_angle = a.theta - b.theta;

    if (diff_angle < 0)
    {
        diff_angle += 2*M_PI;
    }
    else if (diff_angle > 2*M_PI)
    {
        diff_angle -= 2*M_PI;
    }
    double theta_dist = abs(a.theta - b.theta);

    return norm_eu_dist + theta_dist;
}


/**
 * Compare distance to all verticies and find the minimum vertex
 * @param new_vertex vertex to add
 * @return nearest Vertex
 */
Vertex& nearest_vertex(Vertex& new_vertex)
{
    auto comp = [&new_vertex](std::unique_ptr<Vertex>& a, std::unique_ptr<Vertex>& b){
        return compute_se2_metric(new_vertex, *a) < compute_se2_metric(new_vertex, *b);
    };
    auto nearest = std::min_element(vertices.begin(), vertices.end(), comp);
    return **nearest;
}

/**
 * Test function used in order to check that the nearest vertex function is working as expected. This function should
 * find the nearest vertex using both the rotation and translation parameters.
 */
void test_nearest_vertex()
{
    // Check obvious near case
    std::unique_ptr<Vertex> v1 = std::make_unique<Vertex>(Vertex{1,1,M_PI / 4.0, nullptr});
    Vertex& near = nearest_vertex(*v1);

    vertices.emplace_back(std::move(v1));
    std::cout << "TEST: Step1 ID was: " << near.id << std::endl;

    std::unique_ptr<Vertex> v2 = std::make_unique<Vertex>(Vertex{.5,.5,M_PI / 2.0, nullptr});
    near = nearest_vertex(*v2);
    std::cout << "TEST: Step2 ID was: " << near.id << std::endl;
    vertices.emplace_back(std::move(v2));

    std::unique_ptr<Vertex> v3 = std::make_unique<Vertex>(Vertex{.8,.8,M_PI / 2.0, nullptr});
    near = nearest_vertex(*v3);
    std::cout << "TEST: Step3 ID was: " << near.id << std::endl;

    std::unique_ptr<Vertex> v4 = std::make_unique<Vertex>(Vertex{.9,.9,2*M_PI, nullptr});
    near = nearest_vertex(*v3);
    std::cout << "TEST: Step4 ID was: " << near.id << std::endl;
    //auto v2 = std::make_unique<Vertex>(Vertex{1,1,0, nullptr});

}

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

    // Platform
    b2BodyDef platformBodyDef;
    platformBodyDef.type = b2_dynamicBody;
    platformBodyDef.position.Set(10.0f, 5.0f);
    dynamicBox.SetAsBox(1.0f, 0.3f);
    platform = world->CreateBody(&platformBodyDef);
    fixtureDef.shape = &dynamicBox;
    fixtureDef.density = 10.0f;
    fixtureDef.friction = 0.7f;
    fixtureDef.restitution = 0.1f;
    platform->CreateFixture(&fixtureDef);


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
    centerDef.position.Set(WINDOW_HEIGHT * aspect_ratio/2.0, WINDOW_HEIGHT/2);
    centerWall = world->CreateBody(&centerDef);
    centerBox.SetAsBox(1.0, 3);
    fixtureDef.restitution = .01;
    fixtureDef.shape = &centerBox;
    fixtureDef.density = 1.0f;
    fixtureDef.friction = 0.1f;
    centerWall->CreateFixture(&fixtureDef);

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


    // Create Collision Region
    collisionShape.SetAsBox(1.0f, 1.0f);
    collisionTransform.p.Set(1.5, WINDOW_HEIGHT - 1.5);

    robotShape.SetAsBox(1.0, 0.3f);
    robotTransform.Set(b2Vec2{WINDOW_HEIGHT * aspect_ratio * 0.2f, WINDOW_HEIGHT / 2.0f}, 0);

    vertices.emplace_back(std::make_unique<Vertex>(Vertex{0, 0, 0, nullptr}));
    test_nearest_vertex();

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
 * Basic collison checking yielding whether two bodies are touching
 * @param shape
 * @param pos
 * @return
 */
bool checkEndCollison(b2Shape* shape, b2Transform& start_trans, b2Transform& end_trans)
{
    b2Sweep sweepA;
    sweepA.localCenter.SetZero();
    sweepA.c0 = start_trans.p;
    sweepA.c = end_trans.p;
    sweepA.a0 = start_trans.q.GetAngle();
    sweepA.a = end_trans.q.GetAngle();
    sweepA.alpha0 = 0;

    b2Fixture& platformFixture = platform->GetFixtureList()[0];
    b2Shape* platformShape = platformFixture.GetShape();
    const b2Transform& platformTransform = platform->GetTransform();
    b2Sweep sweepB;
    sweepB.localCenter.SetZero();
    sweepB.c0 = platformTransform.p;
    sweepB.c = platformTransform.p;
    sweepB.a = platformTransform.q.GetAngle();
    sweepB.a0 = platformTransform.q.GetAngle();
    sweepB.alpha0 = 0;


    b2TOIInput toiInput{};
    toiInput.sweepA = sweepA;
    toiInput.sweepB = sweepB;
    toiInput.proxyA.Set(shape, 0);
    toiInput.proxyB.Set(platformShape, 0);
    toiInput.tMax = 0;

    b2TOIOutput toiOutput{};

    b2TimeOfImpact(&toiOutput, &toiInput);
    //switch(toiOutput.state)
    //{

    //    case b2TOIOutput::e_unknown:
    //        std::cout << "State unknown" << std::endl;
    //        break;
    //    case b2TOIOutput::e_failed:
    //        std::cout << "State failed" << std::endl;
    //        break;
    //    case b2TOIOutput::e_overlapped:
    //        std::cout << "State overlapped" << std::endl;
    //        break;
    //    case b2TOIOutput::e_touching:
    //        std::cout << "State touching" << std::endl;
    //        break;
    //    case b2TOIOutput::e_separated:
    //        std::cout << "State separated" << std::endl;
    //        break;
    //}

    return toiOutput.state != b2TOIOutput::e_separated;
}
void Helper::paint(QPainter *painter, QPaintEvent *event, int elapsed)
{

    // Calculate time steps
    static std::chrono::time_point last_time = std::chrono::high_resolution_clock::now();
    std::chrono::time_point current_time = std::chrono::high_resolution_clock::now();
    auto step = (current_time - last_time).count() / 1e9;
    last_time = current_time;
    if(!initialized){setUpWorld(event->rect()); return;}  // Skip the initialization pass

    // Save the painter state
    painter->save();

    // Set the simulation
    world->Step(step, velocityIterations, positionIterations);
    // Create the world and mark that it is created

    painter->fillRect(event->rect(), background);

    gravity->Set(commands->gravity_x * 0.5f, commands->gravity_y * 0.5f);
    world->SetGravity(*gravity);

    float height_ratio =  event->rect().height() / 10.0f;
    float width_ratio = event->rect().width() / (10.0f * aspect_ratio);

    // Set the window to use world coordinates with a window height of 10m
    QTransform new_coordinates(width_ratio, 0, 0, -1.0f*height_ratio, 0, event->rect().height());
    painter->setTransform(new_coordinates);
    //gearA->ApplyTorque(10, true);

    platform->SetAwake(true);
    platform->ApplyTorque(.2, true);
    drawBox(painter, *platform->GetFixtureList()[0].GetShape(), platform->GetTransform(), QColor(255, 0, 0));
    drawBox(painter, *groundBody->GetFixtureList()[0].GetShape(), groundBody->GetTransform());
    drawBox(painter, *leftWall->GetFixtureList()[0].GetShape(), leftWall->GetTransform());
    drawBox(painter, *rightWall->GetFixtureList()[0].GetShape(), rightWall->GetTransform());
    drawBox(painter, *topWall->GetFixtureList()[0].GetShape(), topWall->GetTransform());
    drawBox(painter, *centerWall->GetFixtureList()[0].GetShape(), centerWall->GetTransform());
    drawBox(painter, *goalBody->GetFixtureList()[0].GetShape(), goalBody->GetTransform(), QColor(0,255,0));
    drawBox(painter, collisionShape, collisionTransform, QColor(0,0,255));
    drawBox(painter, robotShape, robotTransform, QColor(0,0,255));
   // drawBox(painter, pendulum, 1, 1);
    //drawBox(painter, groundBody, 1, 11);

    if(!checkEndCollison(&robotShape, robotTransform, robotTransform))
    {

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
    painter->restore();

    painter->restore();


}
