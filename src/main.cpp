#include "raylib.h"
#include "raymath.h"
#include <algorithm>
#include <cmath>
#include <vector>

struct Body
{
    Vector3 position;
    Vector3 velocity;
    float radius;
    float bounce;
    Color color;
    Vector3 start;
};

struct OrbitCamera
{
    Camera3D camera;
    float yaw;
    float pitch;
    float distance;
};

const float G = -9.81f;
const float REST_SPEED = 0.3f;
const float FLOOR_FRICTION = 1.5f;

const float MIN_PITCH = 0.05f;
const float MAX_PITCH = 1.5f;
const float MIN_DISTANCE = 2.0f;
const float MAX_DISTANCE = 100.0f;
const float MOUSE_SENSITIVITY = 0.005f;
const float ZOOM_SPEED = 1.0f;

const float HOLD_TO_CLEAR = 0.7f;

OrbitCamera CreateOrbitCamera()
{
    OrbitCamera orbit = {};
    orbit.camera.target = {0.0f, 0.0f, 0.0f};
    orbit.camera.up = {0.0f, 1.0f, 0.0f};
    orbit.camera.fovy = 45.0f;
    orbit.camera.projection = CAMERA_PERSPECTIVE;
    orbit.yaw = 0.8f;
    orbit.pitch = 0.5f;
    orbit.distance = 18.0f;
    return orbit;
}

void UpdateOrbitCamera(OrbitCamera &orbit)
{
    if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT))
    {
        Vector2 delta = GetMouseDelta();
        orbit.yaw -= delta.x * MOUSE_SENSITIVITY;
        orbit.pitch += delta.y * MOUSE_SENSITIVITY;
    }
    orbit.distance -= GetMouseWheelMove() * ZOOM_SPEED;

    orbit.pitch = std::clamp(orbit.pitch, MIN_PITCH, MAX_PITCH);
    orbit.distance = std::clamp(orbit.distance, MIN_DISTANCE, MAX_DISTANCE);

    const Vector3 &target = orbit.camera.target;
    orbit.camera.position.x = target.x + orbit.distance * cosf(orbit.pitch) * sinf(orbit.yaw);
    orbit.camera.position.y = target.y + orbit.distance * sinf(orbit.pitch);
    orbit.camera.position.z = target.z + orbit.distance * cosf(orbit.pitch) * cosf(orbit.yaw);
}

Body MakeBody(Vector3 position, float radius, float bounce, Color color)
{
    Body b = {};
    b.position = position;
    b.velocity = {0.0f, 0.0f, 0.0f};
    b.radius = radius;
    b.bounce = bounce;
    b.color = color;
    b.start = position;
    return b;
}

std::vector<Body> CreateBodies()
{
    return {MakeBody({0.0f, 5.0f, 0.0f}, 0.5f, 0.8f, RED)};
}

float RandomFloat(float min, float max)
{
    return min + (max - min) * GetRandomValue(0, 1000) / 1000.0f;
}

Body CreateRandomBody()
{
    Vector3 position = {RandomFloat(-8.0f, 8.0f), RandomFloat(5.0f, 10.0f), RandomFloat(-8.0f, 8.0f)};
    Color color = {(unsigned char)GetRandomValue(50, 255),
                   (unsigned char)GetRandomValue(50, 255),
                   (unsigned char)GetRandomValue(50, 255),
                   255};
    return MakeBody(position, RandomFloat(0.2f, 1.0f), RandomFloat(0.3f, 0.9f), color);
}

void ResetBodies(std::vector<Body> &bodies)
{
    for (Body &b : bodies)
    {
        b.position = b.start;
        b.velocity = {0.0f, 0.0f, 0.0f};
    }
}

float Mass(const Body &b)
{
    return b.radius * b.radius * b.radius;
}

void CollideBodies(Body &a, Body &b)
{
    Vector3 delta = Vector3Subtract(b.position, a.position);
    float dist = Vector3Length(delta);
    float minDist = a.radius + b.radius;

    if (dist >= minDist)
    {
        return;
    }

    Vector3 normal = dist > 0.0001f ? Vector3Scale(delta, 1.0f / dist) : Vector3{0.0f, 1.0f, 0.0f};

    float massA = Mass(a);
    float massB = Mass(b);
    float totalMass = massA + massB;

    float overlap = minDist - dist;
    a.position = Vector3Subtract(a.position, Vector3Scale(normal, overlap * massB / totalMass));
    b.position = Vector3Add(b.position, Vector3Scale(normal, overlap * massA / totalMass));

    Vector3 relativeVelocity = Vector3Subtract(b.velocity, a.velocity);
    float approachSpeed = Vector3DotProduct(relativeVelocity, normal);
    if (approachSpeed > 0.0f)
    {
        return;
    }

    float bounce = std::min(a.bounce, b.bounce);
    if (-approachSpeed < REST_SPEED)
    {
        bounce = 0.0f;
    }
    float impulse = -(1.0f + bounce) * approachSpeed / (1.0f / massA + 1.0f / massB);

    a.velocity = Vector3Subtract(a.velocity, Vector3Scale(normal, impulse / massA));
    b.velocity = Vector3Add(b.velocity, Vector3Scale(normal, impulse / massB));
}

void CollideFloor(Body &b, float dt)
{
    if (b.position.y - b.radius >= 0.0f)
    {
        return;
    }

    b.position.y = b.radius;
    if (b.velocity.y < 0.0f)
    {
        b.velocity.y = -b.velocity.y * b.bounce;
        if (b.velocity.y < REST_SPEED)
        {
            b.velocity.y = 0.0f;
        }
    }

    float slowdown = std::max(1.0f - FLOOR_FRICTION * dt, 0.0f);
    b.velocity.x *= slowdown;
    b.velocity.z *= slowdown;
}

void UpdatePhysics(std::vector<Body> &bodies, float dt)
{
    for (Body &b : bodies)
    {
        b.velocity.y += G * dt;
        b.position = Vector3Add(b.position, Vector3Scale(b.velocity, dt));
    }

    for (size_t i = 0; i < bodies.size(); i++)
    {
        for (size_t j = i + 1; j < bodies.size(); j++)
        {
            CollideBodies(bodies[i], bodies[j]);
        }
    }

    for (Body &b : bodies)
    {
        CollideFloor(b, dt);
    }
}

void DrawScene(const Camera3D &camera, const std::vector<Body> &bodies, bool paused)
{
    const Color bg = {40, 40, 40, 255};

    BeginDrawing();
    ClearBackground(bg);

    BeginMode3D(camera);
    DrawGrid(20, 1.0f);
    for (const Body &b : bodies)
    {
        DrawSphere(b.position, b.radius, b.color);
    }
    EndMode3D();

    DrawText(TextFormat("Bodies: %d", (int)bodies.size()), 10, 10, 20, WHITE);
    DrawText("SPACE - pause/play   R - reset   N - new ball   Q - delete (hold - delete all)",
             10, 40, 20, LIGHTGRAY);

    if (paused)
    {
        const char *text = "PAUSED";
        int fontSize = 40;
        int x = (GetScreenWidth() - MeasureText(text, fontSize)) / 2;
        DrawText(text, x, 20, fontSize, YELLOW);
    }

    EndDrawing();
}

int main()
{
    InitWindow(1280, 720, "P in 3D");
    SetTargetFPS(60);

    OrbitCamera orbit = CreateOrbitCamera();

    std::vector<Body> bodies = CreateBodies();
    bool paused = false;
    float qHoldTime = 0.0f;

    while (!WindowShouldClose())
    {
        float dt = GetFrameTime();

        if (IsKeyPressed(KEY_SPACE))
        {
            paused = !paused;
        }
        if (IsKeyPressed(KEY_R))
        {
            ResetBodies(bodies);
        }
        if (IsKeyPressed(KEY_N))
        {
            bodies.push_back(CreateRandomBody());
        }

        if (IsKeyPressed(KEY_Q) && !bodies.empty())
        {
            bodies.pop_back();
        }
        if (IsKeyDown(KEY_Q))
        {
            qHoldTime += dt;
            if (qHoldTime >= HOLD_TO_CLEAR)
            {
                bodies.clear();
            }
        }
        else
        {
            qHoldTime = 0.0f;
        }

        UpdateOrbitCamera(orbit);
        if (!paused)
        {
            UpdatePhysics(bodies, dt);
        }
        DrawScene(orbit.camera, bodies, paused);
    }

    CloseWindow();
    return 0;
}
