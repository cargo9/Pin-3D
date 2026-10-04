#include "raylib.h"
#include "raymath.h"
#include "rlgl.h"
#include <algorithm>
#include <cmath>
#include <vector>

enum class Shape
{
    Sphere,
    Cube,
    Box,
    Pyramid3,
    Pyramid4,
    Ellipsoid,
    Cylinder,
    Cone,
    Capsule,
    Count
};

const int SHAPE_COUNT = (int)Shape::Count;

struct Body
{
    Shape shape;
    Vector3 position; 
    Vector3 velocity;
    Vector3 size; 
    float bounce;
    Color color;
    Vector3 start;
};

struct FreeCamera
{
    Camera3D camera;
    float yaw;
    float pitch;
    bool looking;   
    Vector2 savedMouse; 
};

struct SpawnSettings
{
    Shape shape;
    float radius;
    float height;
    float width;
    float depth;
    float bounce;
};

struct Param
{
    const char *name;
    float SpawnSettings::*field;
    float min;
    float max;
    const char *unit;
};

struct Drag
{
    bool active;
    Vector3 target;   
    Vector3 offset;  
    Vector3 velocity; 
};

struct Contact
{
    Vector3 normal; 
    float overlap;
};

const float G = -9.81f;
const float REST_SPEED = 0.3f;
const float FLOOR_FRICTION = 1.5f;
const float DENSITY = 3.0f / (4.0f * PI); 

const float MAX_PITCH = 1.5f;
const float MOUSE_SENSITIVITY = 0.003f;
const float FLY_SPEED = 8.0f;
const float FAST_MULTIPLIER = 3.0f;
const float WHEEL_STEP = 1.5f;
const float MIN_CAMERA_HEIGHT = 0.2f;

const float HOLD_TO_CLEAR = 0.7f;

const int NO_SELECTION = -1;

const float DRAG_SMOOTHING = 0.3f;
const float MAX_THROW_SPEED = 30.0f;
const float LIFT_STEP = 0.5f;
const float MAX_DRAG_DISTANCE = 60.0f;

const int PANEL_HEIGHT = 112;
const int PANEL_PADDING = 10;
const int BUTTON_HEIGHT = 30;
const int BUTTON_GAP = 6;
const int SLIDER_WIDTH = 240;
const int SLIDER_GAP = 30;
const int SPAWN_BUTTON_WIDTH = 130;

const char *ShapeName(Shape shape)
{
    switch (shape)
    {
    case Shape::Sphere: return "Sphere";
    case Shape::Cube: return "Cube";
    case Shape::Box: return "Box";
    case Shape::Pyramid3: return "Pyramid 3";
    case Shape::Pyramid4: return "Pyramid 4";
    case Shape::Ellipsoid: return "Ellipsoid";
    case Shape::Cylinder: return "Cylinder";
    case Shape::Cone: return "Cone";
    case Shape::Capsule: return "Capsule";
    default: return "?";
    }
}

std::vector<Param> GetParams(Shape shape)
{
    const Param radius = {"Radius", &SpawnSettings::radius, 0.2f, 2.0f, " m"};
    const Param height = {"Height", &SpawnSettings::height, 0.3f, 4.0f, " m"};
    const Param width = {"Width", &SpawnSettings::width, 0.3f, 4.0f, " m"};
    const Param depth = {"Depth", &SpawnSettings::depth, 0.3f, 4.0f, " m"};
    const Param bounce = {"Bounce", &SpawnSettings::bounce, 0.0f, 0.95f, ""};

    switch (shape)
    {
    case Shape::Sphere: return {radius, bounce};
    case Shape::Cube: return {{"Size", &SpawnSettings::width, 0.3f, 4.0f, " m"}, bounce};
    case Shape::Box:
    case Shape::Ellipsoid: return {width, height, depth, bounce};
    case Shape::Pyramid4: return {{"Base", &SpawnSettings::width, 0.3f, 4.0f, " m"}, height, bounce};
    case Shape::Pyramid3:
    case Shape::Cylinder:
    case Shape::Cone:
    case Shape::Capsule: return {radius, height, bounce};
    default: return {bounce};
    }
}

Vector3 SpawnSize(const SpawnSettings &s)
{
    float d = 2.0f * s.radius;
    switch (s.shape)
    {
    case Shape::Sphere: return {d, d, d};
    case Shape::Cube: return {s.width, s.width, s.width};
    case Shape::Box:
    case Shape::Ellipsoid: return {s.width, s.height, s.depth};
    case Shape::Pyramid4: return {s.width, s.height, s.width};
    case Shape::Capsule: return {d, std::max(s.height, d), d};
    default: return {d, s.height, d};
    }
}

Vector3 CameraForward(const FreeCamera &cam)
{
    return {cosf(cam.pitch) * sinf(cam.yaw), sinf(cam.pitch), cosf(cam.pitch) * cosf(cam.yaw)};
}

FreeCamera CreateFreeCamera()
{
    FreeCamera cam = {};
    cam.camera.position = {11.0f, 8.6f, 11.0f};
    cam.camera.up = {0.0f, 1.0f, 0.0f};
    cam.camera.fovy = 45.0f;
    cam.camera.projection = CAMERA_PERSPECTIVE;
    cam.yaw = atan2f(-cam.camera.position.x, -cam.camera.position.z);
    cam.pitch = -0.5f;
    cam.camera.target = Vector3Add(cam.camera.position, CameraForward(cam));
    return cam;
}

Vector2 PointerPosition(const FreeCamera &cam)
{
    return cam.looking ? cam.savedMouse : GetMousePosition();
}

void UpdateFreeCamera(FreeCamera &cam, float dt, bool wheelBusy)
{
    if (IsMouseButtonPressed(MOUSE_BUTTON_RIGHT))
    {
        cam.looking = true;
        cam.savedMouse = GetMousePosition();
        DisableCursor();
    }
    else if (cam.looking && IsMouseButtonDown(MOUSE_BUTTON_RIGHT))
    {
        Vector2 delta = GetMouseDelta();
        cam.yaw -= delta.x * MOUSE_SENSITIVITY;
        cam.pitch -= delta.y * MOUSE_SENSITIVITY;
        cam.pitch = std::clamp(cam.pitch, -MAX_PITCH, MAX_PITCH);
    }
    else if (cam.looking)
    {
        cam.looking = false;
        EnableCursor();
        SetMousePosition((int)cam.savedMouse.x, (int)cam.savedMouse.y);
    }

    Vector3 forward = CameraForward(cam);
    Vector3 right = {-cosf(cam.yaw), 0.0f, sinf(cam.yaw)};
    Vector3 up = {0.0f, 1.0f, 0.0f};

    Vector3 move = {0.0f, 0.0f, 0.0f};
    if (IsKeyDown(KEY_W)) move = Vector3Add(move, forward);
    if (IsKeyDown(KEY_S)) move = Vector3Subtract(move, forward);
    if (IsKeyDown(KEY_D)) move = Vector3Add(move, right);
    if (IsKeyDown(KEY_A)) move = Vector3Subtract(move, right);
    if (IsKeyDown(KEY_E)) move = Vector3Add(move, up);
    if (IsKeyDown(KEY_C)) move = Vector3Subtract(move, up);

    float speed = FLY_SPEED * (IsKeyDown(KEY_LEFT_SHIFT) ? FAST_MULTIPLIER : 1.0f);
    Vector3 &position = cam.camera.position;
    position = Vector3Add(position, Vector3Scale(Vector3Normalize(move), speed * dt));
    if (!wheelBusy)
    {
        position = Vector3Add(position, Vector3Scale(forward, GetMouseWheelMove() * WHEEL_STEP));
    }
    position.y = std::max(position.y, MIN_CAMERA_HEIGHT);

    cam.camera.target = Vector3Add(position, forward);
}

Body MakeBody(Shape shape, Vector3 position, Vector3 size, float bounce, Color color)
{
    Body b = {};
    b.shape = shape;
    b.position = position;
    b.velocity = {0.0f, 0.0f, 0.0f};
    b.size = size;
    b.bounce = bounce;
    b.color = color;
    b.start = position;
    return b;
}

std::vector<Body> CreateBodies()
{
    return {MakeBody(Shape::Sphere, {0.0f, 5.0f, 0.0f}, {1.0f, 1.0f, 1.0f}, 0.8f, RED)};
}

float RandomFloat(float min, float max)
{
    return min + (max - min) * GetRandomValue(0, 1000) / 1000.0f;
}

Body CreateRandomBody(const SpawnSettings &settings)
{
    Vector3 position = {RandomFloat(-8.0f, 8.0f), RandomFloat(5.0f, 10.0f), RandomFloat(-8.0f, 8.0f)};
    Color color = {(unsigned char)GetRandomValue(50, 255),
                   (unsigned char)GetRandomValue(50, 255),
                   (unsigned char)GetRandomValue(50, 255),
                   255};
    return MakeBody(settings.shape, position, SpawnSize(settings), settings.bounce, color);
}

void ResetBodies(std::vector<Body> &bodies)
{
    for (Body &b : bodies)
    {
        b.position = b.start;
        b.velocity = {0.0f, 0.0f, 0.0f};
    }
}

float Volume(const Body &b)
{
    const Vector3 &s = b.size;
    float r = s.x / 2.0f;
    switch (b.shape)
    {
    case Shape::Sphere: return 4.0f / 3.0f * PI * r * r * r;
    case Shape::Ellipsoid: return PI / 6.0f * s.x * s.y * s.z;
    case Shape::Cube:
    case Shape::Box: return s.x * s.y * s.z;
    case Shape::Pyramid3: return sqrtf(3.0f) / 4.0f * r * r * s.y;
    case Shape::Pyramid4: return s.x * s.y * s.z / 3.0f;
    case Shape::Cylinder: return PI * r * r * s.y;
    case Shape::Cone: return PI * r * r * s.y / 3.0f;
    case Shape::Capsule: return PI * r * r * (s.y - 2.0f * r) + 4.0f / 3.0f * PI * r * r * r;
    default: return s.x * s.y * s.z;
    }
}

float Mass(const Body &b)
{
    return Volume(b) * DENSITY;
}

bool IsRound(const Body &b)
{
    return b.shape == Shape::Sphere;
}

bool BoxBoxContact(Vector3 posA, Vector3 halfA, Vector3 posB, Vector3 halfB, Contact &contact)
{
    Vector3 d = Vector3Subtract(posB, posA);
    float ox = halfA.x + halfB.x - fabsf(d.x);
    float oy = halfA.y + halfB.y - fabsf(d.y);
    float oz = halfA.z + halfB.z - fabsf(d.z);
    if (ox <= 0.0f || oy <= 0.0f || oz <= 0.0f)
    {
        return false;
    }

    if (ox <= oy && ox <= oz)
    {
        contact = {{d.x < 0.0f ? -1.0f : 1.0f, 0.0f, 0.0f}, ox};
    }
    else if (oy <= oz)
    {
        contact = {{0.0f, d.y < 0.0f ? -1.0f : 1.0f, 0.0f}, oy};
    }
    else
    {
        contact = {{0.0f, 0.0f, d.z < 0.0f ? -1.0f : 1.0f}, oz};
    }
    return true;
}

bool SphereBoxContact(const Body &sphere, const Body &box, Contact &contact)
{
    float r = sphere.size.x / 2.0f;
    Vector3 half = Vector3Scale(box.size, 0.5f);
    Vector3 closest = Vector3Clamp(sphere.position, Vector3Subtract(box.position, half),
                                   Vector3Add(box.position, half));
    Vector3 delta = Vector3Subtract(closest, sphere.position);
    float dist = Vector3Length(delta);

    if (dist >= r)
    {
        return false;
    }
    if (dist > 0.0001f)
    {
        contact = {Vector3Scale(delta, 1.0f / dist), r - dist};
        return true;
    }
    return BoxBoxContact(sphere.position, {r, r, r}, box.position, half, contact);
}

bool SphereSphereContact(const Body &a, const Body &b, Contact &contact)
{
    Vector3 delta = Vector3Subtract(b.position, a.position);
    float dist = Vector3Length(delta);
    float minDist = (a.size.x + b.size.x) / 2.0f;

    if (dist >= minDist)
    {
        return false;
    }
    Vector3 normal = dist > 0.0001f ? Vector3Scale(delta, 1.0f / dist) : Vector3{0.0f, 1.0f, 0.0f};
    contact = {normal, minDist - dist};
    return true;
}

bool FindContact(const Body &a, const Body &b, Contact &contact)
{
    if (IsRound(a) && IsRound(b))
    {
        return SphereSphereContact(a, b, contact);
    }
    if (IsRound(a))
    {
        return SphereBoxContact(a, b, contact);
    }
    if (IsRound(b))
    {
        if (!SphereBoxContact(b, a, contact))
        {
            return false;
        }
        contact.normal = Vector3Negate(contact.normal);
        return true;
    }
    return BoxBoxContact(a.position, Vector3Scale(a.size, 0.5f), b.position, Vector3Scale(b.size, 0.5f),
                         contact);
}

void CollideBodies(Body &a, Body &b)
{
    Contact contact;
    if (!FindContact(a, b, contact))
    {
        return;
    }
    const Vector3 &normal = contact.normal;

    float massA = Mass(a);
    float massB = Mass(b);
    float totalMass = massA + massB;

    a.position = Vector3Subtract(a.position, Vector3Scale(normal, contact.overlap * massB / totalMass));
    b.position = Vector3Add(b.position, Vector3Scale(normal, contact.overlap * massA / totalMass));

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
    float halfHeight = b.size.y / 2.0f;
    if (b.position.y - halfHeight >= 0.0f)
    {
        return;
    }

    b.position.y = halfHeight;
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

bool RayHitBody(Ray ray, const Body &b, float &distance)
{
    RayCollision hit = {};
    if (b.shape == Shape::Sphere)
    {
        hit = GetRayCollisionSphere(ray, b.position, b.size.x / 2.0f);
    }
    else if (b.shape == Shape::Ellipsoid)
    {
        Vector3 radii = Vector3Scale(b.size, 0.5f);
        Ray local = {Vector3Divide(Vector3Subtract(ray.position, b.position), radii),
                     Vector3Normalize(Vector3Divide(ray.direction, radii))};
        hit = GetRayCollisionSphere(local, {0.0f, 0.0f, 0.0f}, 1.0f);
        if (hit.hit)
        {
            Vector3 point = Vector3Add(b.position, Vector3Multiply(hit.point, radii));
            hit.distance = Vector3Distance(ray.position, point);
        }
    }
    else
    {
        Vector3 half = Vector3Scale(b.size, 0.5f);
        hit = GetRayCollisionBox(ray, {Vector3Subtract(b.position, half), Vector3Add(b.position, half)});
    }

    distance = hit.distance;
    return hit.hit;
}

int PickBody(const Camera3D &camera, Vector2 pointer, const std::vector<Body> &bodies, Vector3 &hitPoint)
{
    Ray ray = GetScreenToWorldRay(pointer, camera);

    int picked = NO_SELECTION;
    float closest = 0.0f;
    for (size_t i = 0; i < bodies.size(); i++)
    {
        float distance;
        if (RayHitBody(ray, bodies[i], distance) && (picked == NO_SELECTION || distance < closest))
        {
            picked = (int)i;
            closest = distance;
        }
    }
    hitPoint = Vector3Add(ray.position, Vector3Scale(ray.direction, closest));
    return picked;
}

bool IsMouseOverPanel();

Drag StartDrag(const Body &b, Vector3 grabPoint)
{
    Drag drag = {};
    drag.active = true;
    drag.target = b.position;
    drag.offset = Vector3Subtract(b.position, grabPoint);
    drag.velocity = {0.0f, 0.0f, 0.0f};
    return drag;
}


void UpdateDrag(Drag &drag, const Body &b, const Camera3D &camera, Vector2 pointer, float dt)
{
    Vector3 target = drag.target;
    target.y += GetMouseWheelMove() * LIFT_STEP;
    target.y = std::max(target.y, b.size.y / 2.0f);

    float planeY = target.y - drag.offset.y;
    Ray ray = GetScreenToWorldRay(pointer, camera);
    if (fabsf(ray.direction.y) > 0.0001f)
    {
        float t = (planeY - ray.position.y) / ray.direction.y;
        if (t > 0.0f && t < MAX_DRAG_DISTANCE)
        {
            target.x = ray.position.x + ray.direction.x * t + drag.offset.x;
            target.z = ray.position.z + ray.direction.z * t + drag.offset.z;
        }
    }

    if (dt > 0.0f)
    {
        Vector3 velocity = Vector3Scale(Vector3Subtract(target, drag.target), 1.0f / dt);
        drag.velocity = Vector3Lerp(drag.velocity, velocity, DRAG_SMOOTHING);
        drag.velocity = Vector3ClampValue(drag.velocity, 0.0f, MAX_THROW_SPEED);
    }
    drag.target = target;
}

void UpdateCursor(const FreeCamera &cam, const std::vector<Body> &bodies, const Drag &drag, int &cursor)
{
    Vector3 hitPoint;
    int wanted = MOUSE_CURSOR_DEFAULT;
    if (drag.active)
    {
        wanted = MOUSE_CURSOR_RESIZE_ALL;
    }
    else if (!cam.looking && !IsMouseOverPanel() &&
             PickBody(cam.camera, GetMousePosition(), bodies, hitPoint) != NO_SELECTION)
    {
        wanted = MOUSE_CURSOR_POINTING_HAND;
    }

    if (wanted != cursor)
    {
        SetMouseCursor(wanted);
        cursor = wanted;
    }
}

void HoldDragged(const Drag &drag, Body &b, bool paused)
{
    b.position = drag.target;
    b.velocity = paused ? Vector3{0.0f, 0.0f, 0.0f} : drag.velocity;
}

Rectangle ShapeButtonRect(int i)
{
    float top = (float)(GetScreenHeight() - PANEL_HEIGHT + PANEL_PADDING);
    float width = (GetScreenWidth() - 2.0f * PANEL_PADDING - BUTTON_GAP * (SHAPE_COUNT - 1)) / SHAPE_COUNT;
    return {PANEL_PADDING + i * (width + BUTTON_GAP), top, width, (float)BUTTON_HEIGHT};
}

Rectangle SliderRect(int i)
{
    float top = (float)(GetScreenHeight() - PANEL_HEIGHT + PANEL_PADDING + BUTTON_HEIGHT + 40);
    return {(float)(PANEL_PADDING + i * (SLIDER_WIDTH + SLIDER_GAP)), top, (float)SLIDER_WIDTH, 14.0f};
}

Rectangle SpawnButtonRect()
{
    float top = (float)(GetScreenHeight() - PANEL_HEIGHT + PANEL_PADDING + BUTTON_HEIGHT + 14);
    return {(float)(GetScreenWidth() - PANEL_PADDING - SPAWN_BUTTON_WIDTH), top, (float)SPAWN_BUTTON_WIDTH, 40.0f};
}

bool IsMouseOverPanel()
{
    return GetMouseY() >= GetScreenHeight() - PANEL_HEIGHT;
}

bool UpdateSpawnPanel(SpawnSettings &settings, int &activeSlider)
{
    Vector2 mouse = GetMousePosition();
    bool pressed = IsMouseButtonPressed(MOUSE_BUTTON_LEFT);

    if (pressed)
    {
        for (int i = 0; i < SHAPE_COUNT; i++)
        {
            if (CheckCollisionPointRec(mouse, ShapeButtonRect(i)))
            {
                settings.shape = (Shape)i;
            }
        }
    }

    std::vector<Param> params = GetParams(settings.shape);
    if (pressed)
    {
        for (size_t i = 0; i < params.size(); i++)
        {
            Rectangle grab = SliderRect((int)i);
            grab.y -= 8.0f;
            grab.height += 16.0f;
            if (CheckCollisionPointRec(mouse, grab))
            {
                activeSlider = (int)i;
            }
        }
    }
    if (!IsMouseButtonDown(MOUSE_BUTTON_LEFT) || activeSlider >= (int)params.size())
    {
        activeSlider = NO_SELECTION;
    }
    if (activeSlider != NO_SELECTION)
    {
        const Param &p = params[activeSlider];
        Rectangle bar = SliderRect(activeSlider);
        float t = std::clamp((mouse.x - bar.x) / bar.width, 0.0f, 1.0f);
        settings.*p.field = p.min + t * (p.max - p.min);
    }

    return pressed && CheckCollisionPointRec(mouse, SpawnButtonRect());
}

void DrawButton(Rectangle rect, const char *text, bool active)
{
    bool hover = CheckCollisionPointRec(GetMousePosition(), rect);
    Color fill = active ? YELLOW : (hover ? GRAY : DARKGRAY);
    Color textColor = active ? BLACK : WHITE;
    const int fontSize = 20;

    DrawRectangleRec(rect, fill);
    int textX = (int)(rect.x + (rect.width - MeasureText(text, fontSize)) / 2);
    int textY = (int)(rect.y + (rect.height - fontSize) / 2);
    DrawText(text, textX, textY, fontSize, textColor);
}

void DrawSpawnPanel(const SpawnSettings &settings, int activeSlider)
{
    int top = GetScreenHeight() - PANEL_HEIGHT;
    DrawRectangle(0, top, GetScreenWidth(), PANEL_HEIGHT, Fade(BLACK, 0.6f));
    DrawLine(0, top, GetScreenWidth(), top, GRAY);

    for (int i = 0; i < SHAPE_COUNT; i++)
    {
        DrawButton(ShapeButtonRect(i), ShapeName((Shape)i), settings.shape == (Shape)i);
    }

    std::vector<Param> params = GetParams(settings.shape);
    for (size_t i = 0; i < params.size(); i++)
    {
        const Param &p = params[i];
        float value = settings.*p.field;
        Rectangle bar = SliderRect((int)i);
        float t = (value - p.min) / (p.max - p.min);

        DrawText(TextFormat("%s: %.2f%s", p.name, value, p.unit), (int)bar.x, (int)bar.y - 24, 20, WHITE);
        DrawRectangleRec(bar, DARKGRAY);
        DrawRectangleRec({bar.x, bar.y, bar.width * t, bar.height}, LIGHTGRAY);
        Color knob = (int)i == activeSlider ? YELLOW : WHITE;
        DrawRectangleRec({bar.x + bar.width * t - 5.0f, bar.y - 4.0f, 10.0f, bar.height + 8.0f}, knob);
    }

    DrawButton(SpawnButtonRect(), "Spawn (N)", false);
}

void DrawCone(Vector3 base, float radiusTop, float radius, float height, int slices, Color color, bool wires)
{
    if (wires)
    {
        DrawCylinderWires(base, radiusTop, radius, height, slices, color);
    }
    else
    {
        DrawCylinder(base, radiusTop, radius, height, slices, color);
    }
}

void DrawShape(Shape shape, Vector3 position, Vector3 size, Color color, bool wires)
{
    float radius = size.x / 2.0f;
    Vector3 base = {position.x, position.y - size.y / 2.0f, position.z};

    switch (shape)
    {
    case Shape::Sphere:
        if (wires)
        {
            DrawSphereWires(position, radius, 12, 16, color);
        }
        else
        {
            DrawSphere(position, radius, color);
        }
        break;
    case Shape::Cube:
    case Shape::Box:
        if (wires)
        {
            DrawCubeWiresV(position, size, color);
        }
        else
        {
            DrawCubeV(position, size, color);
        }
        break;
    case Shape::Pyramid3:
        DrawCone(base, 0.0f, radius, size.y, 3, color, wires);
        break;
    case Shape::Pyramid4:
        rlPushMatrix();
        rlTranslatef(base.x, base.y, base.z);
        rlRotatef(45.0f, 0.0f, 1.0f, 0.0f);
        DrawCone({0.0f, 0.0f, 0.0f}, 0.0f, size.x / sqrtf(2.0f), size.y, 4, color, wires);
        rlPopMatrix();
        break;
    case Shape::Ellipsoid:
        rlPushMatrix();
        rlTranslatef(position.x, position.y, position.z);
        rlScalef(size.x / 2.0f, size.y / 2.0f, size.z / 2.0f);
        if (wires)
        {
            DrawSphereWires({0.0f, 0.0f, 0.0f}, 1.0f, 12, 16, color);
        }
        else
        {
            DrawSphere({0.0f, 0.0f, 0.0f}, 1.0f, color);
        }
        rlPopMatrix();
        break;
    case Shape::Cylinder:
        DrawCone(base, radius, radius, size.y, 16, color, wires);
        break;
    case Shape::Cone:
        DrawCone(base, 0.0f, radius, size.y, 16, color, wires);
        break;
    case Shape::Capsule:
    {
        float offset = size.y / 2.0f - radius;
        Vector3 bottom = {position.x, position.y - offset, position.z};
        Vector3 top = {position.x, position.y + offset, position.z};
        if (wires)
        {
            DrawCapsuleWires(bottom, top, radius, 16, 6, color);
        }
        else
        {
            DrawCapsule(bottom, top, radius, 16, 6, color);
        }
        break;
    }
    default:
        break;
    }
}

bool HasEdges(Shape shape)
{
    return shape != Shape::Sphere && shape != Shape::Ellipsoid && shape != Shape::Capsule;
}

void DrawSelectedPanel(const Body &b)
{
    const int width = 300;
    const int height = 194;
    const int x = GetScreenWidth() - width - 10;
    const int y = 10;
    const int fontSize = 20;
    const int line = 24;

    DrawRectangle(x, y, width, height, Fade(BLACK, 0.6f));
    DrawRectangleLines(x, y, width, height, YELLOW);

    int textX = x + 12;
    int textY = y + 10;
    DrawText("Selected body", textX, textY, fontSize, YELLOW);
    textY += line + 4;
    DrawText(TextFormat("Shape:   %s", ShapeName(b.shape)), textX, textY, fontSize, WHITE);
    textY += line;
    if (b.shape == Shape::Sphere)
    {
        DrawText(TextFormat("Radius:  %.2f m", b.size.x / 2.0f), textX, textY, fontSize, WHITE);
    }
    else
    {
        DrawText(TextFormat("Size:    %.2f x %.2f x %.2f m", b.size.x, b.size.y, b.size.z), textX, textY,
                 fontSize, WHITE);
    }
    textY += line;
    DrawText(TextFormat("Mass:    %.2f", Mass(b)), textX, textY, fontSize, WHITE);
    textY += line;
    DrawText(TextFormat("Speed:   %.2f m/s", Vector3Length(b.velocity)), textX, textY, fontSize, WHITE);
    textY += line;
    DrawText(TextFormat("Height:  %.2f m", b.position.y - b.size.y / 2.0f), textX, textY, fontSize, WHITE);
    textY += line;
    DrawText(TextFormat("Bounce:  %.2f", b.bounce), textX, textY, fontSize, WHITE);
}

void DrawScene(const Camera3D &camera, const std::vector<Body> &bodies, int selected, bool paused,
               const SpawnSettings &settings, int activeSlider)
{
    const Color bg = {40, 40, 40, 255};

    BeginDrawing();
    ClearBackground(bg);

    BeginMode3D(camera);
    DrawGrid(20, 1.0f);
    for (const Body &b : bodies)
    {
        DrawShape(b.shape, b.position, b.size, b.color, false);
        if (HasEdges(b.shape))
        {
            DrawShape(b.shape, b.position, b.size, ColorBrightness(b.color, -0.5f), true);
        }
    }
    if (selected != NO_SELECTION)
    {
        const Body &b = bodies[selected];
        DrawShape(b.shape, b.position, Vector3Scale(b.size, 1.08f), YELLOW, true);
        Vector3 bottom = {b.position.x, b.position.y - b.size.y / 2.0f, b.position.z};
        Vector3 floor = {b.position.x, 0.01f, b.position.z};
        DrawLine3D(bottom, floor, YELLOW);
        DrawCircle3D(floor, std::max(b.size.x, b.size.z) / 2.0f, {1.0f, 0.0f, 0.0f}, 90.0f, YELLOW);
    }
    EndMode3D();

    DrawText(TextFormat("Bodies: %d", (int)bodies.size()), 10, 10, 20, WHITE);
    DrawText("SPACE - pause   R - reset   N - spawn   Q - delete last (hold - all)", 10, 40, 20, LIGHTGRAY);
    DrawText("LMB - select / drag (wheel - lift, release moving - throw)   DEL - delete", 10, 65, 20, LIGHTGRAY);
    DrawText("RMB - look   WASD - fly   E/C - up/down   SHIFT - fast   wheel - forward", 10, 90, 20, LIGHTGRAY);

    if (selected != NO_SELECTION)
    {
        DrawSelectedPanel(bodies[selected]);
    }

    DrawSpawnPanel(settings, activeSlider);

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

    FreeCamera cam = CreateFreeCamera();
    int cursor = MOUSE_CURSOR_DEFAULT;

    std::vector<Body> bodies = CreateBodies();
    int selected = NO_SELECTION;
    Drag drag = {};
    bool paused = false;
    float qHoldTime = 0.0f;

    SpawnSettings settings = {Shape::Sphere, 0.5f, 1.0f, 1.0f, 1.0f, 0.7f};
    int activeSlider = NO_SELECTION;

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

        bool spawnClicked = UpdateSpawnPanel(settings, activeSlider);
        if (IsKeyPressed(KEY_N) || spawnClicked)
        {
            bodies.push_back(CreateRandomBody(settings));
        }

        if (IsKeyPressed(KEY_Q) && !bodies.empty())
        {
            bodies.pop_back();
            if (selected >= (int)bodies.size())
            {
                selected = NO_SELECTION;
            }
        }
        if (IsKeyDown(KEY_Q))
        {
            qHoldTime += dt;
            if (qHoldTime >= HOLD_TO_CLEAR)
            {
                bodies.clear();
                selected = NO_SELECTION;
            }
        }
        else
        {
            qHoldTime = 0.0f;
        }

        UpdateFreeCamera(cam, dt, drag.active);

        if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && !cam.looking && !IsMouseOverPanel())
        {
            Vector3 grabPoint;
            selected = PickBody(cam.camera, GetMousePosition(), bodies, grabPoint);
            if (selected != NO_SELECTION)
            {
                drag = StartDrag(bodies[selected], grabPoint);
            }
        }
        if (IsKeyPressed(KEY_DELETE) && selected != NO_SELECTION)
        {
            bodies.erase(bodies.begin() + selected);
            selected = NO_SELECTION;
        }

        if (selected == NO_SELECTION || !IsMouseButtonDown(MOUSE_BUTTON_LEFT))
        {
            drag.active = false;
        }
        if (drag.active)
        {
            UpdateDrag(drag, bodies[selected], cam.camera, PointerPosition(cam), dt);
            HoldDragged(drag, bodies[selected], paused);
        }

        if (!paused)
        {
            UpdatePhysics(bodies, dt);
        }
        if (drag.active)
        {
            HoldDragged(drag, bodies[selected], paused);
        }
        UpdateCursor(cam, bodies, drag, cursor);
        DrawScene(cam.camera, bodies, selected, paused, settings, activeSlider);
    }

    CloseWindow();
    return 0;
}
