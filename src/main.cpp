#include "raylib.h"
#include "raymath.h"
#include "rlgl.h"
#include <algorithm>
#include <cmath>
#include <utility>
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

struct MaterialInfo
{
    const char *name;
    float density;
    float bounce;
    float friction;
    Color color;
};

const MaterialInfo MATERIALS[] = {
    {"Foam", 30.0f, 0.3f, 0.7f, {232, 228, 214, 255}},
    {"Wood", 600.0f, 0.45f, 0.45f, {176, 124, 78, 255}},
    {"Ice", 917.0f, 0.15f, 0.03f, {178, 218, 236, 255}},
    {"Plastic", 950.0f, 0.6f, 0.35f, {106, 155, 216, 255}},
    {"Rubber", 1100.0f, 0.85f, 0.9f, {224, 104, 92, 255}},
    {"Clay", 1800.0f, 0.0f, 0.8f, {190, 112, 82, 255}},
    {"Stone", 2600.0f, 0.25f, 0.6f, {142, 142, 136, 255}},
    {"Steel", 7850.0f, 0.4f, 0.4f, {172, 178, 188, 255}},
    {"Gold", 19300.0f, 0.2f, 0.4f, {227, 184, 91, 255}},
};
const int MATERIAL_COUNT = sizeof(MATERIALS) / sizeof(MATERIALS[0]);

struct Liquid
{
    const char *name;
    float density;
    float drag;
    Color color;
};

const Liquid LIQUIDS[] = {
    {"None", 0.0f, 0.0f, BLANK},
    {"Water", 1000.0f, 1.5f, {64, 132, 214, 110}},
    {"Oil", 900.0f, 5.0f, {206, 168, 64, 140}},
    {"Honey", 1420.0f, 40.0f, {222, 140, 30, 190}},
    {"Mercury", 13534.0f, 2.0f, {182, 188, 198, 215}},
};
const int LIQUID_COUNT = sizeof(LIQUIDS) / sizeof(LIQUIDS[0]);
const int NO_LIQUID = 0;

struct Preset
{
    const char *name;
    float value;
};

const Preset GRAVITIES[] = {
    {"Earth", 9.81f}, {"Moon", 1.62f}, {"Mars", 3.71f}, {"Jupiter", 24.79f}, {"Zero", 0.0f},
};
const int GRAVITY_COUNT = sizeof(GRAVITIES) / sizeof(GRAVITIES[0]);

const Preset TIME_SCALES[] = {
    {"0.1×", 0.1f}, {"0.25×", 0.25f}, {"0.5×", 0.5f}, {"1×", 1.0f}, {"2×", 2.0f},
};
const int TIME_SCALE_COUNT = sizeof(TIME_SCALES) / sizeof(TIME_SCALES[0]);
const int NORMAL_TIME = 3;

struct Plane
{
    Vector3 normal = {0.0f, 0.0f, 0.0f};
    float distance;
};

struct Body
{
    Shape shape;
    int material;
    float density;
    Vector3 position;
    Vector3 velocity;
    Quaternion orientation;
    Vector3 angularVelocity;
    Vector3 size;
    Color color;
    Vector3 start;
    Quaternion startOrientation;
    Vector3 pushDirection;
    float age;
    float maxHeight;
    float rest;
    std::vector<Vector3> hull;
    std::vector<Plane> planes;
    std::vector<std::vector<int>> faces;
    std::vector<Vector3> trail;
};

struct StaticHull
{
    std::vector<Vector3> vertices;
    std::vector<Plane> planes;
    std::vector<std::vector<int>> faces;
};

enum class JointType
{
    Rope,
    Spring,
    Rod,
    Pulley
};

const char *JOINT_NAMES[] = {"Rope", "Spring", "Rod", "Pulley"};
const int JOINT_TYPE_COUNT = sizeof(JOINT_NAMES) / sizeof(JOINT_NAMES[0]);

struct Joint
{
    JointType type;
    int a;
    int b;
    Vector3 anchor;
    Vector3 pulley;
    float length;
    float stiffness;
};

struct Ripple
{
    Vector2 center;
    float age;
    float strength;
    float size;
};

struct Droplet
{
    Vector3 position;
    Vector3 velocity;
    float life;
};

struct World
{
    int gravity;
    int timeScale;
    int liquid;
    float level;
    bool customFriction;
    float friction;
    float windSpeed;
    float windAngle;
    bool airDrag;
    bool rain;
    bool ramp;
    float rampAngle;
    float launchSpeed;
    float pushForce;
    float dropHeight;
    bool trails;
    bool vectors;
    int jointType;
    float jointLength;
    float stiffness;
    float viewPitch;
    std::vector<Joint> joints;
    std::vector<Ripple> ripples;
    std::vector<Droplet> droplets;
    std::vector<Droplet> raindrops;
};

struct Ramp
{
    float bottomX;
    float topX;
    float height;
    Vector3 normal = {0.0f, 0.0f, 0.0f};
};

struct Wall
{
    Vector3 position;
    Vector3 size;
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
    int material;
    float massSlider;
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
    int a;
    int b;
    Vector3 point;
    Vector3 normal = {0.0f, 0.0f, 0.0f};
    float depth;
    float friction;
    float target;
    float push;
    float normalImpulse;
    float pushImpulse;
    Vector3 frictionImpulse;
};

struct Stopwatch
{
    bool running;
    float time;
};

struct Sample
{
    float time;
    float height;
    float speed;
    float kinetic;
    float potential;
};

const int STATIC_BODY = -1;

const float REST_SPEED = 0.3f;
const float PHYSICS_STEP = 1.0f / 240.0f;
const float MAX_FRAME_TIME = 0.05f;
const int SOLVER_ITERATIONS = 10;
const float CONTACT_SLOP = 0.005f;
const float CONTACT_TOLERANCE = 0.01f;
const float CORRECTION = 0.3f;
const float MAX_CORRECTION_SPEED = 2.0f;
const float ROLLING_RESISTANCE = 0.1f;
const float ANGULAR_DAMPING = 0.05f;
const float SLEEP_SPEED = 0.02f;
const float SLEEP_TIME = 0.3f;
const int ROUND_SIDES = 12;
const int ROUND_RINGS = 6;

const float TANK_HALF = 4.0f;
const float TANK_HEIGHT = 3.2f;
const float TANK_WALL = 0.2f;
const float MIN_LEVEL = 0.3f;
const float MAX_LEVEL = 3.0f;
const float SPLASH_SPEED = 1.5f;
const float RIPPLE_LIFE = 1.6f;
const float SURFACE_DAMPING = 0.3f;

const float AIR_DENSITY = 1.225f;
const float MAX_WIND = 30.0f;
const float RAIN_RATE = 1500.0f;
const float RAIN_RADIUS = 25.0f;
const float RAIN_SPEED = 12.0f;
const int MAX_RAINDROPS = 4000;
const float RAIN_RIPPLE_CHANCE = 0.04f;

const float RAMP_TOP_X = -6.5f;
const float RAMP_LENGTH = 7.0f;
const float RAMP_WIDTH = 3.0f;

const float SPRING_DAMPING = 0.05f;
const float JOINT_CORRECTION = 0.5f;
const float PULLEY_RAISE = 1.5f;

const float TRAIL_STEP = 0.05f;
const int MAX_TRAIL = 3000;
const float VECTOR_SCALE = 0.25f;
const float FORCE_SCALE = 0.02f;

const float SAMPLE_INTERVAL = 1.0f / 30.0f;
const float HISTORY_SECONDS = 10.0f;

const float SPAWN_DISTANCE = 12.0f;
const float MAX_SPAWN_DISTANCE = 30.0f;
const float MIN_MASS_SLIDER = 0.02f;

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

const float FLOOR_EXTENT = 1500.0f;
const float FOG_DISTANCE = 180.0f;
const float SHADOW_FADE_HEIGHT = 10.0f;

const float MARGIN = 16.0f;
const float ICON_BUTTON_SIZE = 36.0f;

const float CARD_MAX_WIDTH = 1040.0f;
const float CARD_PADDING = 14.0f;
const float SEGMENT_HEIGHT = 32.0f;
const float SEGMENT_GAP = 8.0f;
const float ROW_GAP = 14.0f;
const float SPAWN_BUTTON_WIDTH = 120.0f;
const float SPAWN_BUTTON_HEIGHT = 40.0f;
const int SLIDER_SLOTS = 5;
const int MASS_SLOT = 3;
const float CARD_HEIGHT =
    2.0f * CARD_PADDING + 2.0f * SEGMENT_HEIGHT + SEGMENT_GAP + ROW_GAP + SPAWN_BUTTON_HEIGHT;
const int MASS_SLIDER = 50;
const int WORLD_SLIDER_BASE = 100;

const float WORLD_WIDTH = 316.0f;
const float WORLD_PADDING = 14.0f;
const float WORLD_SEGMENT_HEIGHT = 28.0f;
const float WORLD_SECTION = 18.0f + WORLD_SEGMENT_HEIGHT + 12.0f;
const float WORLD_SLIDER = 46.0f;
const float WORLD_HINT = 18.0f;
const float WORLD_TOGGLE = 30.0f;

const float SETTINGS_WIDTH = 320.0f;
const float SETTINGS_PADDING = 16.0f;
const float TOGGLE_ROW = 30.0f;
const float BINDING_ROW = 22.0f;

const int SELECTED_ROWS = 13;
const float SELECTED_ROW = 22.0f;
const float SELECTED_WIDTH = 270.0f;
const float SELECTED_HEIGHT = 14.0f + 18.0f + 14.0f + SELECTED_ROWS * SELECTED_ROW + 6.0f;
const float GRAPH_HEIGHT = 156.0f;

const Color COLOR_SKY = {30, 32, 36, 255};
const Color COLOR_FLOOR = {38, 40, 44, 255};
const Color COLOR_GRID_MINOR = {45, 47, 52, 255};
const Color COLOR_GRID_MAJOR = {55, 58, 64, 255};
const Color COLOR_PANEL = {22, 23, 27, 235};
const Color COLOR_BORDER = {255, 255, 255, 20};
const Color COLOR_SURFACE = {255, 255, 255, 10};
const Color COLOR_HOVER = {255, 255, 255, 18};
const Color COLOR_TEXT = {236, 236, 238, 255};
const Color COLOR_MUTED = {148, 151, 160, 255};
const Color COLOR_ACCENT = {217, 119, 87, 255};
const Color COLOR_ACCENT_HOVER = {230, 138, 108, 255};

#ifndef ASSETS_DIR
#define ASSETS_DIR "assets"
#endif

struct UiFont
{
    Font font;
    float size;
};

struct Fonts
{
    UiFont text;
    UiFont bold;
    UiFont small;
    UiFont title;
};

Fonts fonts;

struct UiState
{
    int worldTab;
    int graphTab;
    int linkFrom;
    bool settingsOpen;
    bool showGrid;
    bool showShadows;
    bool showFps;
    int activeSlider;
};

struct Toggle
{
    const char *name;
    bool UiState::*field;
};

const Toggle TOGGLES[] = {
    {"Floor grid", &UiState::showGrid},
    {"Shadows", &UiState::showShadows},
    {"Show FPS", &UiState::showFps},
};
const int TOGGLE_COUNT = sizeof(TOGGLES) / sizeof(TOGGLES[0]);

struct Binding
{
    const char *action;
    const char *keys[4];
};

const Binding BINDINGS[] = {
    {"Settings", {"Esc"}},
    {"Pause / resume", {"Space"}},
    {"Reset positions", {"R"}},
    {"Spawn where you look", {"N"}},
    {"Throw from camera", {"F"}},
    {"Give selected velocity", {"V"}},
    {"Push selected (on / off)", {"P"}},
    {"Hang selected", {"H"}},
    {"Link two bodies", {"J"}},
    {"Cut joints of selected", {"K"}},
    {"Stopwatch (Shift: reset)", {"T"}},
    {"Delete last (hold: all)", {"Q"}},
    {"Delete selected", {"Del"}},
    {"Select, drag, throw", {"LMB"}},
    {"Lift while dragging", {"Wheel"}},
    {"Look around", {"RMB"}},
    {"Fly", {"W", "A", "S", "D"}},
    {"Up / down", {"E", "C"}},
    {"Move faster", {"Shift"}},
    {"Move forward", {"Wheel"}},
};
const int BINDING_COUNT = sizeof(BINDINGS) / sizeof(BINDINGS[0]);

const float SETTINGS_TOGGLES_Y = SETTINGS_PADDING + 18.0f + 16.0f + 22.0f;
const float SETTINGS_BINDINGS_Y = SETTINGS_TOGGLES_Y + TOGGLE_COUNT * TOGGLE_ROW + 14.0f + 22.0f;
const float SETTINGS_HEIGHT = SETTINGS_BINDINGS_Y + BINDING_COUNT * BINDING_ROW + 12.0f;

struct Floor
{
    Shader shader;
    int cameraLoc;
    int gridLoc;
};

const char *FLOOR_VS = R"(#version 330
in vec3 vertexPosition;
uniform mat4 mvp;
out vec3 worldPos;
void main()
{
    worldPos = vertexPosition;
    gl_Position = mvp * vec4(vertexPosition, 1.0);
}
)";

const char *FLOOR_FS = R"(#version 330
in vec3 worldPos;
uniform vec3 cameraPos;
uniform float showGrid;
uniform vec3 floorColor;
uniform vec3 minorColor;
uniform vec3 majorColor;
uniform vec3 fogColor;
uniform float fogDistance;
out vec4 finalColor;

float GridLine(vec2 p, float spacing)
{
    vec2 c = p / spacing;
    vec2 g = abs(fract(c - 0.5) - 0.5) / fwidth(c);
    return 1.0 - min(min(g.x, g.y), 1.0);
}

void main()
{
    vec2 p = worldPos.xz;
    vec2 d = fwidth(p);
    float density = max(d.x, d.y);

    float minor = GridLine(p, 1.0) * (1.0 - smoothstep(0.04, 0.15, density));
    float major = GridLine(p, 10.0) * (1.0 - smoothstep(0.4, 1.5, density));

    vec3 color = floorColor;
    color = mix(color, minorColor, minor * showGrid);
    color = mix(color, majorColor, major * showGrid);

    float fog = smoothstep(fogDistance * 0.1, fogDistance, length(worldPos.xz - cameraPos.xz));
    finalColor = vec4(mix(color, fogColor, fog), 1.0);
}
)";

struct Lighting
{
    Shader shader;
    int cameraLoc;
};

const char *LIT_VS = R"(#version 330
in vec3 vertexPosition;
in vec4 vertexColor;
uniform mat4 mvp;
out vec3 worldPos;
out vec4 color;
void main()
{
    worldPos = vertexPosition;
    color = vertexColor;
    gl_Position = mvp * vec4(vertexPosition, 1.0);
}
)";

const char *LIT_FS = R"(#version 330
in vec3 worldPos;
in vec4 color;
uniform vec3 cameraPos;
out vec4 finalColor;

void main()
{
    vec3 normal = normalize(cross(dFdx(worldPos), dFdy(worldPos)));
    if (dot(normal, cameraPos - worldPos) < 0.0) normal = -normal;

    vec3 light = normalize(vec3(0.45, 1.0, 0.3));
    float diffuse = max(dot(normal, light), 0.0);
    float ambient = mix(0.38, 0.55, normal.y * 0.5 + 0.5);
    finalColor = vec4(color.rgb * (ambient + 0.6 * diffuse), color.a);
}
)";

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

    switch (shape)
    {
    case Shape::Sphere: return {radius};
    case Shape::Cube: return {{"Size", &SpawnSettings::width, 0.3f, 4.0f, " m"}};
    case Shape::Box:
    case Shape::Ellipsoid: return {width, height, depth};
    case Shape::Pyramid4: return {{"Base", &SpawnSettings::width, 0.3f, 4.0f, " m"}, height};
    case Shape::Pyramid3:
    case Shape::Cylinder:
    case Shape::Cone:
    case Shape::Capsule: return {radius, height};
    default: return {};
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

struct Ring
{
    float rx;
    float rz;
    float y;
};

using Faces = std::vector<std::vector<int>>;

void AddFace(std::vector<Plane> &planes, Faces &faces, const std::vector<Vector3> &v, std::vector<int> polygon)
{
    Vector3 normal = Vector3CrossProduct(Vector3Subtract(v[polygon[1]], v[polygon[0]]),
                                         Vector3Subtract(v[polygon[2]], v[polygon[0]]));
    float length = Vector3Length(normal);
    if (length < 0.000001f)
    {
        return;
    }
    normal = Vector3Scale(normal, 1.0f / length);
    float distance = Vector3DotProduct(normal, v[polygon[0]]);
    if (distance < 0.0f)
    {
        normal = Vector3Negate(normal);
        distance = -distance;
    }
    planes.push_back({normal, distance});
    faces.push_back(std::move(polygon));
}

void BuildRevolution(const std::vector<Ring> &rings, int sides, float angleOffset, const float *bottomApex,
                     const float *topApex, std::vector<Vector3> &vertices, std::vector<Plane> &planes, Faces &faces)
{
    for (const Ring &ring : rings)
    {
        for (int s = 0; s < sides; s++)
        {
            float angle = angleOffset + 2.0f * PI * s / sides;
            vertices.push_back({sinf(angle) * ring.rx, ring.y, cosf(angle) * ring.rz});
        }
    }
    auto index = [sides](int ring, int side) { return ring * sides + (side % sides); };
    auto cap = [&](int ring) {
        std::vector<int> polygon;
        for (int s = 0; s < sides; s++)
        {
            polygon.push_back(index(ring, s));
        }
        return polygon;
    };

    for (int r = 0; r + 1 < (int)rings.size(); r++)
    {
        for (int s = 0; s < sides; s++)
        {
            AddFace(planes, faces, vertices, {index(r, s), index(r, s + 1), index(r + 1, s + 1), index(r + 1, s)});
        }
    }

    int last = (int)rings.size() - 1;
    if (bottomApex != nullptr)
    {
        int apex = (int)vertices.size();
        vertices.push_back({0.0f, *bottomApex, 0.0f});
        for (int s = 0; s < sides; s++)
        {
            AddFace(planes, faces, vertices, {apex, index(0, s), index(0, s + 1)});
        }
    }
    else
    {
        planes.push_back({{0.0f, -1.0f, 0.0f}, -rings[0].y});
        faces.push_back(cap(0));
    }
    if (topApex != nullptr)
    {
        int apex = (int)vertices.size();
        vertices.push_back({0.0f, *topApex, 0.0f});
        for (int s = 0; s < sides; s++)
        {
            AddFace(planes, faces, vertices, {apex, index(last, s), index(last, s + 1)});
        }
    }
    else
    {
        planes.push_back({{0.0f, 1.0f, 0.0f}, rings[last].y});
        faces.push_back(cap(last));
    }
}

void BuildHull(Body &b)
{
    b.hull.clear();
    b.planes.clear();
    b.faces.clear();
    Vector3 h = Vector3Scale(b.size, 0.5f);
    const float diagonal = sqrtf(2.0f);

    switch (b.shape)
    {
    case Shape::Sphere:
        return;
    case Shape::Cube:
    case Shape::Box:
        BuildRevolution({{h.x * diagonal, h.z * diagonal, -h.y}, {h.x * diagonal, h.z * diagonal, h.y}}, 4, PI / 4.0f,
                        nullptr, nullptr, b.hull, b.planes, b.faces);
        return;
    case Shape::Pyramid3:
        BuildRevolution({{h.x, h.z, -h.y}}, 3, 0.0f, nullptr, &h.y, b.hull, b.planes, b.faces);
        return;
    case Shape::Pyramid4:
        BuildRevolution({{h.x * diagonal, h.z * diagonal, -h.y}}, 4, PI / 4.0f, nullptr, &h.y, b.hull, b.planes, b.faces);
        return;
    case Shape::Cylinder:
        BuildRevolution({{h.x, h.z, -h.y}, {h.x, h.z, h.y}}, 16, 0.0f, nullptr, nullptr, b.hull, b.planes, b.faces);
        return;
    case Shape::Cone:
        BuildRevolution({{h.x, h.z, -h.y}}, 16, 0.0f, nullptr, &h.y, b.hull, b.planes, b.faces);
        return;
    case Shape::Ellipsoid:
    {
        std::vector<Ring> rings;
        for (int i = 1; i <= ROUND_RINGS; i++)
        {
            float phi = -PI / 2.0f + PI * i / (ROUND_RINGS + 1);
            rings.push_back({h.x * cosf(phi), h.z * cosf(phi), h.y * sinf(phi)});
        }
        float bottom = -h.y;
        BuildRevolution(rings, ROUND_SIDES, 0.0f, &bottom, &h.y, b.hull, b.planes, b.faces);
        return;
    }
    case Shape::Capsule:
    {
        float r = h.x;
        float offset = h.y - r;
        std::vector<Ring> rings;
        int half = ROUND_RINGS / 2;
        for (int i = 1; i <= half; i++)
        {
            float phi = -PI / 2.0f + PI / 2.0f * i / half;
            rings.push_back({r * cosf(phi), r * cosf(phi), r * sinf(phi) - offset});
        }
        for (int i = 0; i < half; i++)
        {
            float phi = PI / 2.0f * i / half;
            rings.push_back({r * cosf(phi), r * cosf(phi), r * sinf(phi) + offset});
        }
        float bottom = -h.y;
        BuildRevolution(rings, ROUND_SIDES, 0.0f, &bottom, &h.y, b.hull, b.planes, b.faces);
        return;
    }
    default:
        return;
    }
}

const MaterialInfo &MaterialOf(const Body &b)
{
    return MATERIALS[b.material];
}

Body MakeBody(Shape shape, int material, Vector3 position, Vector3 size, Color color)
{
    Body b = {};
    b.shape = shape;
    b.material = material;
    b.density = MATERIALS[material].density;
    b.position = position;
    b.velocity = {0.0f, 0.0f, 0.0f};
    b.orientation = QuaternionIdentity();
    b.angularVelocity = {0.0f, 0.0f, 0.0f};
    b.size = size;
    b.color = color;
    b.start = position;
    b.startOrientation = b.orientation;
    b.maxHeight = position.y - size.y / 2.0f;
    BuildHull(b);
    return b;
}

int FindMaterial(const char *name)
{
    for (int i = 0; i < MATERIAL_COUNT; i++)
    {
        if (TextIsEqual(MATERIALS[i].name, name))
        {
            return i;
        }
    }
    return 0;
}

Body MakeDemoBody(Shape shape, const char *material, Vector3 position, Vector3 size)
{
    int m = FindMaterial(material);
    return MakeBody(shape, m, position, size, MATERIALS[m].color);
}

std::vector<Body> CreateBodies()
{
    return {
        MakeDemoBody(Shape::Cube, "Wood", {-1.8f, 5.0f, 0.5f}, {0.9f, 0.9f, 0.9f}),
        MakeDemoBody(Shape::Sphere, "Steel", {0.0f, 6.0f, -0.5f}, {0.8f, 0.8f, 0.8f}),
        MakeDemoBody(Shape::Box, "Ice", {1.8f, 7.0f, 0.5f}, {1.2f, 0.5f, 0.8f}),
    };
}

float RandomFloat(float min, float max)
{
    return min + (max - min) * GetRandomValue(0, 1000) / 1000.0f;
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
    case Shape::Pyramid3: return 3.0f * sqrtf(3.0f) / 4.0f * r * r * s.y / 3.0f;
    case Shape::Pyramid4: return s.x * s.y * s.z / 3.0f;
    case Shape::Cylinder: return PI * r * r * s.y;
    case Shape::Cone: return PI * r * r * s.y / 3.0f;
    case Shape::Capsule: return PI * r * r * (s.y - 2.0f * r) + 4.0f / 3.0f * PI * r * r * r;
    default: return s.x * s.y * s.z;
    }
}

float Density(const Body &b)
{
    return b.density;
}

float Mass(const Body &b)
{
    return Volume(b) * Density(b);
}

float InverseMass(const Body &b)
{
    return 1.0f / Mass(b);
}

bool IsRound(const Body &b)
{
    return b.shape == Shape::Sphere;
}

bool Rolls(const Body &b)
{
    return b.shape == Shape::Sphere || b.shape == Shape::Ellipsoid || b.shape == Shape::Capsule ||
           b.shape == Shape::Cylinder || b.shape == Shape::Cone;
}

float SpawnMass(const SpawnSettings &settings)
{
    if (settings.massSlider < MIN_MASS_SLIDER)
    {
        return 0.0f;
    }
    return powf(10.0f, -1.0f + 5.0f * settings.massSlider);
}

Vector3 SpawnPoint(const FreeCamera &cam, float height)
{
    Vector3 forward = CameraForward(cam);
    Vector3 origin = cam.camera.position;
    float distance = SPAWN_DISTANCE;
    if (forward.y < -0.05f)
    {
        distance = std::min(-origin.y / forward.y, MAX_SPAWN_DISTANCE);
    }
    Vector3 ground = Vector3Add(origin, Vector3Scale(forward, distance));
    return {ground.x + RandomFloat(-0.8f, 0.8f), height, ground.z + RandomFloat(-0.8f, 0.8f)};
}

Body CreateBody(const SpawnSettings &settings, Vector3 position)
{
    Vector3 size = SpawnSize(settings);
    position.y += size.y / 2.0f;
    Color color = ColorBrightness(MATERIALS[settings.material].color, RandomFloat(-0.12f, 0.12f));
    Body b = MakeBody(settings.shape, settings.material, position, size, color);
    float mass = SpawnMass(settings);
    if (mass > 0.0f)
    {
        b.density = mass / Volume(b);
    }
    return b;
}

Vector3 HorizontalForward(const FreeCamera &cam)
{
    return Vector3Normalize({sinf(cam.yaw), 0.0f, cosf(cam.yaw)});
}

Body ThrowBody(const SpawnSettings &settings, const FreeCamera &cam, float speed)
{
    Vector3 forward = CameraForward(cam);
    Vector3 size = SpawnSize(settings);
    float distance = 1.5f + std::max({size.x, size.y, size.z});
    Vector3 position = Vector3Add(cam.camera.position, Vector3Scale(forward, distance));
    position.y = std::max(position.y, size.y / 2.0f) - size.y / 2.0f;
    Body b = CreateBody(settings, position);
    b.velocity = Vector3Scale(forward, speed);
    return b;
}

void ResetStats(Body &b);

void ResetBodies(std::vector<Body> &bodies)
{
    for (Body &b : bodies)
    {
        b.position = b.start;
        b.orientation = b.startOrientation;
        b.velocity = {0.0f, 0.0f, 0.0f};
        b.angularVelocity = {0.0f, 0.0f, 0.0f};
        b.pushDirection = {0.0f, 0.0f, 0.0f};
        b.trail.clear();
        ResetStats(b);
    }
}

Vector3 ToWorld(const Body &b, Vector3 local)
{
    return Vector3Add(b.position, Vector3RotateByQuaternion(local, b.orientation));
}

Vector3 ToLocal(const Body &b, Vector3 world)
{
    return Vector3RotateByQuaternion(Vector3Subtract(world, b.position), QuaternionInvert(b.orientation));
}

Vector3 RotateToWorld(const Body &b, Vector3 local)
{
    return Vector3RotateByQuaternion(local, b.orientation);
}

Vector3 RotateToLocal(const Body &b, Vector3 world)
{
    return Vector3RotateByQuaternion(world, QuaternionInvert(b.orientation));
}

void Extent(const Body &b, float &low, float &high)
{
    if (b.hull.empty())
    {
        low = b.position.y - b.size.x / 2.0f;
        high = b.position.y + b.size.x / 2.0f;
        return;
    }
    low = 1e9f;
    high = -1e9f;
    for (const Vector3 &v : b.hull)
    {
        float y = ToWorld(b, v).y;
        low = std::min(low, y);
        high = std::max(high, y);
    }
}

float BottomHeight(const Body &b)
{
    float low;
    float high;
    Extent(b, low, high);
    return low;
}

void ResetStats(Body &b)
{
    b.age = 0.0f;
    b.maxHeight = BottomHeight(b);
}

float BoundingRadius(const Body &b)
{
    return Vector3Length(b.size) / 2.0f;
}

Vector3 LocalInertia(const Body &b)
{
    float m = Mass(b);
    Vector3 s = b.size;
    float r = s.x / 2.0f;
    switch (b.shape)
    {
    case Shape::Sphere:
    {
        float i = 0.4f * m * r * r;
        return {i, i, i};
    }
    case Shape::Ellipsoid:
    {
        Vector3 h = Vector3Scale(s, 0.5f);
        return {m * (h.y * h.y + h.z * h.z) / 5.0f, m * (h.x * h.x + h.z * h.z) / 5.0f,
                m * (h.x * h.x + h.y * h.y) / 5.0f};
    }
    case Shape::Cylinder:
    case Shape::Capsule:
    {
        float side = m * (3.0f * r * r + s.y * s.y) / 12.0f;
        return {side, 0.5f * m * r * r, side};
    }
    case Shape::Cone:
    {
        float side = m * (3.0f * r * r / 20.0f + 3.0f * s.y * s.y / 80.0f);
        return {side, 0.3f * m * r * r, side};
    }
    default:
        return {m * (s.y * s.y + s.z * s.z) / 12.0f, m * (s.x * s.x + s.z * s.z) / 12.0f,
                m * (s.x * s.x + s.y * s.y) / 12.0f};
    }
}

Vector3 ApplyInverseInertia(const Body &b, Vector3 v)
{
    Vector3 inertia = LocalInertia(b);
    Vector3 local = RotateToLocal(b, v);
    local = {local.x / inertia.x, local.y / inertia.y, local.z / inertia.z};
    return RotateToWorld(b, local);
}

Vector3 PointVelocity(const Body &b, Vector3 offset)
{
    return Vector3Add(b.velocity, Vector3CrossProduct(b.angularVelocity, offset));
}

void ApplyImpulse(Body &b, Vector3 impulse, Vector3 offset)
{
    b.velocity = Vector3Add(b.velocity, Vector3Scale(impulse, InverseMass(b)));
    b.angularVelocity =
        Vector3Add(b.angularVelocity, ApplyInverseInertia(b, Vector3CrossProduct(offset, impulse)));
}

float AngularMass(const Body &b, Vector3 offset, Vector3 direction)
{
    Vector3 turn = ApplyInverseInertia(b, Vector3CrossProduct(offset, direction));
    return Vector3DotProduct(direction, Vector3CrossProduct(turn, offset));
}

bool HasLiquid(const World &world)
{
    return world.liquid != NO_LIQUID;
}

bool InsideTank(Vector3 p)
{
    return fabsf(p.x) < TANK_HALF && fabsf(p.z) < TANK_HALF;
}

StaticHull BoxHull(Vector3 center, Vector3 size)
{
    Vector3 h = Vector3Scale(size, 0.5f);
    StaticHull hull;
    for (int i = 0; i < 8; i++)
    {
        hull.vertices.push_back({center.x + (i & 1 ? h.x : -h.x), center.y + (i & 2 ? h.y : -h.y),
                                 center.z + (i & 4 ? h.z : -h.z)});
    }
    hull.planes = {
        {{1.0f, 0.0f, 0.0f}, center.x + h.x},  {{-1.0f, 0.0f, 0.0f}, -(center.x - h.x)},
        {{0.0f, 1.0f, 0.0f}, center.y + h.y},  {{0.0f, -1.0f, 0.0f}, -(center.y - h.y)},
        {{0.0f, 0.0f, 1.0f}, center.z + h.z},  {{0.0f, 0.0f, -1.0f}, -(center.z - h.z)},
    };
    hull.faces = {{1, 3, 7, 5}, {0, 2, 6, 4}, {2, 3, 7, 6}, {0, 1, 5, 4}, {4, 5, 7, 6}, {0, 1, 3, 2}};
    return hull;
}

std::vector<Wall> TankWalls()
{
    float offset = TANK_HALF + TANK_WALL / 2.0f;
    float length = 2.0f * (TANK_HALF + TANK_WALL);
    float y = TANK_HEIGHT / 2.0f;
    return {
        {{offset, y, 0.0f}, {TANK_WALL, TANK_HEIGHT, length}},
        {{-offset, y, 0.0f}, {TANK_WALL, TANK_HEIGHT, length}},
        {{0.0f, y, offset}, {length, TANK_HEIGHT, TANK_WALL}},
        {{0.0f, y, -offset}, {length, TANK_HEIGHT, TANK_WALL}},
    };
}

Ramp GetRamp(const World &world)
{
    float angle = world.rampAngle * DEG2RAD;
    Ramp ramp = {};
    ramp.topX = RAMP_TOP_X;
    ramp.bottomX = RAMP_TOP_X - RAMP_LENGTH * cosf(angle);
    ramp.height = RAMP_LENGTH * sinf(angle);
    ramp.normal = {-sinf(angle), cosf(angle), 0.0f};
    return ramp;
}

StaticHull RampHull(const Ramp &ramp)
{
    float w = RAMP_WIDTH / 2.0f;
    StaticHull hull;
    hull.vertices = {
        {ramp.bottomX, 0.0f, -w}, {ramp.bottomX, 0.0f, w},   {ramp.topX, 0.0f, -w},
        {ramp.topX, 0.0f, w},     {ramp.topX, ramp.height, -w}, {ramp.topX, ramp.height, w},
    };
    hull.planes = {
        {ramp.normal, Vector3DotProduct(ramp.normal, {ramp.bottomX, 0.0f, 0.0f})},
        {{1.0f, 0.0f, 0.0f}, ramp.topX},
        {{0.0f, -1.0f, 0.0f}, 0.0f},
        {{0.0f, 0.0f, 1.0f}, w},
        {{0.0f, 0.0f, -1.0f}, w},
    };
    hull.faces = {{0, 1, 5, 4}, {2, 3, 5, 4}, {0, 1, 3, 2}, {1, 3, 5}, {0, 2, 4}};
    return hull;
}

std::vector<StaticHull> StaticHulls(const World &world)
{
    std::vector<StaticHull> hulls;
    if (HasLiquid(world))
    {
        for (const Wall &wall : TankWalls())
        {
            hulls.push_back(BoxHull(wall.position, wall.size));
        }
    }
    if (world.ramp)
    {
        hulls.push_back(RampHull(GetRamp(world)));
    }
    return hulls;
}

bool InsideHull(const std::vector<Plane> &planes, Vector3 p, float &depth, Vector3 &normal)
{
    float best = -1e9f;
    for (const Plane &plane : planes)
    {
        float d = Vector3DotProduct(plane.normal, p) - plane.distance;
        if (d > 0.0f)
        {
            return false;
        }
        if (d > best)
        {
            best = d;
            normal = plane.normal;
        }
    }
    depth = -best;
    return true;
}

float HullDistance(const std::vector<Plane> &planes, Vector3 p, Vector3 &normal)
{
    float best = -1e9f;
    for (const Plane &plane : planes)
    {
        float d = Vector3DotProduct(plane.normal, p) - plane.distance;
        if (d > best)
        {
            best = d;
            normal = plane.normal;
        }
    }
    return best;
}

float PairFriction(const World &world, float a, float b)
{
    return world.customFriction ? world.friction : sqrtf(a * b);
}

float ContactFriction(const World &world, const Body &b)
{
    return world.customFriction ? world.friction : MaterialOf(b).friction;
}

void AddContact(std::vector<Contact> &contacts, int a, int b, Vector3 point, Vector3 normal, float depth,
                float friction)
{
    Contact c = {};
    c.a = a;
    c.b = b;
    c.point = point;
    c.normal = normal;
    c.depth = depth;
    c.friction = friction;
    contacts.push_back(c);
}

void FloorContacts(const std::vector<Body> &bodies, int i, const World &world, std::vector<Contact> &contacts)
{
    const Body &b = bodies[i];
    float friction = ContactFriction(world, b);
    Vector3 down = {0.0f, -1.0f, 0.0f};
    if (b.hull.empty())
    {
        float r = b.size.x / 2.0f;
        if (b.position.y < r)
        {
            AddContact(contacts, i, STATIC_BODY, {b.position.x, 0.0f, b.position.z}, down, r - b.position.y, friction);
        }
        return;
    }
    for (const Vector3 &v : b.hull)
    {
        Vector3 w = ToWorld(b, v);
        if (w.y < 0.0f)
        {
            AddContact(contacts, i, STATIC_BODY, w, down, -w.y, friction);
        }
    }
}

struct WorldHull
{
    std::vector<Vector3> vertices;
    std::vector<Plane> planes;
    Faces faces;
};

WorldHull MakeWorldHull(const Body &b)
{
    WorldHull hull;
    for (const Vector3 &v : b.hull)
    {
        hull.vertices.push_back(ToWorld(b, v));
    }
    for (const Plane &plane : b.planes)
    {
        Vector3 normal = RotateToWorld(b, plane.normal);
        hull.planes.push_back({normal, plane.distance + Vector3DotProduct(normal, b.position)});
    }
    hull.faces = b.faces;
    return hull;
}

WorldHull MakeWorldHull(const StaticHull &s)
{
    return {s.vertices, s.planes, s.faces};
}

void Project(const std::vector<Vector3> &vertices, Vector3 axis, float &low, float &high)
{
    low = 1e9f;
    high = -1e9f;
    for (const Vector3 &v : vertices)
    {
        float d = Vector3DotProduct(v, axis);
        low = std::min(low, d);
        high = std::max(high, d);
    }
}

bool SeparatingAxis(const WorldHull &a, const WorldHull &b, Vector3 &normal, float &depth)
{
    depth = 1e9f;
    auto test = [&](Vector3 axis) {
        float a0;
        float a1;
        float b0;
        float b1;
        Project(a.vertices, axis, a0, a1);
        Project(b.vertices, axis, b0, b1);
        float forward = a1 - b0;
        float backward = b1 - a0;
        if (forward <= 0.0f || backward <= 0.0f)
        {
            return false;
        }
        if (forward < depth)
        {
            depth = forward;
            normal = axis;
        }
        if (backward < depth)
        {
            depth = backward;
            normal = Vector3Negate(axis);
        }
        return true;
    };
    for (const Plane &plane : a.planes)
    {
        if (!test(plane.normal))
        {
            return false;
        }
    }
    for (const Plane &plane : b.planes)
    {
        if (!test(plane.normal))
        {
            return false;
        }
    }
    return true;
}

std::vector<Vector3> ClipPolygon(const std::vector<Vector3> &polygon, Vector3 normal, float distance)
{
    std::vector<Vector3> result;
    for (size_t k = 0; k < polygon.size(); k++)
    {
        Vector3 from = polygon[k];
        Vector3 to = polygon[(k + 1) % polygon.size()];
        float dFrom = Vector3DotProduct(normal, from) - distance;
        float dTo = Vector3DotProduct(normal, to) - distance;
        if (dFrom <= 0.0f)
        {
            result.push_back(from);
        }
        if ((dFrom <= 0.0f) != (dTo <= 0.0f))
        {
            result.push_back(Vector3Lerp(from, to, dFrom / (dFrom - dTo)));
        }
    }
    return result;
}

int AlignedFace(const WorldHull &hull, Vector3 direction)
{
    int best = 0;
    for (size_t k = 1; k < hull.planes.size(); k++)
    {
        if (Vector3DotProduct(hull.planes[k].normal, direction) > Vector3DotProduct(hull.planes[best].normal, direction))
        {
            best = (int)k;
        }
    }
    return best;
}

bool ClipContacts(const WorldHull &reference, const WorldHull &incident, Vector3 outward, int i, int j, Vector3 normal,
                  float friction, std::vector<Contact> &contacts)
{
    int refFace = AlignedFace(reference, outward);
    int incFace = AlignedFace(incident, Vector3Negate(outward));
    const Plane &refPlane = reference.planes[refFace];

    std::vector<Vector3> refPolygon;
    Vector3 centroid = {0.0f, 0.0f, 0.0f};
    for (int index : reference.faces[refFace])
    {
        refPolygon.push_back(reference.vertices[index]);
        centroid = Vector3Add(centroid, reference.vertices[index]);
    }
    centroid = Vector3Scale(centroid, 1.0f / refPolygon.size());

    std::vector<Vector3> clipped;
    for (int index : incident.faces[incFace])
    {
        clipped.push_back(incident.vertices[index]);
    }
    for (size_t k = 0; k < refPolygon.size() && !clipped.empty(); k++)
    {
        Vector3 from = refPolygon[k];
        Vector3 to = refPolygon[(k + 1) % refPolygon.size()];
        Vector3 side = Vector3CrossProduct(Vector3Subtract(to, from), refPlane.normal);
        if (Vector3Length(side) < 0.000001f)
        {
            continue;
        }
        side = Vector3Normalize(side);
        if (Vector3DotProduct(side, Vector3Subtract(centroid, from)) > 0.0f)
        {
            side = Vector3Negate(side);
        }
        clipped = ClipPolygon(clipped, side, Vector3DotProduct(side, from));
    }

    bool found = false;
    for (const Vector3 &p : clipped)
    {
        float separation = Vector3DotProduct(refPlane.normal, p) - refPlane.distance;
        if (separation <= CONTACT_TOLERANCE)
        {
            AddContact(contacts, i, j, p, normal, std::max(-separation, 0.0f), friction);
            found = true;
        }
    }
    return found;
}

void HullContacts(const WorldHull &a, const WorldHull &b, int i, int j, float friction,
                  std::vector<Contact> &contacts)
{
    Vector3 normal = {0.0f, 0.0f, 0.0f};
    float depth;
    if (!SeparatingAxis(a, b, normal, depth))
    {
        return;
    }

    float alignA = Vector3DotProduct(a.planes[AlignedFace(a, normal)].normal, normal);
    float alignB = -Vector3DotProduct(b.planes[AlignedFace(b, Vector3Negate(normal))].normal, normal);
    bool found = alignB >= alignA - 0.001f
                     ? ClipContacts(b, a, Vector3Negate(normal), i, j, normal, friction, contacts)
                     : ClipContacts(a, b, normal, i, j, normal, friction, contacts);
    if (found)
    {
        return;
    }

    Vector3 deepA = a.vertices[0];
    Vector3 deepB = b.vertices[0];
    for (const Vector3 &v : a.vertices)
    {
        if (Vector3DotProduct(v, normal) > Vector3DotProduct(deepA, normal))
        {
            deepA = v;
        }
    }
    for (const Vector3 &v : b.vertices)
    {
        if (Vector3DotProduct(v, normal) < Vector3DotProduct(deepB, normal))
        {
            deepB = v;
        }
    }
    AddContact(contacts, i, j, Vector3Lerp(deepA, deepB, 0.5f), normal, depth, friction);
}

void StaticContacts(const std::vector<Body> &bodies, const std::vector<WorldHull> &hulls, int i,
                    const WorldHull &hull, const World &world, std::vector<Contact> &contacts)
{
    const Body &b = bodies[i];
    float friction = ContactFriction(world, b);
    if (b.hull.empty())
    {
        Vector3 normal = {0.0f, 0.0f, 0.0f};
        float r = b.size.x / 2.0f;
        float d = HullDistance(hull.planes, b.position, normal);
        if (d < r)
        {
            Vector3 n = Vector3Negate(normal);
            AddContact(contacts, i, STATIC_BODY, Vector3Add(b.position, Vector3Scale(n, r)), n, r - d, friction);
        }
        return;
    }
    HullContacts(hulls[i], hull, i, STATIC_BODY, friction, contacts);
}

void SphereHullContact(const std::vector<Body> &bodies, int sphere, int hull, float friction,
                       std::vector<Contact> &contacts)
{
    const Body &s = bodies[sphere];
    const Body &h = bodies[hull];
    float r = s.size.x / 2.0f;
    Vector3 local = ToLocal(h, s.position);
    Vector3 normal = {0.0f, 0.0f, 0.0f};
    float d;
    Vector3 surface;

    if (h.shape == Shape::Box || h.shape == Shape::Cube)
    {
        Vector3 half = Vector3Scale(h.size, 0.5f);
        Vector3 closest = Vector3Clamp(local, Vector3Negate(half), half);
        Vector3 delta = Vector3Subtract(local, closest);
        float distance = Vector3Length(delta);
        if (distance > 0.0001f)
        {
            if (distance >= r)
            {
                return;
            }
            normal = Vector3Scale(delta, 1.0f / distance);
            d = distance;
            surface = closest;
        }
        else
        {
            d = HullDistance(h.planes, local, normal);
            surface = Vector3Subtract(local, Vector3Scale(normal, d));
        }
    }
    else
    {
        d = HullDistance(h.planes, local, normal);
        if (d >= r)
        {
            return;
        }
        surface = Vector3Subtract(local, Vector3Scale(normal, d));
    }

    Vector3 worldNormal = RotateToWorld(h, normal);
    Vector3 point = ToWorld(h, surface);
    if (sphere < hull)
    {
        AddContact(contacts, sphere, hull, point, Vector3Negate(worldNormal), r - d, friction);
    }
    else
    {
        AddContact(contacts, hull, sphere, point, worldNormal, r - d, friction);
    }
}

void BodyContacts(const std::vector<Body> &bodies, const std::vector<WorldHull> &hulls, int i, int j,
                  const World &world, std::vector<Contact> &contacts)
{
    const Body &a = bodies[i];
    const Body &b = bodies[j];
    if (Vector3Distance(a.position, b.position) > BoundingRadius(a) + BoundingRadius(b))
    {
        return;
    }
    float friction = PairFriction(world, MaterialOf(a).friction, MaterialOf(b).friction);

    if (a.hull.empty() && b.hull.empty())
    {
        Vector3 delta = Vector3Subtract(b.position, a.position);
        float distance = Vector3Length(delta);
        float reach = (a.size.x + b.size.x) / 2.0f;
        if (distance >= reach)
        {
            return;
        }
        Vector3 normal = distance > 0.0001f ? Vector3Scale(delta, 1.0f / distance) : Vector3{0.0f, 1.0f, 0.0f};
        Vector3 point = Vector3Add(a.position, Vector3Scale(normal, a.size.x / 2.0f));
        AddContact(contacts, i, j, point, normal, reach - distance, friction);
        return;
    }
    if (a.hull.empty())
    {
        SphereHullContact(bodies, i, j, friction, contacts);
        return;
    }
    if (b.hull.empty())
    {
        SphereHullContact(bodies, j, i, friction, contacts);
        return;
    }
    HullContacts(hulls[i], hulls[j], i, j, friction, contacts);
}

Vector3 RelativeVelocity(const std::vector<Body> &bodies, const Contact &c)
{
    const Body &a = bodies[c.a];
    Vector3 va = PointVelocity(a, Vector3Subtract(c.point, a.position));
    if (c.b == STATIC_BODY)
    {
        return Vector3Negate(va);
    }
    const Body &b = bodies[c.b];
    return Vector3Subtract(PointVelocity(b, Vector3Subtract(c.point, b.position)), va);
}

float EffectiveMass(const std::vector<Body> &bodies, const Contact &c, Vector3 direction)
{
    const Body &a = bodies[c.a];
    float k = InverseMass(a) + AngularMass(a, Vector3Subtract(c.point, a.position), direction);
    if (c.b != STATIC_BODY)
    {
        const Body &b = bodies[c.b];
        k += InverseMass(b) + AngularMass(b, Vector3Subtract(c.point, b.position), direction);
    }
    return k;
}

void ApplyContactImpulse(std::vector<Body> &bodies, const Contact &c, Vector3 impulse)
{
    Body &a = bodies[c.a];
    ApplyImpulse(a, Vector3Negate(impulse), Vector3Subtract(c.point, a.position));
    if (c.b != STATIC_BODY)
    {
        Body &b = bodies[c.b];
        ApplyImpulse(b, impulse, Vector3Subtract(c.point, b.position));
    }
}

float ContactBounce(const std::vector<Body> &bodies, const Contact &c)
{
    float bounce = MaterialOf(bodies[c.a]).bounce;
    if (c.b != STATIC_BODY)
    {
        bounce = std::min(bounce, MaterialOf(bodies[c.b]).bounce);
    }
    return bounce;
}

void PrepareContacts(const std::vector<Body> &bodies, std::vector<Contact> &contacts, float dt)
{
    for (Contact &c : contacts)
    {
        float approach = Vector3DotProduct(RelativeVelocity(bodies, c), c.normal);
        c.target = approach < -REST_SPEED ? -ContactBounce(bodies, c) * approach : 0.0f;
        c.push = std::min(CORRECTION * std::max(c.depth - CONTACT_SLOP, 0.0f) / dt, MAX_CORRECTION_SPEED);
    }
}

struct Pseudo
{
    Vector3 linear;
    Vector3 angular;
};

void SolvePush(const std::vector<Body> &bodies, std::vector<Pseudo> &pseudo, Contact &c)
{
    const Body &a = bodies[c.a];
    Vector3 ra = Vector3Subtract(c.point, a.position);
    Vector3 va = Vector3Add(pseudo[c.a].linear, Vector3CrossProduct(pseudo[c.a].angular, ra));
    Vector3 vb = Vector3Zero();
    Vector3 rb = Vector3Zero();
    if (c.b != STATIC_BODY)
    {
        rb = Vector3Subtract(c.point, bodies[c.b].position);
        vb = Vector3Add(pseudo[c.b].linear, Vector3CrossProduct(pseudo[c.b].angular, rb));
    }
    float approach = Vector3DotProduct(Vector3Subtract(vb, va), c.normal);
    float impulse = (c.push - approach) / EffectiveMass(bodies, c, c.normal);
    float total = std::max(c.pushImpulse + impulse, 0.0f);
    impulse = total - c.pushImpulse;
    c.pushImpulse = total;

    Vector3 j = Vector3Scale(c.normal, impulse);
    pseudo[c.a].linear = Vector3Subtract(pseudo[c.a].linear, Vector3Scale(j, InverseMass(a)));
    pseudo[c.a].angular = Vector3Subtract(pseudo[c.a].angular, ApplyInverseInertia(a, Vector3CrossProduct(ra, j)));
    if (c.b != STATIC_BODY)
    {
        const Body &b = bodies[c.b];
        pseudo[c.b].linear = Vector3Add(pseudo[c.b].linear, Vector3Scale(j, InverseMass(b)));
        pseudo[c.b].angular = Vector3Add(pseudo[c.b].angular, ApplyInverseInertia(b, Vector3CrossProduct(rb, j)));
    }
}

void SolveContact(std::vector<Body> &bodies, Contact &c)
{
    Vector3 relative = RelativeVelocity(bodies, c);
    float approach = Vector3DotProduct(relative, c.normal);
    float k = EffectiveMass(bodies, c, c.normal);
    float impulse = (c.target - approach) / k;
    float total = std::max(c.normalImpulse + impulse, 0.0f);
    impulse = total - c.normalImpulse;
    c.normalImpulse = total;
    ApplyContactImpulse(bodies, c, Vector3Scale(c.normal, impulse));

    relative = RelativeVelocity(bodies, c);
    Vector3 tangent = Vector3Subtract(relative, Vector3Scale(c.normal, Vector3DotProduct(relative, c.normal)));
    float slide = Vector3Length(tangent);
    if (slide < 0.00001f)
    {
        return;
    }
    Vector3 direction = Vector3Scale(tangent, 1.0f / slide);
    Vector3 change = Vector3Scale(direction, -slide / EffectiveMass(bodies, c, direction));
    Vector3 friction = Vector3Add(c.frictionImpulse, change);
    float limit = c.friction * c.normalImpulse;
    float amount = Vector3Length(friction);
    if (amount > limit)
    {
        friction = Vector3Scale(friction, limit / amount);
    }
    change = Vector3Subtract(friction, c.frictionImpulse);
    c.frictionImpulse = friction;
    ApplyContactImpulse(bodies, c, change);
}

float SubmergedFraction(const Body &b, float level)
{
    float low;
    float high;
    Extent(b, low, high);
    float u = std::clamp((level - low) / std::max(high - low, 0.0001f), 0.0f, 1.0f);
    if (b.shape == Shape::Sphere || b.shape == Shape::Ellipsoid)
    {
        return u * u * (3.0f - 2.0f * u);
    }
    return u;
}

float LiquidFraction(const Body &b, const World &world)
{
    if (!HasLiquid(world) || !InsideTank(b.position))
    {
        return 0.0f;
    }
    return SubmergedFraction(b, world.level);
}

Vector3 BuoyancyCenter(const Body &b, float level)
{
    if (b.hull.empty())
    {
        float r = b.size.x / 2.0f;
        float top = std::min(level, b.position.y + r);
        float bottom = b.position.y - r;
        return {b.position.x, (top + bottom) / 2.0f, b.position.z};
    }
    Vector3 sum = {0.0f, 0.0f, 0.0f};
    int count = 0;
    for (const Vector3 &v : b.hull)
    {
        Vector3 w = ToWorld(b, v);
        if (w.y < level)
        {
            sum = Vector3Add(sum, w);
            count++;
        }
    }
    return count > 0 ? Vector3Scale(sum, 1.0f / count) : b.position;
}

float BuoyantForce(const Body &b, const World &world)
{
    float fraction = LiquidFraction(b, world);
    return LIQUIDS[world.liquid].density * Volume(b) * fraction * GRAVITIES[world.gravity].value;
}

void ApplyLiquid(Body &b, const World &world, float dt)
{
    float fraction = LiquidFraction(b, world);
    if (fraction <= 0.0f)
    {
        return;
    }

    const Liquid &liquid = LIQUIDS[world.liquid];
    Vector3 lift = {0.0f, BuoyantForce(b, world) * dt, 0.0f};
    ApplyImpulse(b, lift, Vector3Subtract(BuoyancyCenter(b, world.level), b.position));

    float wetted = std::max(fraction, SURFACE_DAMPING);
    float drag = std::max(1.0f - liquid.drag * wetted * sqrtf(liquid.density / Density(b)) * dt, 0.0f);
    b.velocity = Vector3Scale(b.velocity, drag);
    b.angularVelocity = Vector3Scale(b.angularVelocity, drag);
}

void Splash(World &world, Vector3 at, float speed)
{
    world.ripples.push_back({{at.x, at.z}, 0.0f, std::min(speed / 6.0f, 1.0f), 2.4f});

    int count = std::min((int)(speed * 3.0f), 30);
    for (int i = 0; i < count; i++)
    {
        float angle = RandomFloat(0.0f, 2.0f * PI);
        float out = RandomFloat(0.15f, 0.6f) * sqrtf(speed);
        Vector3 velocity = {cosf(angle) * out, RandomFloat(1.0f, 2.0f) * sqrtf(speed), sinf(angle) * out};
        world.droplets.push_back({{at.x, world.level, at.z}, velocity, RandomFloat(0.6f, 1.2f)});
    }
}

void UpdateEffects(World &world, float gravity, float dt)
{
    if (!HasLiquid(world))
    {
        world.ripples.clear();
        world.droplets.clear();
        return;
    }

    for (Ripple &r : world.ripples)
    {
        r.age += dt;
    }
    std::erase_if(world.ripples, [](const Ripple &r) { return r.age >= RIPPLE_LIFE; });

    for (Droplet &d : world.droplets)
    {
        d.velocity.y -= gravity * dt;
        d.position = Vector3Add(d.position, Vector3Scale(d.velocity, dt));
        d.life -= dt;
    }
    std::erase_if(world.droplets, [&](const Droplet &d) {
        bool landed = d.velocity.y < 0.0f && d.position.y < world.level && InsideTank(d.position);
        return d.life <= 0.0f || landed || d.position.y < 0.0f;
    });
}

Vector3 WindVector(const World &world)
{
    float angle = world.windAngle * DEG2RAD;
    return {cosf(angle) * world.windSpeed, 0.0f, sinf(angle) * world.windSpeed};
}

float CrossSection(const Body &b)
{
    const Vector3 &s = b.size;
    if (IsRound(b))
    {
        return PI * s.x * s.x / 4.0f;
    }
    return (s.x * s.y + s.y * s.z + s.x * s.z) / 3.0f;
}

void ApplyAir(Body &b, const World &world, float dt)
{
    if (!world.airDrag && world.windSpeed <= 0.0f)
    {
        return;
    }
    float exposed = 1.0f - LiquidFraction(b, world);
    Vector3 relative = Vector3Subtract(WindVector(world), b.velocity);
    float speed = Vector3Length(relative);
    if (exposed <= 0.0f || speed < 0.0001f)
    {
        return;
    }

    float cd = IsRound(b) ? 0.47f : 1.05f;
    float k = 0.5f * AIR_DENSITY * cd * CrossSection(b) * speed * exposed / Mass(b);
    b.velocity = Vector3Add(b.velocity, Vector3Scale(relative, std::min(k * dt, 1.0f)));
}

Vector3 JointEnd(const std::vector<Body> &bodies, const Joint &joint)
{
    return joint.b == NO_SELECTION ? joint.anchor : bodies[joint.b].position;
}

float JointInverseMass(const std::vector<Body> &bodies, int index)
{
    return index == NO_SELECTION ? 0.0f : InverseMass(bodies[index]);
}

void ApplySprings(std::vector<Body> &bodies, const World &world, float dt)
{
    for (const Joint &joint : world.joints)
    {
        if (joint.type != JointType::Spring)
        {
            continue;
        }
        Body &a = bodies[joint.a];
        Vector3 delta = Vector3Subtract(JointEnd(bodies, joint), a.position);
        float length = Vector3Length(delta);
        if (length < 0.0001f)
        {
            continue;
        }
        Vector3 direction = Vector3Scale(delta, 1.0f / length);
        float invA = InverseMass(a);
        float invB = JointInverseMass(bodies, joint.b);
        float stiffness = std::min(joint.stiffness, 0.5f / ((invA + invB) * dt * dt));
        Vector3 vb = joint.b == NO_SELECTION ? Vector3Zero() : bodies[joint.b].velocity;
        float stretchSpeed = Vector3DotProduct(Vector3Subtract(vb, a.velocity), direction);
        float damping = SPRING_DAMPING * 2.0f * sqrtf(stiffness / (invA + invB));
        float force = stiffness * (length - joint.length) + damping * stretchSpeed;
        Vector3 impulse = Vector3Scale(direction, force * dt);
        a.velocity = Vector3Add(a.velocity, Vector3Scale(impulse, invA));
        if (joint.b != NO_SELECTION)
        {
            Body &b = bodies[joint.b];
            b.velocity = Vector3Subtract(b.velocity, Vector3Scale(impulse, invB));
        }
    }
}

void SolveJoints(std::vector<Body> &bodies, const World &world)
{
    for (const Joint &joint : world.joints)
    {
        Body &a = bodies[joint.a];
        float invA = InverseMass(a);
        if (joint.type == JointType::Pulley)
        {
            Body &b = bodies[joint.b];
            float invB = InverseMass(b);
            Vector3 ua = Vector3Normalize(Vector3Subtract(a.position, joint.anchor));
            Vector3 ub = Vector3Normalize(Vector3Subtract(b.position, joint.pulley));
            float total = Vector3Distance(a.position, joint.anchor) + Vector3Distance(b.position, joint.pulley);
            float growth = Vector3DotProduct(a.velocity, ua) + Vector3DotProduct(b.velocity, ub);
            if (total >= joint.length && growth > 0.0f)
            {
                float lambda = growth / (invA + invB);
                a.velocity = Vector3Subtract(a.velocity, Vector3Scale(ua, lambda * invA));
                b.velocity = Vector3Subtract(b.velocity, Vector3Scale(ub, lambda * invB));
            }
            continue;
        }
        if (joint.type == JointType::Spring)
        {
            continue;
        }

        Vector3 delta = Vector3Subtract(JointEnd(bodies, joint), a.position);
        float length = Vector3Length(delta);
        if (length < 0.0001f || (joint.type == JointType::Rope && length < joint.length))
        {
            continue;
        }
        Vector3 direction = Vector3Scale(delta, 1.0f / length);
        float invB = JointInverseMass(bodies, joint.b);
        Vector3 vb = joint.b == NO_SELECTION ? Vector3Zero() : bodies[joint.b].velocity;
        float growth = Vector3DotProduct(Vector3Subtract(vb, a.velocity), direction);
        if (joint.type == JointType::Rope && growth <= 0.0f)
        {
            continue;
        }
        float lambda = growth / (invA + invB);
        a.velocity = Vector3Add(a.velocity, Vector3Scale(direction, lambda * invA));
        if (joint.b != NO_SELECTION)
        {
            Body &b = bodies[joint.b];
            b.velocity = Vector3Subtract(b.velocity, Vector3Scale(direction, lambda * invB));
        }
    }
}

void CorrectJoints(std::vector<Body> &bodies, const World &world)
{
    for (const Joint &joint : world.joints)
    {
        Body &a = bodies[joint.a];
        float invA = InverseMass(a);
        if (joint.type == JointType::Pulley)
        {
            Body &b = bodies[joint.b];
            float invB = InverseMass(b);
            float total = Vector3Distance(a.position, joint.anchor) + Vector3Distance(b.position, joint.pulley);
            float excess = (total - joint.length) * JOINT_CORRECTION;
            if (excess > 0.0f)
            {
                Vector3 ua = Vector3Normalize(Vector3Subtract(a.position, joint.anchor));
                Vector3 ub = Vector3Normalize(Vector3Subtract(b.position, joint.pulley));
                a.position = Vector3Subtract(a.position, Vector3Scale(ua, excess * invA / (invA + invB)));
                b.position = Vector3Subtract(b.position, Vector3Scale(ub, excess * invB / (invA + invB)));
            }
            continue;
        }
        if (joint.type == JointType::Spring)
        {
            continue;
        }

        Vector3 delta = Vector3Subtract(JointEnd(bodies, joint), a.position);
        float length = Vector3Length(delta);
        float error = length - joint.length;
        if (length < 0.0001f || (joint.type == JointType::Rope && error <= 0.0f))
        {
            continue;
        }
        float invB = JointInverseMass(bodies, joint.b);
        Vector3 shift = Vector3Scale(delta, error / length * JOINT_CORRECTION / (invA + invB));
        a.position = Vector3Add(a.position, Vector3Scale(shift, invA));
        if (joint.b != NO_SELECTION)
        {
            Body &b = bodies[joint.b];
            b.position = Vector3Subtract(b.position, Vector3Scale(shift, invB));
        }
    }
}

bool CanSleep(const Body &b, const World &world, int index)
{
    if (Vector3Length(b.pushDirection) > 0.0f || LiquidFraction(b, world) > 0.0f)
    {
        return false;
    }
    for (const Joint &joint : world.joints)
    {
        if (joint.a == index || joint.b == index)
        {
            return false;
        }
    }
    return true;
}

void IntegrateOrientation(Body &b, Vector3 w, float dt)
{
    Quaternion spin = QuaternionMultiply({w.x, w.y, w.z, 0.0f}, b.orientation);
    Quaternion q = b.orientation;
    q.x += 0.5f * spin.x * dt;
    q.y += 0.5f * spin.y * dt;
    q.z += 0.5f * spin.z * dt;
    q.w += 0.5f * spin.w * dt;
    b.orientation = QuaternionNormalize(q);
}

void StepPhysics(std::vector<Body> &bodies, World &world, float dt)
{
    float gravity = GRAVITIES[world.gravity].value;
    for (Body &b : bodies)
    {
        b.velocity.y -= gravity * dt;
        b.velocity = Vector3Add(b.velocity, Vector3Scale(b.pushDirection, world.pushForce * InverseMass(b) * dt));
        ApplyLiquid(b, world, dt);
        ApplyAir(b, world, dt);
        b.angularVelocity = Vector3Scale(b.angularVelocity, std::max(1.0f - ANGULAR_DAMPING * dt, 0.0f));
    }
    ApplySprings(bodies, world, dt);

    std::vector<Contact> contacts;
    std::vector<WorldHull> statics;
    for (const StaticHull &hull : StaticHulls(world))
    {
        statics.push_back(MakeWorldHull(hull));
    }
    std::vector<WorldHull> hulls;
    for (const Body &b : bodies)
    {
        hulls.push_back(MakeWorldHull(b));
    }
    for (int i = 0; i < (int)bodies.size(); i++)
    {
        for (int j = i + 1; j < (int)bodies.size(); j++)
        {
            BodyContacts(bodies, hulls, i, j, world, contacts);
        }
        for (const WorldHull &hull : statics)
        {
            StaticContacts(bodies, hulls, i, hull, world, contacts);
        }
        FloorContacts(bodies, i, world, contacts);
    }

    PrepareContacts(bodies, contacts, dt);
    std::vector<Pseudo> pseudo(bodies.size(), {Vector3Zero(), Vector3Zero()});
    for (int iteration = 0; iteration < SOLVER_ITERATIONS; iteration++)
    {
        for (Contact &c : contacts)
        {
            SolveContact(bodies, c);
            SolvePush(bodies, pseudo, c);
        }
        SolveJoints(bodies, world);
    }

    std::vector<bool> touching(bodies.size(), false);
    for (const Contact &c : contacts)
    {
        touching[c.a] = true;
        if (c.b != STATIC_BODY)
        {
            touching[c.b] = true;
        }
    }

    for (size_t i = 0; i < bodies.size(); i++)
    {
        Body &b = bodies[i];
        if (touching[i] && Rolls(b))
        {
            b.angularVelocity = Vector3Scale(b.angularVelocity, std::max(1.0f - ROLLING_RESISTANCE * dt, 0.0f));
        }
        bool slow = Vector3Length(b.velocity) < SLEEP_SPEED && Vector3Length(b.angularVelocity) < SLEEP_SPEED;
        b.rest = touching[i] && slow && CanSleep(b, world, (int)i) ? b.rest + dt : 0.0f;
        if (b.rest > SLEEP_TIME)
        {
            b.velocity = Vector3Zero();
            b.angularVelocity = Vector3Zero();
        }

        bool above = b.position.y > world.level;
        b.position = Vector3Add(b.position, Vector3Scale(Vector3Add(b.velocity, pseudo[i].linear), dt));
        IntegrateOrientation(b, Vector3Add(b.angularVelocity, pseudo[i].angular), dt);
        if (HasLiquid(world) && above && b.position.y <= world.level && InsideTank(b.position) &&
            b.velocity.y < -SPLASH_SPEED)
        {
            Splash(world, b.position, -b.velocity.y);
        }
    }

    CorrectJoints(bodies, world);
}

void UpdateRain(World &world, Vector3 viewer, float dt)
{
    if (!world.rain)
    {
        world.raindrops.clear();
        return;
    }

    Vector3 wind = WindVector(world);
    float wanted = RAIN_RATE * dt;
    int count = (int)wanted + (RandomFloat(0.0f, 1.0f) < wanted - (int)wanted ? 1 : 0);
    for (int i = 0; i < count && (int)world.raindrops.size() < MAX_RAINDROPS; i++)
    {
        Vector3 position = {viewer.x + RandomFloat(-RAIN_RADIUS, RAIN_RADIUS),
                            std::max(viewer.y, 0.0f) + RandomFloat(2.0f, 14.0f),
                            viewer.z + RandomFloat(-RAIN_RADIUS, RAIN_RADIUS)};
        world.raindrops.push_back({position, {wind.x, -RAIN_SPEED, wind.z}, 0.0f});
    }

    for (Droplet &d : world.raindrops)
    {
        d.position = Vector3Add(d.position, Vector3Scale(d.velocity, dt));
        bool inLiquid = HasLiquid(world) && InsideTank(d.position) && d.position.y < world.level;
        if (inLiquid && RandomFloat(0.0f, 1.0f) < RAIN_RIPPLE_CHANCE)
        {
            world.ripples.push_back({{d.position.x, d.position.z}, 0.0f, 0.35f, 0.35f});
        }
        if (inLiquid || d.position.y < 0.0f)
        {
            d.life = -1.0f;
        }
    }
    std::erase_if(world.raindrops, [](const Droplet &d) { return d.life < 0.0f; });
}

float UpdatePhysics(std::vector<Body> &bodies, World &world, Vector3 viewer, float dt)
{
    float simDt = std::min(dt, MAX_FRAME_TIME) * TIME_SCALES[world.timeScale].value;
    int steps = std::max((int)ceilf(simDt / PHYSICS_STEP), 1);
    for (int i = 0; i < steps; i++)
    {
        StepPhysics(bodies, world, simDt / steps);
    }
    for (Body &b : bodies)
    {
        b.age += simDt;
        b.maxHeight = std::max(b.maxHeight, BottomHeight(b));
    }
    UpdateEffects(world, GRAVITIES[world.gravity].value, simDt);
    UpdateRain(world, viewer, simDt);
    return simDt;
}

void RecordTrails(std::vector<Body> &bodies, bool enabled)
{
    for (Body &b : bodies)
    {
        if (!enabled)
        {
            b.trail.clear();
            continue;
        }
        if (b.trail.empty() || Vector3Distance(b.trail.back(), b.position) > TRAIL_STEP)
        {
            b.trail.push_back(b.position);
        }
        if ((int)b.trail.size() > MAX_TRAIL)
        {
            b.trail.erase(b.trail.begin());
        }
    }
}

void RemoveBody(std::vector<Body> &bodies, World &world, int index)
{
    bodies.erase(bodies.begin() + index);
    std::erase_if(world.joints, [index](const Joint &j) { return j.a == index || j.b == index; });
    for (Joint &j : world.joints)
    {
        if (j.a > index)
        {
            j.a--;
        }
        if (j.b > index)
        {
            j.b--;
        }
    }
}

void CutJoints(World &world, int index)
{
    std::erase_if(world.joints, [index](const Joint &j) { return j.a == index || j.b == index; });
}

void HangBody(const std::vector<Body> &bodies, World &world, int index)
{
    JointType type = (JointType)world.jointType;
    if (type == JointType::Pulley)
    {
        type = JointType::Rope;
    }
    Vector3 anchor = Vector3Add(bodies[index].position, {0.0f, world.jointLength, 0.0f});
    world.joints.push_back({type, index, NO_SELECTION, anchor, Vector3Zero(), world.jointLength, world.stiffness});
}

void LinkBodies(const std::vector<Body> &bodies, World &world, int a, int b)
{
    JointType type = (JointType)world.jointType;
    Vector3 pa = bodies[a].position;
    Vector3 pb = bodies[b].position;
    if (type == JointType::Pulley)
    {
        float top = std::max(pa.y, pb.y) + PULLEY_RAISE;
        Vector3 left = {pa.x, top, pa.z};
        Vector3 right = {pb.x, top, pb.z};
        float length = Vector3Distance(pa, left) + Vector3Distance(pb, right);
        world.joints.push_back({type, a, b, left, right, length, world.stiffness});
        return;
    }
    world.joints.push_back({type, a, b, Vector3Zero(), Vector3Zero(), Vector3Distance(pa, pb), world.stiffness});
}

float SpringStretch(const std::vector<Body> &bodies, const World &world, int index)
{
    for (const Joint &joint : world.joints)
    {
        if (joint.type == JointType::Spring && (joint.a == index || joint.b == index))
        {
            return Vector3Distance(bodies[joint.a].position, JointEnd(bodies, joint)) - joint.length;
        }
    }
    return 0.0f;
}

bool HasSpring(const World &world, int index)
{
    for (const Joint &joint : world.joints)
    {
        if (joint.type == JointType::Spring && (joint.a == index || joint.b == index))
        {
            return true;
        }
    }
    return false;
}

bool RayHitBody(Ray ray, const Body &b, float &distance)
{
    Ray local = {ToLocal(b, ray.position), RotateToLocal(b, ray.direction)};
    RayCollision hit = {};
    if (b.shape == Shape::Sphere)
    {
        hit = GetRayCollisionSphere(local, {0.0f, 0.0f, 0.0f}, b.size.x / 2.0f);
    }
    else if (b.shape == Shape::Ellipsoid)
    {
        Vector3 radii = Vector3Scale(b.size, 0.5f);
        Ray unit = {Vector3Divide(local.position, radii), Vector3Normalize(Vector3Divide(local.direction, radii))};
        hit = GetRayCollisionSphere(unit, {0.0f, 0.0f, 0.0f}, 1.0f);
        if (hit.hit)
        {
            hit.distance = Vector3Distance(local.position, Vector3Multiply(hit.point, radii));
        }
    }
    else
    {
        Vector3 half = Vector3Scale(b.size, 0.5f);
        hit = GetRayCollisionBox(local, {Vector3Negate(half), half});
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

bool IsMouseOverUi(const UiState &ui, bool hasSelection);

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

void UpdateCursor(const FreeCamera &cam, const std::vector<Body> &bodies, const Drag &drag, const UiState &ui,
                  bool hasSelection, int &cursor)
{
    Vector3 hitPoint;
    int wanted = MOUSE_CURSOR_DEFAULT;
    if (drag.active)
    {
        wanted = MOUSE_CURSOR_RESIZE_ALL;
    }
    else if (!cam.looking && !IsMouseOverUi(ui, hasSelection) &&
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
    b.angularVelocity = Vector3Scale(b.angularVelocity, 0.9f);
}

UiFont LoadUiFont(const char *file, float size)
{
    std::vector<int> codepoints;
    for (int c = 32; c < 127; c++)
    {
        codepoints.push_back(c);
    }
    for (int c : {0x00D7, 0x00B7, 0x00B3, 0x00B2, 0x00B0, 0x03B1, 0x03BC})
    {
        codepoints.push_back(c);
    }

    Font font = LoadFontEx(TextFormat("%s/fonts/%s", ASSETS_DIR, file), (int)size, codepoints.data(),
                           (int)codepoints.size());
    if (!IsFontValid(font))
    {
        font = GetFontDefault();
    }
    SetTextureFilter(font.texture, TEXTURE_FILTER_BILINEAR);
    return {font, size};
}

Fonts LoadFonts()
{
    return {LoadUiFont("Inter-Medium.ttf", 15.0f), LoadUiFont("Inter-SemiBold.ttf", 15.0f),
            LoadUiFont("Inter-SemiBold.ttf", 12.0f), LoadUiFont("Inter-SemiBold.ttf", 18.0f)};
}

void UnloadFonts(const Fonts &f)
{
    for (const UiFont *u : {&f.text, &f.bold, &f.small, &f.title})
    {
        if (u->font.texture.id != GetFontDefault().texture.id)
        {
            UnloadFont(u->font);
        }
    }
}

float TextWidth(const UiFont &f, const char *text)
{
    return MeasureTextEx(f.font, text, f.size, 0.0f).x;
}

void Text(const UiFont &f, const char *text, float x, float y, Color color)
{
    DrawTextEx(f.font, text, {roundf(x), roundf(y)}, f.size, 0.0f, color);
}

void TextRight(const UiFont &f, const char *text, float right, float y, Color color)
{
    Text(f, text, right - TextWidth(f, text), y, color);
}

void TextCentered(const UiFont &f, const char *text, Rectangle rect, Color color)
{
    Text(f, text, rect.x + (rect.width - TextWidth(f, text)) / 2.0f, rect.y + (rect.height - f.size) / 2.0f, color);
}

void FillRounded(Rectangle rect, float radius, Color color)
{
    DrawRectangleRounded(rect, 2.0f * radius / std::min(rect.width, rect.height), 8, color);
}

void StrokeRounded(Rectangle rect, float radius, Color color)
{
    DrawRectangleRoundedLinesEx(rect, 2.0f * radius / std::min(rect.width, rect.height), 8, 1.0f, color);
}

void DrawCard(Rectangle rect)
{
    FillRounded({rect.x - 2.0f, rect.y + 2.0f, rect.width + 4.0f, rect.height + 6.0f}, 14.0f, {0, 0, 0, 50});
    FillRounded(rect, 12.0f, COLOR_PANEL);
    StrokeRounded(rect, 12.0f, COLOR_BORDER);
}

bool IsHovered(Rectangle rect)
{
    return CheckCollisionPointRec(GetMousePosition(), rect);
}

Rectangle SettingsButtonRect()
{
    return {MARGIN, MARGIN, ICON_BUTTON_SIZE, ICON_BUTTON_SIZE};
}

Rectangle SettingsPanelRect()
{
    return {MARGIN, MARGIN + ICON_BUTTON_SIZE + 8.0f, SETTINGS_WIDTH, SETTINGS_HEIGHT};
}

Rectangle ToggleRowRect(int i)
{
    Rectangle panel = SettingsPanelRect();
    return {panel.x + SETTINGS_PADDING, panel.y + SETTINGS_TOGGLES_Y + i * TOGGLE_ROW,
            panel.width - 2.0f * SETTINGS_PADDING, TOGGLE_ROW};
}

Rectangle SegmentRect(Rectangle bar, int count, int i)
{
    float width = (bar.width - 6.0f) / count;
    return {bar.x + 3.0f + i * width, bar.y + 3.0f, width, bar.height - 6.0f};
}

int ClickedSegment(Rectangle bar, int count)
{
    if (!IsMouseButtonPressed(MOUSE_BUTTON_LEFT))
    {
        return NO_SELECTION;
    }
    for (int i = 0; i < count; i++)
    {
        if (IsHovered(SegmentRect(bar, count, i)))
        {
            return i;
        }
    }
    return NO_SELECTION;
}

bool SliderGrabbed(Rectangle track)
{
    Rectangle grab = {track.x - 8.0f, track.y - 30.0f, track.width + 16.0f, 40.0f};
    return IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && IsHovered(grab);
}

float SliderValue(Rectangle track, float min, float max)
{
    float t = std::clamp((GetMouseX() - track.x) / track.width, 0.0f, 1.0f);
    return min + t * (max - min);
}

Rectangle SpawnCardRect()
{
    float width = std::min(GetScreenWidth() - 2.0f * MARGIN, CARD_MAX_WIDTH);
    return {(GetScreenWidth() - width) / 2.0f, GetScreenHeight() - MARGIN - CARD_HEIGHT, width, CARD_HEIGHT};
}

Rectangle ShapeBarRect()
{
    Rectangle card = SpawnCardRect();
    return {card.x + CARD_PADDING, card.y + CARD_PADDING, card.width - 2.0f * CARD_PADDING, SEGMENT_HEIGHT};
}

Rectangle MaterialBarRect()
{
    Rectangle bar = ShapeBarRect();
    bar.y += SEGMENT_HEIGHT + SEGMENT_GAP;
    return bar;
}

float SliderRowTop()
{
    Rectangle bar = MaterialBarRect();
    return bar.y + bar.height + ROW_GAP;
}

Rectangle SpawnButtonRect()
{
    Rectangle card = SpawnCardRect();
    return {card.x + card.width - CARD_PADDING - SPAWN_BUTTON_WIDTH, SliderRowTop(), SPAWN_BUTTON_WIDTH,
            SPAWN_BUTTON_HEIGHT};
}

Rectangle SliderRect(int i)
{
    Rectangle bar = ShapeBarRect();
    float area = SpawnButtonRect().x - 24.0f - bar.x;
    float slot = area / SLIDER_SLOTS;
    return {bar.x + i * slot, SliderRowTop() + 30.0f, slot - 24.0f, 4.0f};
}

enum class ItemKind
{
    Segments,
    Slider,
    Toggle
};

struct PanelItem
{
    ItemKind kind = ItemKind::Toggle;
    const char *label = "";
    int World::*choice = nullptr;
    std::vector<const char *> options = {};
    float World::*value = nullptr;
    float min = 0.0f;
    float max = 1.0f;
    const char *format = "";
    bool World::*flag = nullptr;
    bool (*enabled)(const World &) = nullptr;
    const char *(*info)(const World &) = nullptr;
};

const char *TAB_NAMES[] = {"World", "Weather", "Lab", "Joints"};
const int TAB_COUNT = sizeof(TAB_NAMES) / sizeof(TAB_NAMES[0]);

std::vector<const char *> PresetNames(const Preset *presets, int count)
{
    std::vector<const char *> names;
    for (int i = 0; i < count; i++)
    {
        names.push_back(presets[i].name);
    }
    return names;
}

std::vector<const char *> LiquidNames()
{
    std::vector<const char *> names;
    for (const Liquid &l : LIQUIDS)
    {
        names.push_back(l.name);
    }
    return names;
}

const char *GravityInfo(const World &world)
{
    return TextFormat("%.2f m/s²", GRAVITIES[world.gravity].value);
}

const char *LiquidInfo(const World &world)
{
    return HasLiquid(world) ? TextFormat("%.0f kg/m³", LIQUIDS[world.liquid].density) : "";
}

const char *RampInfo(const World &world)
{
    return TextFormat("Slides when friction < tan α = %.2f", tanf(world.rampAngle * DEG2RAD));
}

const char *WindInfo(const World &world)
{
    const Preset scale[] = {{"Calm", 0.5f}, {"Breeze", 8.0f}, {"Strong wind", 17.0f}, {"Storm", 25.0f}};
    for (const Preset &p : scale)
    {
        if (world.windSpeed < p.value)
        {
            return p.name;
        }
    }
    return "Hurricane";
}

bool HasWind(const World &world)
{
    return world.windSpeed > 0.0f;
}

bool HasRamp(const World &world)
{
    return world.ramp;
}

bool HasCustomFriction(const World &world)
{
    return world.customFriction;
}

bool IsSpringJoint(const World &world)
{
    return (JointType)world.jointType == JointType::Spring;
}

const char *FrictionInfo(const World &world)
{
    return world.friction < 0.005f ? "Frictionless" : "Same μ for every surface";
}

const char *ThrowInfo(const World &world)
{
    return TextFormat("F throw · V give selected · angle %.0f°", world.viewPitch);
}

const char *PushInfo(const World &)
{
    return "P on selected · pushes where you look";
}

const char *DropInfo(const World &)
{
    return "Bottom height for N";
}

const char *JointInfo(const World &)
{
    return "H hang selected · J link · K cut";
}

const char *StiffnessInfo(const World &world)
{
    return TextFormat("1 kg stretches %.1f cm", 100.0f * GRAVITIES[world.gravity].value / world.stiffness);
}

PanelItem SegmentsItem(const char *label, int World::*choice, std::vector<const char *> options,
                       const char *(*info)(const World &) = nullptr)
{
    PanelItem item;
    item.kind = ItemKind::Segments;
    item.label = label;
    item.choice = choice;
    item.options = std::move(options);
    item.info = info;
    return item;
}

PanelItem SliderItem(const char *label, float World::*value, float min, float max, const char *format,
                     bool (*enabled)(const World &) = nullptr, const char *(*info)(const World &) = nullptr)
{
    PanelItem item;
    item.kind = ItemKind::Slider;
    item.label = label;
    item.value = value;
    item.min = min;
    item.max = max;
    item.format = format;
    item.enabled = enabled;
    item.info = info;
    return item;
}

PanelItem ToggleItem(const char *label, bool World::*flag)
{
    PanelItem item;
    item.kind = ItemKind::Toggle;
    item.label = label;
    item.flag = flag;
    return item;
}

std::vector<PanelItem> TabItems(int tab)
{
    switch (tab)
    {
    case 0:
        return {
            SegmentsItem("GRAVITY", &World::gravity, PresetNames(GRAVITIES, GRAVITY_COUNT), GravityInfo),
            SegmentsItem("TIME", &World::timeScale, PresetNames(TIME_SCALES, TIME_SCALE_COUNT)),
            SegmentsItem("LIQUID", &World::liquid, LiquidNames(), LiquidInfo),
            SliderItem("Liquid level", &World::level, MIN_LEVEL, MAX_LEVEL, "%.2f m", HasLiquid),
            ToggleItem("Custom friction", &World::customFriction),
            SliderItem("Friction μ", &World::friction, 0.0f, 1.0f, "%.2f", HasCustomFriction, FrictionInfo),
        };
    case 1:
        return {
            SliderItem("Wind speed", &World::windSpeed, 0.0f, MAX_WIND, "%.1f m/s", nullptr, WindInfo),
            SliderItem("Wind direction", &World::windAngle, 0.0f, 360.0f, "%.0f°", HasWind),
            ToggleItem("Air resistance", &World::airDrag),
            ToggleItem("Rain", &World::rain),
        };
    case 2:
        return {
            ToggleItem("Inclined plane", &World::ramp),
            SliderItem("Plane angle", &World::rampAngle, 5.0f, 60.0f, "%.0f°", HasRamp, RampInfo),
            SliderItem("Speed", &World::launchSpeed, 0.5f, 40.0f, "%.1f m/s", nullptr, ThrowInfo),
            SliderItem("Push force", &World::pushForce, 0.0f, 500.0f, "%.0f N", nullptr, PushInfo),
            SliderItem("Drop height", &World::dropHeight, 0.0f, 40.0f, "%.1f m", nullptr, DropInfo),
            ToggleItem("Trajectories", &World::trails),
            ToggleItem("Velocity & force vectors", &World::vectors),
        };
    default:
    {
        std::vector<const char *> names(JOINT_NAMES, JOINT_NAMES + JOINT_TYPE_COUNT);
        return {
            SegmentsItem("TYPE", &World::jointType, names),
            SliderItem("Hang length", &World::jointLength, 0.3f, 10.0f, "%.2f m", nullptr, JointInfo),
            SliderItem("Stiffness k", &World::stiffness, 5.0f, 1000.0f, "%.0f N/m", IsSpringJoint, StiffnessInfo),
        };
    }
    }
}

float ItemHeight(const PanelItem &item)
{
    switch (item.kind)
    {
    case ItemKind::Segments: return WORLD_SECTION;
    case ItemKind::Slider: return WORLD_SLIDER + (item.info != nullptr ? WORLD_HINT : 0.0f);
    default: return WORLD_TOGGLE;
    }
}

float WorldCardHeight(int tab)
{
    float height = 2.0f * WORLD_PADDING + WORLD_SEGMENT_HEIGHT + 14.0f;
    for (const PanelItem &item : TabItems(tab))
    {
        height += ItemHeight(item);
    }
    return height - 8.0f;
}

Rectangle WorldCardRect(int tab)
{
    return {MARGIN, MARGIN + ICON_BUTTON_SIZE + 8.0f, WORLD_WIDTH, WorldCardHeight(tab)};
}

Rectangle WorldTabsRect(int tab)
{
    Rectangle card = WorldCardRect(tab);
    return {card.x + WORLD_PADDING, card.y + WORLD_PADDING, card.width - 2.0f * WORLD_PADDING,
            WORLD_SEGMENT_HEIGHT};
}

std::vector<Rectangle> WorldItemRects(int tab, const std::vector<PanelItem> &items)
{
    Rectangle card = WorldCardRect(tab);
    float y = card.y + WORLD_PADDING + WORLD_SEGMENT_HEIGHT + 14.0f;
    std::vector<Rectangle> rects;
    for (const PanelItem &item : items)
    {
        float height = ItemHeight(item);
        rects.push_back({card.x + WORLD_PADDING, y, card.width - 2.0f * WORLD_PADDING, height});
        y += height;
    }
    return rects;
}

Rectangle ItemBar(Rectangle rect)
{
    return {rect.x, rect.y + 18.0f, rect.width, WORLD_SEGMENT_HEIGHT};
}

Rectangle ItemTrack(Rectangle rect)
{
    return {rect.x + 8.0f, rect.y + 26.0f, rect.width - 16.0f, 4.0f};
}

Rectangle ItemToggleRow(Rectangle rect)
{
    return {rect.x + 8.0f, rect.y, rect.width - 16.0f, WORLD_TOGGLE};
}

bool ItemEnabled(const PanelItem &item, const World &world)
{
    return item.enabled == nullptr || item.enabled(world);
}

Rectangle SelectedCardRect()
{
    return {GetScreenWidth() - MARGIN - SELECTED_WIDTH, MARGIN, SELECTED_WIDTH, SELECTED_HEIGHT};
}

Rectangle GraphCardRect()
{
    Rectangle selected = SelectedCardRect();
    return {selected.x, selected.y + selected.height + 8.0f, selected.width, GRAPH_HEIGHT};
}

Rectangle GraphTabsRect()
{
    Rectangle card = GraphCardRect();
    return {card.x + 12.0f, card.y + 12.0f, card.width - 24.0f, 26.0f};
}

const char *GRAPH_NAMES[] = {"Height", "Speed", "Energy"};
const int GRAPH_COUNT = sizeof(GRAPH_NAMES) / sizeof(GRAPH_NAMES[0]);

bool IsMouseOverUi(const UiState &ui, bool hasSelection)
{
    return IsHovered(SpawnCardRect()) || IsHovered(SettingsButtonRect()) || (!ui.settingsOpen && IsHovered(WorldCardRect(ui.worldTab))) ||
           (ui.settingsOpen && IsHovered(SettingsPanelRect())) || (hasSelection && (IsHovered(SelectedCardRect()) || IsHovered(GraphCardRect())));
}

void UpdateSettings(UiState &ui)
{
    if (IsKeyPressed(KEY_ESCAPE))
    {
        ui.settingsOpen = !ui.settingsOpen;
    }
    if (!IsMouseButtonPressed(MOUSE_BUTTON_LEFT))
    {
        return;
    }

    if (IsHovered(SettingsButtonRect()))
    {
        ui.settingsOpen = !ui.settingsOpen;
    }
    else if (ui.settingsOpen && !IsHovered(SettingsPanelRect()))
    {
        ui.settingsOpen = false;
    }
    else if (ui.settingsOpen)
    {
        for (int i = 0; i < TOGGLE_COUNT; i++)
        {
            if (IsHovered(ToggleRowRect(i)))
            {
                ui.*TOGGLES[i].field = !(ui.*TOGGLES[i].field);
            }
        }
    }
}

void UpdateWorldPanel(World &world, UiState &ui)
{
    int clicked = ClickedSegment(WorldTabsRect(ui.worldTab), TAB_COUNT);
    if (clicked != NO_SELECTION)
    {
        ui.worldTab = clicked;
        return;
    }

    std::vector<PanelItem> items = TabItems(ui.worldTab);
    std::vector<Rectangle> rects = WorldItemRects(ui.worldTab, items);
    for (size_t i = 0; i < items.size(); i++)
    {
        const PanelItem &item = items[i];
        int sliderId = WORLD_SLIDER_BASE + (int)i;
        switch (item.kind)
        {
        case ItemKind::Segments:
            clicked = ClickedSegment(ItemBar(rects[i]), (int)item.options.size());
            if (clicked != NO_SELECTION)
            {
                world.*item.choice = clicked;
            }
            break;
        case ItemKind::Slider:
            if (ItemEnabled(item, world) && SliderGrabbed(ItemTrack(rects[i])))
            {
                ui.activeSlider = sliderId;
            }
            if (ui.activeSlider == sliderId)
            {
                world.*item.value = SliderValue(ItemTrack(rects[i]), item.min, item.max);
            }
            break;
        case ItemKind::Toggle:
            if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && IsHovered(ItemToggleRow(rects[i])))
            {
                world.*item.flag = !(world.*item.flag);
            }
            break;
        }
    }
}

bool UpdateSpawnPanel(SpawnSettings &settings, int &activeSlider)
{
    int clicked = ClickedSegment(ShapeBarRect(), SHAPE_COUNT);
    if (clicked != NO_SELECTION)
    {
        settings.shape = (Shape)clicked;
    }
    clicked = ClickedSegment(MaterialBarRect(), MATERIAL_COUNT);
    if (clicked != NO_SELECTION)
    {
        settings.material = clicked;
    }

    std::vector<Param> params = GetParams(settings.shape);
    for (size_t i = 0; i < params.size(); i++)
    {
        if (SliderGrabbed(SliderRect((int)i)))
        {
            activeSlider = (int)i;
        }
    }
    if (SliderGrabbed(SliderRect(MASS_SLOT)))
    {
        activeSlider = MASS_SLIDER;
    }
    if (activeSlider == MASS_SLIDER)
    {
        settings.massSlider = SliderValue(SliderRect(MASS_SLOT), 0.0f, 1.0f);
        return false;
    }
    bool spawnSlider = activeSlider != NO_SELECTION && activeSlider < MASS_SLIDER;
    if (spawnSlider && activeSlider >= (int)params.size())
    {
        activeSlider = NO_SELECTION;
        spawnSlider = false;
    }
    if (spawnSlider)
    {
        const Param &p = params[activeSlider];
        settings.*p.field = SliderValue(SliderRect(activeSlider), p.min, p.max);
    }

    return IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && IsHovered(SpawnButtonRect());
}

bool UpdateUi(UiState &ui, SpawnSettings &settings, World &world, bool hasSelection)
{
    if (!IsMouseButtonDown(MOUSE_BUTTON_LEFT))
    {
        ui.activeSlider = NO_SELECTION;
    }
    UpdateSettings(ui);
    if (hasSelection)
    {
        int clicked = ClickedSegment(GraphTabsRect(), GRAPH_COUNT);
        if (clicked != NO_SELECTION)
        {
            ui.graphTab = clicked;
        }
    }
    if (!ui.settingsOpen)
    {
        UpdateWorldPanel(world, ui);
    }
    return UpdateSpawnPanel(settings, ui.activeSlider);
}

void DrawSlidersIcon(Vector2 center, Color color)
{
    const float knobs[] = {-3.0f, 4.0f, -1.0f};
    for (int i = 0; i < 3; i++)
    {
        float y = center.y + (i - 1) * 5.0f;
        DrawLineEx({center.x - 8.0f, y}, {center.x + 8.0f, y}, 1.5f, color);
        DrawCircleV({center.x + knobs[i], y}, 2.5f, color);
    }
}

void DrawToggleSwitch(Rectangle row, bool on)
{
    Rectangle pill = {row.x + row.width - 34.0f, row.y + (row.height - 18.0f) / 2.0f, 34.0f, 18.0f};
    FillRounded(pill, 9.0f, on ? COLOR_ACCENT : Color{70, 72, 78, 255});
    float knobX = on ? pill.x + pill.width - 9.0f : pill.x + 9.0f;
    DrawCircleV({knobX, pill.y + 9.0f}, 6.5f, on ? WHITE : Color{200, 201, 205, 255});
}

void DrawToggleRow(Rectangle row, const char *label, bool on)
{
    if (IsHovered(row))
    {
        FillRounded({row.x - 6.0f, row.y, row.width + 12.0f, row.height}, 6.0f, COLOR_SURFACE);
    }
    Text(fonts.text, label, row.x, row.y + (row.height - fonts.text.size) / 2.0f, COLOR_TEXT);
    DrawToggleSwitch(row, on);
}

float DrawKeyChip(const char *key, float right, float centerY)
{
    float width = std::max(TextWidth(fonts.small, key) + 12.0f, 20.0f);
    Rectangle chip = {right - width, centerY - 10.0f, width, 20.0f};
    FillRounded(chip, 5.0f, COLOR_SURFACE);
    StrokeRounded(chip, 5.0f, COLOR_BORDER);
    TextCentered(fonts.small, key, chip, COLOR_TEXT);
    return chip.x;
}

void DrawSectionLabel(const char *text, float x, float y)
{
    Text(fonts.small, text, x, y, COLOR_MUTED);
}

void DrawSettingsPanel(const UiState &ui)
{
    Rectangle panel = SettingsPanelRect();
    DrawCard(panel);

    float x = panel.x + SETTINGS_PADDING;
    float right = panel.x + panel.width - SETTINGS_PADDING;
    Text(fonts.title, "Settings", x, panel.y + SETTINGS_PADDING, COLOR_TEXT);

    DrawSectionLabel("DISPLAY", x, panel.y + SETTINGS_TOGGLES_Y - 20.0f);
    for (int i = 0; i < TOGGLE_COUNT; i++)
    {
        DrawToggleRow(ToggleRowRect(i), TOGGLES[i].name, ui.*TOGGLES[i].field);
    }

    DrawSectionLabel("CONTROLS", x, panel.y + SETTINGS_BINDINGS_Y - 20.0f);
    for (int i = 0; i < BINDING_COUNT; i++)
    {
        const Binding &b = BINDINGS[i];
        float top = panel.y + SETTINGS_BINDINGS_Y + i * BINDING_ROW;
        float centerY = top + BINDING_ROW / 2.0f;
        Text(fonts.text, b.action, x, centerY - fonts.text.size / 2.0f, COLOR_MUTED);

        int count = 0;
        while (count < 4 && b.keys[count] != nullptr)
        {
            count++;
        }
        float keyRight = right;
        for (int k = count - 1; k >= 0; k--)
        {
            keyRight = DrawKeyChip(b.keys[k], keyRight, centerY) - 4.0f;
        }
    }
}

void DrawTopBar(const UiState &ui, int bodyCount, const Stopwatch &stopwatch)
{
    Rectangle button = SettingsButtonRect();
    bool hover = IsHovered(button);
    Color fill = ui.settingsOpen ? COLOR_ACCENT : COLOR_PANEL;
    FillRounded(button, 10.0f, hover && !ui.settingsOpen ? Color{38, 40, 46, 240} : fill);
    StrokeRounded(button, 10.0f, COLOR_BORDER);
    DrawSlidersIcon({button.x + button.width / 2.0f, button.y + button.height / 2.0f}, COLOR_TEXT);

    const char *count = TextFormat("%d", bodyCount);
    const char *label = bodyCount == 1 ? "body" : "bodies";
    const char *fps = TextFormat("%d fps", GetFPS());
    float width = TextWidth(fonts.bold, count) + 5.0f + TextWidth(fonts.text, label) + 28.0f;
    if (ui.showFps)
    {
        width += TextWidth(fonts.text, fps) + 21.0f;
    }

    Rectangle chip = {button.x + button.width + 8.0f, button.y, width, ICON_BUTTON_SIZE};
    FillRounded(chip, 10.0f, COLOR_PANEL);
    StrokeRounded(chip, 10.0f, COLOR_BORDER);

    float textY = chip.y + (chip.height - fonts.text.size) / 2.0f;
    float x = chip.x + 14.0f;
    Text(fonts.bold, count, x, textY, COLOR_TEXT);
    x += TextWidth(fonts.bold, count) + 5.0f;
    Text(fonts.text, label, x, textY, COLOR_MUTED);
    x += TextWidth(fonts.text, label);
    if (ui.showFps)
    {
        DrawCircleV({x + 10.0f, chip.y + chip.height / 2.0f}, 1.5f, COLOR_MUTED);
        Text(fonts.text, fps, x + 21.0f, textY, COLOR_MUTED);
    }

    if (!stopwatch.running && stopwatch.time <= 0.0f)
    {
        return;
    }
    const char *time = TextFormat("%.2f s", stopwatch.time);
    Rectangle watch = {chip.x + chip.width + 8.0f, chip.y, TextWidth(fonts.bold, time) + 44.0f, ICON_BUTTON_SIZE};
    FillRounded(watch, 10.0f, COLOR_PANEL);
    StrokeRounded(watch, 10.0f, stopwatch.running ? ColorAlpha(COLOR_ACCENT, 0.6f) : COLOR_BORDER);
    Vector2 dial = {watch.x + 17.0f, watch.y + watch.height / 2.0f};
    DrawCircleLinesV(dial, 6.0f, stopwatch.running ? COLOR_ACCENT : COLOR_MUTED);
    float hand = stopwatch.time * 2.0f * PI;
    DrawLineEx(dial, {dial.x + sinf(hand) * 4.5f, dial.y - cosf(hand) * 4.5f}, 1.5f,
               stopwatch.running ? COLOR_ACCENT : COLOR_MUTED);
    Text(fonts.bold, time, watch.x + 30.0f, textY, COLOR_TEXT);
}

void DrawPill(const char *title, const char *hint, float y)
{
    float width = 14.0f + 8.0f + 8.0f + TextWidth(fonts.bold, title) + 10.0f + TextWidth(fonts.text, hint) + 16.0f;
    Rectangle pill = {(GetScreenWidth() - width) / 2.0f, y, width, ICON_BUTTON_SIZE};
    FillRounded(pill, pill.height / 2.0f, COLOR_PANEL);
    StrokeRounded(pill, pill.height / 2.0f, ColorAlpha(COLOR_ACCENT, 0.5f));

    float textY = pill.y + (pill.height - fonts.text.size) / 2.0f;
    float x = pill.x + 18.0f;
    DrawCircleV({x, pill.y + pill.height / 2.0f}, 4.0f, COLOR_ACCENT);
    x += 12.0f;
    Text(fonts.bold, title, x, textY, COLOR_TEXT);
    x += TextWidth(fonts.bold, title) + 10.0f;
    Text(fonts.text, hint, x, textY, COLOR_MUTED);
}

void DrawSegmented(Rectangle bar, const std::vector<const char *> &labels, int active, bool compact,
                   const Color *dots = nullptr)
{
    int count = (int)labels.size();
    FillRounded(bar, 8.0f, COLOR_SURFACE);
    for (int i = 0; i < count; i++)
    {
        Rectangle rect = SegmentRect(bar, count, i);
        bool selected = i == active;
        bool hover = IsHovered(rect);
        if (selected)
        {
            FillRounded(rect, 6.0f, COLOR_ACCENT);
        }
        else if (hover)
        {
            FillRounded(rect, 6.0f, COLOR_HOVER);
        }

        const UiFont &font = compact ? fonts.small : (selected ? fonts.bold : fonts.text);
        Color color = selected ? WHITE : (hover ? COLOR_TEXT : COLOR_MUTED);
        if (dots != nullptr)
        {
            float width = 10.0f + 6.0f + TextWidth(font, labels[i]);
            float x = rect.x + (rect.width - width) / 2.0f;
            DrawCircleV({x + 5.0f, rect.y + rect.height / 2.0f}, 5.0f, dots[i]);
            DrawCircleLinesV({x + 5.0f, rect.y + rect.height / 2.0f}, 5.0f, {0, 0, 0, 60});
            Text(font, labels[i], x + 16.0f, rect.y + (rect.height - font.size) / 2.0f, color);
        }
        else
        {
            TextCentered(font, labels[i], rect, color);
        }
    }
}

void DrawSlider(Rectangle track, const char *label, const char *value, float t, bool active, bool enabled)
{
    float labelY = track.y - 26.0f;
    Text(fonts.text, label, track.x, labelY, COLOR_MUTED);
    TextRight(fonts.bold, value, track.x + track.width, labelY, enabled ? COLOR_TEXT : COLOR_MUTED);

    FillRounded(track, 2.0f, {255, 255, 255, 28});
    if (!enabled)
    {
        return;
    }
    if (t > 0.0f)
    {
        FillRounded({track.x, track.y, std::max(track.width * t, 4.0f), track.height}, 2.0f, COLOR_ACCENT);
    }
    Vector2 knob = {track.x + track.width * t, track.y + track.height / 2.0f};
    if (active)
    {
        DrawCircleV(knob, 11.0f, ColorAlpha(COLOR_ACCENT, 0.3f));
    }
    DrawCircleV(knob, 7.0f, WHITE);
}

void DrawWorldPanel(const World &world, const UiState &ui)
{
    Rectangle card = WorldCardRect(ui.worldTab);
    DrawCard(card);
    DrawSegmented(WorldTabsRect(ui.worldTab), std::vector<const char *>(TAB_NAMES, TAB_NAMES + TAB_COUNT), ui.worldTab,
                  false);

    std::vector<PanelItem> items = TabItems(ui.worldTab);
    std::vector<Rectangle> rects = WorldItemRects(ui.worldTab, items);
    for (size_t i = 0; i < items.size(); i++)
    {
        const PanelItem &item = items[i];
        Rectangle rect = rects[i];
        bool enabled = ItemEnabled(item, world);
        switch (item.kind)
        {
        case ItemKind::Segments:
            DrawSectionLabel(item.label, rect.x, rect.y);
            if (item.info != nullptr)
            {
                TextRight(fonts.small, item.info(world), rect.x + rect.width, rect.y, COLOR_MUTED);
            }
            DrawSegmented(ItemBar(rect), item.options, world.*item.choice, true);
            break;
        case ItemKind::Slider:
        {
            Rectangle track = ItemTrack(rect);
            float value = world.*item.value;
            DrawSlider(track, item.label, enabled ? TextFormat(item.format, value) : "-",
                       (value - item.min) / (item.max - item.min), ui.activeSlider == WORLD_SLIDER_BASE + (int)i,
                       enabled);
            if (item.info != nullptr && enabled)
            {
                Text(fonts.small, item.info(world), track.x, track.y + 14.0f, COLOR_MUTED);
            }
            break;
        }
        case ItemKind::Toggle:
            DrawToggleRow(ItemToggleRow(rect), item.label, world.*item.flag);
            break;
        }
    }
}

const char *FormatMass(float kg);

void DrawSpawnPanel(const SpawnSettings &settings, int activeSlider)
{
    DrawCard(SpawnCardRect());

    std::vector<const char *> labels;
    for (int i = 0; i < SHAPE_COUNT; i++)
    {
        labels.push_back(ShapeName((Shape)i));
    }
    DrawSegmented(ShapeBarRect(), labels, (int)settings.shape, false);

    labels.clear();
    Color dots[MATERIAL_COUNT];
    for (int i = 0; i < MATERIAL_COUNT; i++)
    {
        labels.push_back(MATERIALS[i].name);
        dots[i] = MATERIALS[i].color;
    }
    DrawSegmented(MaterialBarRect(), labels, settings.material, false, dots);

    std::vector<Param> params = GetParams(settings.shape);
    for (size_t i = 0; i < params.size(); i++)
    {
        const Param &p = params[i];
        float value = settings.*p.field;
        DrawSlider(SliderRect((int)i), p.name, TextFormat("%.2f%s", value, p.unit), (value - p.min) / (p.max - p.min),
                   (int)i == activeSlider, true);
    }

    float mass = SpawnMass(settings);
    DrawSlider(SliderRect(MASS_SLOT), "Mass", mass > 0.0f ? FormatMass(mass) : "by material", settings.massSlider,
               activeSlider == MASS_SLIDER, true);

    const MaterialInfo &material = MATERIALS[settings.material];
    Rectangle info = SliderRect(SLIDER_SLOTS - 1);
    if (info.width >= 130.0f)
    {
        Text(fonts.text, "Density", info.x, info.y - 26.0f, COLOR_MUTED);
        TextRight(fonts.bold, TextFormat("%.0f kg/m³", material.density), info.x + info.width, info.y - 26.0f,
                  COLOR_TEXT);
        Text(fonts.small, TextFormat("e %.2f  ·  μ %.2f", material.bounce, material.friction), info.x, info.y - 4.0f,
             COLOR_MUTED);
    }

    Rectangle spawn = SpawnButtonRect();
    FillRounded(spawn, 10.0f, IsHovered(spawn) ? COLOR_ACCENT_HOVER : COLOR_ACCENT);
    float labelWidth = TextWidth(fonts.bold, "Spawn") + 8.0f + 18.0f;
    float x = spawn.x + (spawn.width - labelWidth) / 2.0f;
    Text(fonts.bold, "Spawn", x, spawn.y + (spawn.height - fonts.bold.size) / 2.0f, WHITE);
    Rectangle key = {x + labelWidth - 18.0f, spawn.y + (spawn.height - 18.0f) / 2.0f, 18.0f, 18.0f};
    FillRounded(key, 4.0f, {255, 255, 255, 50});
    TextCentered(fonts.small, "N", key, WHITE);
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
            DrawSphereEx(position, radius, 24, 32, color);
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
            DrawSphereEx({0.0f, 0.0f, 0.0f}, 1.0f, 24, 32, color);
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

void SetShaderColor(Shader shader, const char *name, Color color)
{
    Vector3 value = {color.r / 255.0f, color.g / 255.0f, color.b / 255.0f};
    SetShaderValue(shader, GetShaderLocation(shader, name), &value, SHADER_UNIFORM_VEC3);
}

Floor LoadFloor()
{
    Floor floor = {};
    floor.shader = LoadShaderFromMemory(FLOOR_VS, FLOOR_FS);
    floor.cameraLoc = GetShaderLocation(floor.shader, "cameraPos");
    floor.gridLoc = GetShaderLocation(floor.shader, "showGrid");

    SetShaderColor(floor.shader, "floorColor", COLOR_FLOOR);
    SetShaderColor(floor.shader, "minorColor", COLOR_GRID_MINOR);
    SetShaderColor(floor.shader, "majorColor", COLOR_GRID_MAJOR);
    SetShaderColor(floor.shader, "fogColor", COLOR_SKY);
    float fogDistance = FOG_DISTANCE;
    SetShaderValue(floor.shader, GetShaderLocation(floor.shader, "fogDistance"), &fogDistance,
                   SHADER_UNIFORM_FLOAT);
    return floor;
}

Lighting LoadLighting()
{
    Lighting lighting = {};
    lighting.shader = LoadShaderFromMemory(LIT_VS, LIT_FS);
    lighting.cameraLoc = GetShaderLocation(lighting.shader, "cameraPos");
    return lighting;
}

void DrawFloor(const Floor &floor, const Camera3D &camera, bool showGrid)
{
    float grid = showGrid ? 1.0f : 0.0f;
    SetShaderValue(floor.shader, floor.cameraLoc, &camera.position, SHADER_UNIFORM_VEC3);
    SetShaderValue(floor.shader, floor.gridLoc, &grid, SHADER_UNIFORM_FLOAT);

    float x = camera.position.x;
    float z = camera.position.z;
    BeginShaderMode(floor.shader);
    rlBegin(RL_QUADS);
    rlColor4ub(255, 255, 255, 255);
    rlVertex3f(x - FLOOR_EXTENT, 0.0f, z - FLOOR_EXTENT);
    rlVertex3f(x - FLOOR_EXTENT, 0.0f, z + FLOOR_EXTENT);
    rlVertex3f(x + FLOOR_EXTENT, 0.0f, z + FLOOR_EXTENT);
    rlVertex3f(x + FLOOR_EXTENT, 0.0f, z - FLOOR_EXTENT);
    rlEnd();
    EndShaderMode();
}

void DrawShadow(const Body &b)
{
    float height = b.position.y - b.size.y / 2.0f;
    float strength = std::clamp(1.0f - height / SHADOW_FADE_HEIGHT, 0.0f, 1.0f);
    if (strength <= 0.0f)
    {
        return;
    }

    float radius = std::max(b.size.x, b.size.z) * 0.6f * (1.0f + height * 0.08f);
    float inner = radius * 0.55f;
    unsigned char center = (unsigned char)(120.0f * strength);
    unsigned char middle = (unsigned char)(80.0f * strength);
    const int segments = 32;

    rlBegin(RL_TRIANGLES);
    for (int i = 0; i < segments; i++)
    {
        float a0 = 2.0f * PI * i / segments;
        float a1 = 2.0f * PI * (i + 1) / segments;
        Vector3 c = {b.position.x, 0.0f, b.position.z};
        Vector3 in0 = {c.x + cosf(a0) * inner, 0.0f, c.z + sinf(a0) * inner};
        Vector3 in1 = {c.x + cosf(a1) * inner, 0.0f, c.z + sinf(a1) * inner};
        Vector3 out0 = {c.x + cosf(a0) * radius, 0.0f, c.z + sinf(a0) * radius};
        Vector3 out1 = {c.x + cosf(a1) * radius, 0.0f, c.z + sinf(a1) * radius};

        rlColor4ub(0, 0, 0, center);
        rlVertex3f(c.x, c.y, c.z);
        rlColor4ub(0, 0, 0, middle);
        rlVertex3f(in1.x, in1.y, in1.z);
        rlVertex3f(in0.x, in0.y, in0.z);

        rlVertex3f(in0.x, in0.y, in0.z);
        rlVertex3f(in1.x, in1.y, in1.z);
        rlColor4ub(0, 0, 0, 0);
        rlVertex3f(out1.x, out1.y, out1.z);

        rlColor4ub(0, 0, 0, middle);
        rlVertex3f(in0.x, in0.y, in0.z);
        rlColor4ub(0, 0, 0, 0);
        rlVertex3f(out1.x, out1.y, out1.z);
        rlVertex3f(out0.x, out0.y, out0.z);
    }
    rlEnd();
}

void DrawGround(const Floor &floor, const Camera3D &camera, const std::vector<Body> &bodies, const UiState &ui)
{
    rlDrawRenderBatchActive();
    rlDisableDepthMask();
    rlDisableBackfaceCulling();

    DrawFloor(floor, camera, ui.showGrid);
    if (ui.showShadows)
    {
        for (const Body &b : bodies)
        {
            DrawShadow(b);
        }
    }

    rlDrawRenderBatchActive();
    rlEnableBackfaceCulling();
    rlEnableDepthMask();
}

void DrawTank(const World &world, const Lighting &lighting)
{
    if (!HasLiquid(world))
    {
        return;
    }
    const Liquid &liquid = LIQUIDS[world.liquid];
    float inner = 2.0f * TANK_HALF;
    float outer = inner + 2.0f * TANK_WALL;
    Color surface = ColorBrightness(liquid.color, 0.35f);

    rlDrawRenderBatchActive();
    rlDisableDepthMask();

    BeginShaderMode(lighting.shader);
    DrawCubeV({0.0f, world.level / 2.0f, 0.0f}, {inner, world.level, inner}, liquid.color);
    for (const Wall &wall : TankWalls())
    {
        DrawCubeV(wall.position, wall.size, {220, 232, 245, 26});
    }
    for (const Droplet &d : world.droplets)
    {
        DrawSphereEx(d.position, 0.035f, 4, 6, ColorAlpha(surface, 0.9f));
    }
    EndShaderMode();

    DrawCubeWiresV({0.0f, world.level / 2.0f, 0.0f}, {inner, world.level, inner}, ColorAlpha(surface, 0.55f));
    DrawCubeWiresV({0.0f, TANK_HEIGHT / 2.0f, 0.0f}, {outer, TANK_HEIGHT, outer}, {255, 255, 255, 45});

    for (const Ripple &r : world.ripples)
    {
        float t = r.age / RIPPLE_LIFE;
        float room = TANK_HALF - std::max(fabsf(r.center.x), fabsf(r.center.y));
        Vector3 center = {r.center.x, world.level + 0.01f, r.center.y};
        for (float scale : {1.0f, 0.6f})
        {
            float radius = std::min((0.3f + r.size * t) * scale, room);
            Color color = ColorAlpha(surface, (1.0f - t) * r.strength * 0.8f);
            DrawCircle3D(center, radius, {1.0f, 0.0f, 0.0f}, 90.0f, color);
        }
    }

    rlDrawRenderBatchActive();
    rlEnableDepthMask();
}

const char *FormatMass(float kg)
{
    if (kg < 1.0f)
    {
        return TextFormat("%.0f g", kg * 1000.0f);
    }
    if (kg < 1000.0f)
    {
        return TextFormat("%.1f kg", kg);
    }
    return TextFormat("%.2f t", kg / 1000.0f);
}

const char *FormatEnergy(float joules)
{
    if (joules < 1000.0f)
    {
        return TextFormat("%.1f J", joules);
    }
    return TextFormat("%.2f kJ", joules / 1000.0f);
}

float PotentialEnergy(const Body &b, const World &world)
{
    return Mass(b) * GRAVITIES[world.gravity].value * BottomHeight(b);
}

float KineticEnergy(const Body &b)
{
    float speed = Vector3Length(b.velocity);
    Vector3 inertia = LocalInertia(b);
    Vector3 w = RotateToLocal(b, b.angularVelocity);
    float spin = inertia.x * w.x * w.x + inertia.y * w.y * w.y + inertia.z * w.z * w.z;
    return 0.5f * Mass(b) * speed * speed + 0.5f * spin;
}

void DrawSelectedPanel(const std::vector<Body> &bodies, int index, const World &world)
{
    const Body &b = bodies[index];
    Rectangle card = SelectedCardRect();
    DrawCard(card);

    float x = card.x + 16.0f;
    float right = card.x + card.width - 16.0f;
    float y = card.y + 14.0f;

    DrawCircleV({x + 5.0f, y + fonts.title.size / 2.0f + 1.0f}, 5.0f, b.color);
    Text(fonts.title, ShapeName(b.shape), x + 18.0f, y, COLOR_TEXT);
    TextRight(fonts.small, MaterialOf(b).name, right, y + 4.0f, COLOR_MUTED);
    y += fonts.title.size + 14.0f;

    auto row = [&](const char *label, const char *value) {
        Text(fonts.text, label, x, y, COLOR_MUTED);
        TextRight(fonts.bold, value, right, y, COLOR_TEXT);
        y += SELECTED_ROW;
    };

    float mass = Mass(b);
    float speed = Vector3Length(b.velocity);
    float fraction = LiquidFraction(b, world);
    if (b.shape == Shape::Sphere)
    {
        row("Radius", TextFormat("%.2f m", b.size.x / 2.0f));
    }
    else
    {
        row("Size", TextFormat("%.2f × %.2f × %.2f m", b.size.x, b.size.y, b.size.z));
    }
    row("Mass", FormatMass(mass));
    row("Speed", TextFormat("%.2f m/s", speed));
    row("Spin", TextFormat("%.2f rad/s", Vector3Length(b.angularVelocity)));
    row("Height", TextFormat("%.2f m", BottomHeight(b)));
    row("Max height", TextFormat("%.2f m", b.maxHeight));
    row("Time", TextFormat("%.2f s", b.age));
    row("Momentum", TextFormat("%.2f kg·m/s", mass * speed));
    row("Kinetic energy", FormatEnergy(KineticEnergy(b)));
    row("Potential energy", FormatEnergy(PotentialEnergy(b, world)));
    if (HasSpring(world, index))
    {
        row("Spring stretch", TextFormat("%.3f m", SpringStretch(bodies, world, index)));
    }
    else
    {
        row("Range", TextFormat("%.2f m", Vector2Distance({b.start.x, b.start.z}, {b.position.x, b.position.z})));
    }
    if (fraction > 0.0f)
    {
        row("Buoyant force", TextFormat("%.1f N", BuoyantForce(b, world)));
        float depth = std::max(world.level - b.position.y, 0.0f);
        row("Pressure", TextFormat("%.2f kPa", LIQUIDS[world.liquid].density * GRAVITIES[world.gravity].value * depth / 1000.0f));
    }
    else
    {
        row("Weight", TextFormat("%.1f N", mass * GRAVITIES[world.gravity].value));
        row("Submerged", "-");
    }
}

void DrawGraphPanel(const std::vector<Sample> &history, int tab)
{
    Rectangle card = GraphCardRect();
    DrawCard(card);
    DrawSegmented(GraphTabsRect(), std::vector<const char *>(GRAPH_NAMES, GRAPH_NAMES + GRAPH_COUNT), tab, true);

    Rectangle plot = {card.x + 14.0f, card.y + 50.0f, card.width - 28.0f, card.height - 64.0f};
    DrawRectangleLinesEx(plot, 1.0f, COLOR_BORDER);
    if (history.size() < 2)
    {
        TextCentered(fonts.small, "Waiting for data", plot, COLOR_MUTED);
        return;
    }

    float start = history.front().time;
    float span = std::max(history.back().time - start, 1.0f);
    auto value = [tab](const Sample &s, int series) {
        if (tab == 0)
        {
            return s.height;
        }
        if (tab == 1)
        {
            return s.speed;
        }
        return series == 0 ? s.kinetic : series == 1 ? s.potential : s.kinetic + s.potential;
    };
    int seriesCount = tab == 2 ? 3 : 1;
    const Color colors[] = {COLOR_ACCENT, {106, 155, 216, 255}, COLOR_TEXT};
    const char *names[] = {"Kinetic", "Potential", "Total"};

    float top = 0.0001f;
    for (const Sample &s : history)
    {
        for (int k = 0; k < seriesCount; k++)
        {
            top = std::max(top, value(s, k));
        }
    }
    top *= 1.1f;

    for (int k = 0; k < seriesCount; k++)
    {
        for (size_t i = 1; i < history.size(); i++)
        {
            Vector2 from = {plot.x + (history[i - 1].time - start) / span * plot.width,
                            plot.y + plot.height - value(history[i - 1], k) / top * plot.height};
            Vector2 to = {plot.x + (history[i].time - start) / span * plot.width,
                          plot.y + plot.height - value(history[i], k) / top * plot.height};
            DrawLineEx(from, to, 1.5f, colors[k]);
        }
    }

    const char *units[] = {"m", "m/s", tab == 2 && top > 1000.0f ? "kJ" : "J"};
    float shown = tab == 2 && top > 1000.0f ? top / 1000.0f : top;
    Text(fonts.small, TextFormat("%.2f %s", shown, units[tab]), plot.x + 4.0f, plot.y + 3.0f, COLOR_MUTED);
    TextRight(fonts.small, TextFormat("%.1f s", span), plot.x + plot.width - 4.0f, plot.y + plot.height - 15.0f,
              COLOR_MUTED);
    if (tab == 2)
    {
        float lx = plot.x + 4.0f;
        for (int k = 0; k < 3; k++)
        {
            DrawCircleV({lx + 3.0f, plot.y + plot.height - 9.0f}, 3.0f, colors[k]);
            Text(fonts.small, names[k], lx + 9.0f, plot.y + plot.height - 15.0f, COLOR_MUTED);
            lx += TextWidth(fonts.small, names[k]) + 18.0f;
        }
    }
}

void DrawBodyShape(const Body &b, Vector3 size, Color color, bool wires)
{
    Vector3 axis;
    float angle;
    QuaternionToAxisAngle(b.orientation, &axis, &angle);
    rlPushMatrix();
    rlTranslatef(b.position.x, b.position.y, b.position.z);
    rlRotatef(angle * RAD2DEG, axis.x, axis.y, axis.z);
    DrawShape(b.shape, {0.0f, 0.0f, 0.0f}, size, color, wires);
    rlPopMatrix();
}

void DrawBodies(const Lighting &lighting, const Camera3D &camera, const std::vector<Body> &bodies)
{
    SetShaderValue(lighting.shader, lighting.cameraLoc, &camera.position, SHADER_UNIFORM_VEC3);
    BeginShaderMode(lighting.shader);
    for (const Body &b : bodies)
    {
        DrawBodyShape(b, b.size, b.color, false);
    }
    EndShaderMode();

    for (const Body &b : bodies)
    {
        if (HasEdges(b.shape))
        {
            DrawBodyShape(b, b.size, ColorBrightness(b.color, -0.45f), true);
        }
    }
}

void DrawRamp(const World &world, const Lighting &lighting)
{
    if (!world.ramp)
    {
        return;
    }
    Ramp ramp = GetRamp(world);
    float w = RAMP_WIDTH / 2.0f;
    Vector3 b0 = {ramp.bottomX, 0.0f, -w};
    Vector3 b1 = {ramp.bottomX, 0.0f, w};
    Vector3 t0 = {ramp.topX, 0.0f, -w};
    Vector3 t1 = {ramp.topX, 0.0f, w};
    Vector3 h0 = {ramp.topX, ramp.height, -w};
    Vector3 h1 = {ramp.topX, ramp.height, w};
    const Color color = {122, 128, 140, 255};

    rlDisableBackfaceCulling();
    BeginShaderMode(lighting.shader);
    rlBegin(RL_TRIANGLES);
    rlColor4ub(color.r, color.g, color.b, color.a);
    for (const Vector3 &v : {b0, b1, h1, b0, h1, h0, t0, t1, h1, t0, h1, h0, b0, t0, h0, b1, t1, h1})
    {
        rlVertex3f(v.x, v.y, v.z);
    }
    rlEnd();
    EndShaderMode();
    rlEnableBackfaceCulling();

    Color edge = ColorBrightness(color, -0.4f);
    for (auto [from, to] : {std::pair{b0, b1}, {h0, h1}, {b0, h0}, {b1, h1}, {t0, h0}, {t1, h1}, {t0, t1}})
    {
        DrawLine3D(from, to, edge);
    }
}

void DrawArrow(Vector3 from, Vector3 to, float radius, Color color)
{
    Vector3 delta = Vector3Subtract(to, from);
    float length = Vector3Length(delta);
    if (length < 0.05f)
    {
        return;
    }
    float head = std::min(radius * 4.0f, length * 0.5f);
    Vector3 neck = Vector3Subtract(to, Vector3Scale(delta, head / length));
    DrawCylinderEx(from, neck, radius, radius, 8, color);
    DrawCylinderEx(neck, to, radius * 2.8f, 0.0f, 8, color);
}

void DrawSpring(Vector3 from, Vector3 to, Color color)
{
    Vector3 axis = Vector3Subtract(to, from);
    float length = Vector3Length(axis);
    if (length < 0.001f)
    {
        return;
    }
    Vector3 dir = Vector3Scale(axis, 1.0f / length);
    Vector3 side = fabsf(dir.y) < 0.9f ? Vector3{0.0f, 1.0f, 0.0f} : Vector3{1.0f, 0.0f, 0.0f};
    Vector3 u = Vector3Normalize(Vector3CrossProduct(dir, side));
    Vector3 v = Vector3CrossProduct(dir, u);
    const int turns = 14;
    const int steps = turns * 12;
    const float radius = 0.12f;
    Vector3 previous = from;
    for (int i = 1; i <= steps; i++)
    {
        float t = (float)i / steps;
        float angle = 2.0f * PI * turns * t;
        float coil = (i < 6 || i > steps - 6) ? 0.0f : radius;
        Vector3 point = Vector3Add(Vector3Add(from, Vector3Scale(axis, t)),
                                   Vector3Add(Vector3Scale(u, cosf(angle) * coil), Vector3Scale(v, sinf(angle) * coil)));
        DrawLine3D(previous, point, color);
        previous = point;
    }
}

void DrawJoints(const std::vector<Body> &bodies, const World &world)
{
    const Color rope = {214, 206, 190, 255};
    const Color metal = {170, 176, 186, 255};
    for (const Joint &joint : world.joints)
    {
        Vector3 a = bodies[joint.a].position;
        if (joint.type == JointType::Pulley)
        {
            Vector3 b = bodies[joint.b].position;
            Vector3 across = Vector3Subtract(joint.pulley, joint.anchor);
            across.y = 0.0f;
            across = Vector3Length(across) > 0.001f ? Vector3Normalize(across) : Vector3{1.0f, 0.0f, 0.0f};
            Vector3 axle = Vector3Normalize(Vector3CrossProduct(across, {0.0f, 1.0f, 0.0f}));
            for (Vector3 wheel : {joint.anchor, joint.pulley})
            {
                DrawCylinderEx(Vector3Subtract(wheel, Vector3Scale(axle, 0.05f)), Vector3Add(wheel, Vector3Scale(axle, 0.05f)),
                               0.2f, 0.2f, 20, metal);
                DrawLine3D(wheel, {wheel.x, wheel.y + 0.6f, wheel.z}, metal);
            }
            Vector3 lift = {0.0f, 0.2f, 0.0f};
            DrawLine3D(Vector3Add(joint.anchor, lift), Vector3Add(joint.pulley, lift), rope);
            DrawLine3D(a, joint.anchor, rope);
            DrawLine3D(b, joint.pulley, rope);
            continue;
        }
        Vector3 end = JointEnd(bodies, joint);
        if (joint.b == NO_SELECTION)
        {
            DrawCubeV({end.x, end.y + 0.03f, end.z}, {0.5f, 0.06f, 0.5f}, {90, 94, 104, 255});
        }
        switch (joint.type)
        {
        case JointType::Spring: DrawSpring(end, a, metal); break;
        case JointType::Rod: DrawCylinderEx(end, a, 0.03f, 0.03f, 8, metal); break;
        default: DrawLine3D(end, a, rope); break;
        }
    }
}

void DrawOverlays(const std::vector<Body> &bodies, const World &world)
{
    for (const Body &b : bodies)
    {
        Color color = ColorAlpha(ColorBrightness(b.color, 0.25f), 0.85f);
        for (size_t i = 1; i < b.trail.size(); i++)
        {
            DrawLine3D(b.trail[i - 1], b.trail[i], color);
        }
        if (world.vectors)
        {
            DrawArrow(b.position, Vector3Add(b.position, Vector3Scale(b.velocity, VECTOR_SCALE)), 0.03f,
                      {255, 214, 102, 255});
        }
        if (Vector3Length(b.pushDirection) > 0.0f)
        {
            Vector3 force = Vector3Scale(b.pushDirection, std::max(world.pushForce * FORCE_SCALE, 0.4f));
            DrawArrow(Vector3Subtract(b.position, force), b.position, 0.04f, {235, 96, 96, 255});
        }
    }
    DrawJoints(bodies, world);

    if (HasWind(world))
    {
        Vector3 dir = Vector3Normalize(WindVector(world));
        float length = 0.6f + 2.4f * world.windSpeed / MAX_WIND;
        Vector3 center = {0.0f, TANK_HEIGHT + 2.0f, 0.0f};
        Vector3 from = Vector3Subtract(center, Vector3Scale(dir, length / 2.0f));
        DrawArrow(from, Vector3Add(from, Vector3Scale(dir, length)), 0.05f, {200, 220, 240, 200});
    }
}

void DrawRain(const World &world)
{
    if (world.raindrops.empty())
    {
        return;
    }
    rlDrawRenderBatchActive();
    rlDisableDepthMask();
    for (const Droplet &d : world.raindrops)
    {
        DrawLine3D(d.position, Vector3Subtract(d.position, Vector3Scale(d.velocity, 0.035f)), {170, 192, 220, 120});
    }
    rlDrawRenderBatchActive();
    rlEnableDepthMask();
}

void DrawScene(const Camera3D &camera, const Floor &floor, const Lighting &lighting, const std::vector<Body> &bodies,
               const World &world, int selected, bool paused, const SpawnSettings &settings, const UiState &ui,
               const Stopwatch &stopwatch, const std::vector<Sample> &history)
{
    BeginDrawing();
    ClearBackground(COLOR_SKY);

    BeginMode3D(camera);
    DrawGround(floor, camera, bodies, ui);
    DrawRamp(world, lighting);
    DrawBodies(lighting, camera, bodies);
    DrawOverlays(bodies, world);
    if (selected != NO_SELECTION)
    {
        const Body &b = bodies[selected];
        DrawBodyShape(b, Vector3Scale(b.size, 1.08f), COLOR_ACCENT, true);
        float bottom = BottomHeight(b);
        Vector3 floorPoint = {b.position.x, 0.01f, b.position.z};
        DrawLine3D({b.position.x, bottom, b.position.z}, floorPoint, COLOR_ACCENT);
        DrawCircle3D(floorPoint, std::max(b.size.x, b.size.z) / 2.0f, {1.0f, 0.0f, 0.0f}, 90.0f, COLOR_ACCENT);
    }
    if (ui.linkFrom != NO_SELECTION)
    {
        DrawBodyShape(bodies[ui.linkFrom], Vector3Scale(bodies[ui.linkFrom].size, 1.12f), {120, 200, 255, 255}, true);
    }
    DrawTank(world, lighting);
    DrawRain(world);
    EndMode3D();

    if (selected != NO_SELECTION)
    {
        DrawSelectedPanel(bodies, selected, world);
        DrawGraphPanel(history, ui.graphTab);
    }
    DrawSpawnPanel(settings, ui.activeSlider);
    float pillY = MARGIN;
    if (paused)
    {
        DrawPill("Paused", "Space to resume", pillY);
        pillY += ICON_BUTTON_SIZE + 8.0f;
    }
    if (ui.linkFrom != NO_SELECTION)
    {
        DrawPill(TextFormat("Linking with a %s", JOINT_NAMES[world.jointType]), "Click another body · J to cancel",
                 pillY);
    }
    if (!ui.settingsOpen)
    {
        DrawWorldPanel(world, ui);
    }
    DrawTopBar(ui, (int)bodies.size(), stopwatch);
    if (ui.settingsOpen)
    {
        DrawSettingsPanel(ui);
    }

    EndDrawing();
}

void RecordHistory(std::vector<Sample> &history, int &historyBody, float &clock, const std::vector<Body> &bodies,
                   int selected, const World &world, float dt)
{
    if (selected != historyBody)
    {
        history.clear();
        historyBody = selected;
        clock = 0.0f;
    }
    if (selected == NO_SELECTION)
    {
        return;
    }
    clock += dt;
    float last = history.empty() ? -1.0f : history.back().time;
    if (!history.empty() && clock - last < SAMPLE_INTERVAL)
    {
        return;
    }
    const Body &b = bodies[selected];
    history.push_back({clock, BottomHeight(b), Vector3Length(b.velocity), KineticEnergy(b), PotentialEnergy(b, world)});
    while (!history.empty() && clock - history.front().time > HISTORY_SECONDS)
    {
        history.erase(history.begin());
    }
}

int main()
{
    SetConfigFlags(FLAG_MSAA_4X_HINT | FLAG_WINDOW_RESIZABLE | FLAG_VSYNC_HINT);
    InitWindow(1440, 900, "P in 3D");
    SetTargetFPS(60);
    SetExitKey(KEY_NULL);

    fonts = LoadFonts();
    Floor floor = LoadFloor();
    Lighting lighting = LoadLighting();

    FreeCamera cam = CreateFreeCamera();
    int cursor = MOUSE_CURSOR_DEFAULT;

    std::vector<Body> bodies = CreateBodies();
    int selected = NO_SELECTION;
    Drag drag = {};
    bool paused = false;
    float qHoldTime = 0.0f;
    Stopwatch stopwatch = {};
    std::vector<Sample> history;
    int historyBody = NO_SELECTION;
    float historyClock = 0.0f;

    SpawnSettings settings = {Shape::Sphere, 0.5f, 1.0f, 1.0f, 1.0f, FindMaterial("Rubber"), 0.0f};
    UiState ui = {0, 0, NO_SELECTION, false, true, true, false, NO_SELECTION};
    World world = {};
    world.timeScale = NORMAL_TIME;
    world.liquid = 1;
    world.level = 1.8f;
    world.friction = 0.2f;
    world.ramp = true;
    world.rampAngle = 25.0f;
    world.launchSpeed = 15.0f;
    world.pushForce = 50.0f;
    world.dropHeight = 6.0f;
    world.jointLength = 2.0f;
    world.stiffness = 100.0f;

    while (!WindowShouldClose())
    {
        float dt = GetFrameTime();
        world.viewPitch = cam.pitch * RAD2DEG;

        if (IsKeyPressed(KEY_SPACE))
        {
            paused = !paused;
        }
        if (IsKeyPressed(KEY_R))
        {
            ResetBodies(bodies);
        }
        if (IsKeyPressed(KEY_T))
        {
            if (IsKeyDown(KEY_LEFT_SHIFT))
            {
                stopwatch = {};
            }
            else
            {
                stopwatch.running = !stopwatch.running;
            }
        }

        bool overUi = IsMouseOverUi(ui, selected != NO_SELECTION);
        bool spawnClicked = UpdateUi(ui, settings, world, selected != NO_SELECTION);
        if (IsKeyPressed(KEY_N) || spawnClicked)
        {
            bodies.push_back(CreateBody(settings, SpawnPoint(cam, world.dropHeight)));
        }
        if (IsKeyPressed(KEY_F))
        {
            bodies.push_back(ThrowBody(settings, cam, world.launchSpeed));
        }

        if (selected != NO_SELECTION)
        {
            Body &b = bodies[selected];
            if (IsKeyPressed(KEY_V))
            {
                b.velocity = Vector3Scale(HorizontalForward(cam), world.launchSpeed);
                b.start = b.position;
                ResetStats(b);
            }
            if (IsKeyPressed(KEY_P))
            {
                bool pushing = Vector3Length(b.pushDirection) > 0.0f;
                b.pushDirection = pushing ? Vector3Zero() : HorizontalForward(cam);
                if (!pushing)
                {
                    b.start = b.position;
                    ResetStats(b);
                }
            }
            if (IsKeyPressed(KEY_H))
            {
                HangBody(bodies, world, selected);
            }
            if (IsKeyPressed(KEY_K))
            {
                CutJoints(world, selected);
            }
        }
        if (IsKeyPressed(KEY_J))
        {
            ui.linkFrom = ui.linkFrom == NO_SELECTION ? selected : NO_SELECTION;
        }

        if (IsKeyPressed(KEY_Q) && !bodies.empty())
        {
            RemoveBody(bodies, world, (int)bodies.size() - 1);
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
                world.joints.clear();
                selected = NO_SELECTION;
            }
        }
        else
        {
            qHoldTime = 0.0f;
        }
        if (ui.linkFrom >= (int)bodies.size())
        {
            ui.linkFrom = NO_SELECTION;
        }

        UpdateFreeCamera(cam, dt, drag.active || (overUi && !cam.looking));

        if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && !cam.looking && !overUi)
        {
            Vector3 grabPoint;
            int picked = PickBody(cam.camera, GetMousePosition(), bodies, grabPoint);
            if (ui.linkFrom != NO_SELECTION)
            {
                if (picked != NO_SELECTION && picked != ui.linkFrom)
                {
                    LinkBodies(bodies, world, ui.linkFrom, picked);
                }
                ui.linkFrom = NO_SELECTION;
            }
            else
            {
                selected = picked;
                if (selected != NO_SELECTION)
                {
                    drag = StartDrag(bodies[selected], grabPoint);
                }
            }
        }
        if (IsKeyPressed(KEY_DELETE) && selected != NO_SELECTION)
        {
            RemoveBody(bodies, world, selected);
            selected = NO_SELECTION;
            ui.linkFrom = NO_SELECTION;
        }

        bool wasDragging = drag.active;
        if (selected == NO_SELECTION || !IsMouseButtonDown(MOUSE_BUTTON_LEFT))
        {
            drag.active = false;
        }
        if (wasDragging && !drag.active && selected != NO_SELECTION)
        {
            bodies[selected].start = bodies[selected].position;
            ResetStats(bodies[selected]);
        }
        if (drag.active)
        {
            UpdateDrag(drag, bodies[selected], cam.camera, PointerPosition(cam), dt);
            HoldDragged(drag, bodies[selected], paused);
        }

        if (!paused)
        {
            float simDt = UpdatePhysics(bodies, world, cam.camera.position, dt);
            RecordTrails(bodies, world.trails);
            RecordHistory(history, historyBody, historyClock, bodies, selected, world, simDt);
            if (stopwatch.running)
            {
                stopwatch.time += simDt;
            }
        }
        if (drag.active)
        {
            HoldDragged(drag, bodies[selected], paused);
        }
        UpdateCursor(cam, bodies, drag, ui, selected != NO_SELECTION, cursor);
        DrawScene(cam.camera, floor, lighting, bodies, world, selected, paused, settings, ui, stopwatch, history);
    }

    UnloadShader(floor.shader);
    UnloadShader(lighting.shader);
    UnloadFonts(fonts);
    CloseWindow();
    return 0;
}
