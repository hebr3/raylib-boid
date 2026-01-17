#include "raylib.h"
#include "raymath.h"
#include <stdbool.h>
#include <math.h>
#include <string.h>

// ============================================================================
// GAMESTATE - Data
// ============================================================================

#define SCREEN_WIDTH 2560
#define SCREEN_HEIGHT 1440

// ============================================================================
// COMPONENTS - Pure data
// ============================================================================

typedef struct {
    bool active;
} Entity;

// ============================================================================
// SPATIAL GRID - For fast neighbor lookups
// ============================================================================

#define CELL_SIZE 50
#define GRID_WIDTH (SCREEN_WIDTH / CELL_SIZE + 1)
#define GRID_HEIGHT (SCREEN_HEIGHT / CELL_SIZE + 1)
#define MAX_ENTITIES_PER_CELL 100

typedef struct {
    int entities[MAX_ENTITIES_PER_CELL];
    int count;
} GridCell;

typedef struct {
    GridCell cells[GRID_WIDTH][GRID_HEIGHT];
} SpatialGrid;

SpatialGrid spatialGrid;

void ClearSpatialGrid(SpatialGrid *grid)
{
    memset(grid, 0, sizeof(SpatialGrid));
}

void AddToSpatialGrid(SpatialGrid *grid, int entityId, Vector2 pos)
{
    int gridX = (int)(pos.x / CELL_SIZE);
    int gridY = (int)(pos.y / CELL_SIZE);
    
    // Clamp to grid bounds
    if (gridX < 0) gridX = 0;
    if (gridX >= GRID_WIDTH) gridX = GRID_WIDTH - 1;
    if (gridY < 0) gridY = 0;
    if (gridY >= GRID_HEIGHT) gridY = GRID_HEIGHT - 1;
    
    GridCell *cell = &grid->cells[gridX][gridY];
    if (cell->count < MAX_ENTITIES_PER_CELL)
    {
        cell->entities[cell->count++] = entityId;
    }
}

// Query nearby entities within a radius
void QuerySpatialGrid(SpatialGrid *grid, Vector2 pos, float radius, int *outEntities, int *outCount, int maxResults)
{
    *outCount = 0;
    
    // Calculate grid cell range to check
    int minX = (int)((pos.x - radius) / CELL_SIZE);
    int maxX = (int)((pos.x + radius) / CELL_SIZE);
    int minY = (int)((pos.y - radius) / CELL_SIZE);
    int maxY = (int)((pos.y + radius) / CELL_SIZE);
    
    // Clamp to grid bounds
    if (minX < 0) minX = 0;
    if (maxX >= GRID_WIDTH) maxX = GRID_WIDTH - 1;
    if (minY < 0) minY = 0;
    if (maxY >= GRID_HEIGHT) maxY = GRID_HEIGHT - 1;
    
    // Collect entities from nearby cells
    for (int y = minY; y <= maxY; y++)
    {
        for (int x = minX; x <= maxX; x++)
        {
            GridCell *cell = &grid->cells[x][y];
            for (int i = 0; i < cell->count; i++)
            {
                if (*outCount < maxResults)
                {
                    outEntities[(*outCount)++] = cell->entities[i];
                }
            }
        }
    }
}

// ============================================================================
// ECS DATA - Struct of Arrays
// ============================================================================

#define MAX_ENTITIES 8000

Entity entities[MAX_ENTITIES];
Vector2 positions[MAX_ENTITIES];
Vector2 velocities[MAX_ENTITIES];
Vector2 accelerations[MAX_ENTITIES];
Color colors[MAX_ENTITIES];

int entityCount = 0;

// ============================================================================
// BOID PARAMETERS
// ============================================================================

typedef struct {
    float perceptionRadius;
    float separationRadius;
    float maxSpeed;
    float maxForce;
    
    float separationWeight;
    float alignmentWeight;
    float cohesionWeight;
} BoidParams;

BoidParams boidParams = {
    .perceptionRadius = 20.0f,
    .separationRadius = 10.0f,
    .maxSpeed = 5.0f,
    .maxForce = 0.7f,
    .separationWeight = 3.0f,
    .alignmentWeight = 1.0f,
    .cohesionWeight = 0.5f,
};

// ============================================================================
// UTILITY FUNCTIONS
// ============================================================================

Color GetRandomColor()
{
    Color blue = (Color){100, 143, 255, 255};
    Color purple = (Color){120, 94, 240, 255};
    Color pink = (Color){220, 38, 127, 255};
    Color orange = (Color){254, 97, 0, 255};
    Color yellow = (Color){255, 176, 0, 255};
    Color colorPalette[5] = { blue, purple, pink, orange, yellow };
    return colorPalette[GetRandomValue(0, 4)];
}

Vector2 Vector2Limit(Vector2 v, float max)
{
    float magSq = v.x * v.x + v.y * v.y;
    if (magSq > max * max)
    {
        float mag = sqrtf(magSq);
        return (Vector2){ (v.x / mag) * max, (v.y / mag) * max };
    }
    return v;
}

Vector2 Vector2SetMag(Vector2 v, float mag)
{
    float currentMag = sqrtf(v.x * v.x + v.y * v.y);
    if (currentMag > 0)
    {
        return (Vector2){ (v.x / currentMag) * mag, (v.y / currentMag) * mag };
    }
    return v;
}

// ============================================================================
// ENTITY MANAGEMENT
// ============================================================================

int CreateEntity(Vector2 position, Vector2 velocity, Color color)
{
    if (entityCount >= MAX_ENTITIES) return -1;
    
    int id = entityCount;
    entities[id].active = true;
    positions[id] = position;
    velocities[id] = velocity;
    accelerations[id] = (Vector2){ 0, 0 };
    colors[id] = color;
    
    entityCount++;
    return id;
}

void CreateRandomEntity(int width, int height, int border)
{
    Vector2 pos = {
        GetRandomValue(border, width - border) * 1.0f,
        GetRandomValue(border, height - border) * 1.0f,
    };
    
    Vector2 vel = {
        GetRandomValue(-4, 4) * 0.25f,
        GetRandomValue(-4, 4) * 0.25f,
    };
    
    Color color = GetRandomColor();
    
    CreateEntity(pos, vel, color);
}

// ============================================================================
// SPATIAL GRID UPDATE SYSTEM
// ============================================================================

void SpatialGridUpdateSystem(SpatialGrid *grid, Vector2 *pos, Entity *ent, int count)
{
    ClearSpatialGrid(grid);
    
    for (int i = 0; i < count; i++)
    {
        if (!ent[i].active) continue;
        AddToSpatialGrid(grid, i, pos[i]);
    }
}

// ============================================================================
// BOID SYSTEMS - Flocking behavior (with spatial partitioning)
// ============================================================================

void BoidSeparationSystem(SpatialGrid *grid, Vector2 *pos, Vector2 *vel, Vector2 *acc, Entity *ent, int count, BoidParams params)
{
    int nearbyEntities[MAX_ENTITIES_PER_CELL * 9]; // Max entities in 3x3 grid
    int nearbyCount;
    
    for (int i = 0; i < count; i++)
    {
        if (!ent[i].active) continue;
        
        Vector2 steering = { 0, 0 };
        int total = 0;
        
        // Query spatial grid for nearby entities
        QuerySpatialGrid(grid, pos[i], params.separationRadius, nearbyEntities, &nearbyCount, MAX_ENTITIES_PER_CELL * 9);
        
        // Check only nearby boids
        for (int k = 0; k < nearbyCount; k++)
        {
            int j = nearbyEntities[k];
            if (i == j || !ent[j].active) continue;
            
            float dist = Vector2Distance(pos[i], pos[j]);
            
            if (dist < params.separationRadius && dist > 0)
            {
                Vector2 diff = Vector2Subtract(pos[i], pos[j]);
                diff.x /= dist;
                diff.y /= dist;
                
                steering = Vector2Add(steering, diff);
                total++;
            }
        }
        
        if (total > 0)
        {
            steering.x /= total;
            steering.y /= total;
            
            steering = Vector2SetMag(steering, params.maxSpeed);
            steering = Vector2Subtract(steering, vel[i]);
            steering = Vector2Limit(steering, params.maxForce);
            
            steering.x *= params.separationWeight;
            steering.y *= params.separationWeight;
            
            acc[i] = Vector2Add(acc[i], steering);
        }
    }
}

void BoidAlignmentSystem(SpatialGrid *grid, Vector2 *pos, Vector2 *vel, Vector2 *acc, Entity *ent, int count, BoidParams params)
{
    int nearbyEntities[MAX_ENTITIES_PER_CELL * 9];
    int nearbyCount;
    
    for (int i = 0; i < count; i++)
    {
        if (!ent[i].active) continue;
        
        Vector2 steering = { 0, 0 };
        int total = 0;
        
        QuerySpatialGrid(grid, pos[i], params.perceptionRadius, nearbyEntities, &nearbyCount, MAX_ENTITIES_PER_CELL * 9);
        
        for (int k = 0; k < nearbyCount; k++)
        {
            int j = nearbyEntities[k];
            if (i == j || !ent[j].active) continue;
            
            float dist = Vector2Distance(pos[i], pos[j]);
            
            if (dist < params.perceptionRadius)
            {
                steering = Vector2Add(steering, vel[j]);
                total++;
            }
        }
        
        if (total > 0)
        {
            steering.x /= total;
            steering.y /= total;
            
            steering = Vector2SetMag(steering, params.maxSpeed);
            steering = Vector2Subtract(steering, vel[i]);
            steering = Vector2Limit(steering, params.maxForce);
            
            steering.x *= params.alignmentWeight;
            steering.y *= params.alignmentWeight;
            
            acc[i] = Vector2Add(acc[i], steering);
        }
    }
}

void BoidCohesionSystem(SpatialGrid *grid, Vector2 *pos, Vector2 *vel, Vector2 *acc, Entity *ent, int count, BoidParams params)
{
    int nearbyEntities[MAX_ENTITIES_PER_CELL * 9];
    int nearbyCount;
    
    for (int i = 0; i < count; i++)
    {
        if (!ent[i].active) continue;
        
        Vector2 steering = { 0, 0 };
        int total = 0;
        
        QuerySpatialGrid(grid, pos[i], params.perceptionRadius, nearbyEntities, &nearbyCount, MAX_ENTITIES_PER_CELL * 9);
        
        for (int k = 0; k < nearbyCount; k++)
        {
            int j = nearbyEntities[k];
            if (i == j || !ent[j].active) continue;
            
            float dist = Vector2Distance(pos[i], pos[j]);
            
            if (dist < params.perceptionRadius)
            {
                steering = Vector2Add(steering, pos[j]);
                total++;
            }
        }
        
        if (total > 0)
        {
            steering.x /= total;
            steering.y /= total;
            
            steering = Vector2Subtract(steering, pos[i]);
            steering = Vector2SetMag(steering, params.maxSpeed);
            steering = Vector2Subtract(steering, vel[i]);
            steering = Vector2Limit(steering, params.maxForce);
            
            steering.x *= params.cohesionWeight;
            steering.y *= params.cohesionWeight;
            
            acc[i] = Vector2Add(acc[i], steering);
        }
    }
}

// ============================================================================
// CORE SYSTEMS
// ============================================================================

void AccelerationResetSystem(Vector2 *acc, Entity *ent, int count)
{
    for (int i = 0; i < count; i++)
    {
        if (!ent[i].active) continue;
        acc[i] = (Vector2){ 0, 0 };
    }
}

void PhysicsSystem(Vector2 *pos, Vector2 *vel, Vector2 *acc, Entity *ent, int count, float maxSpeed)
{
    for (int i = 0; i < count; i++)
    {
        if (!ent[i].active) continue;
        
        vel[i] = Vector2Add(vel[i], acc[i]);
        vel[i] = Vector2Limit(vel[i], maxSpeed);
        pos[i] = Vector2Add(pos[i], vel[i]);
    }
}

void WrapAroundSystem(Vector2 *pos, Entity *ent, int count, int width, int height)
{
    for (int i = 0; i < count; i++)
    {
        if (!ent[i].active) continue;
        
        if (pos[i].x < 0) pos[i].x = width;
        if (pos[i].x > width) pos[i].x = 0;
        if (pos[i].y < 0) pos[i].y = height;
        if (pos[i].y > height) pos[i].y = 0;
    }
}

void RenderSystem(Texture2D tex, Vector2 *pos, Vector2 *vel, Color *col, Entity *ent, int count)
{
    for (int i = 0; i < count; i++)
    {
        if (!ent[i].active) continue;
        
        float rotation = atan2f(vel[i].y, vel[i].x) * RAD2DEG + 90.0f;
        
        Rectangle source = { 0, 0, 8, 8 };
        Rectangle dest = { pos[i].x, pos[i].y, 8, 8 };
        Vector2 origin = { 4, 4 };
        
        DrawTexturePro(tex, source, dest, origin, rotation, col[i]);
    }
}

// ============================================================================
// MAIN
// ============================================================================

int main(void)
{
    InitWindow(SCREEN_WIDTH, SCREEN_HEIGHT, "Boid Simulation - ECS + Spatial Partitioning");
    SetTargetFPS(60);
    
    for (int i = 0; i < MAX_ENTITIES; i++)
    {
        CreateRandomEntity(SCREEN_WIDTH, SCREEN_HEIGHT, 20);
    }
    
    Texture2D tex = LoadTexture("resources/boid.png");
    
    Color customBlack = (Color){ 31, 31, 31 };

    while (!WindowShouldClose())
    {
        if (IsKeyDown(KEY_ONE)) boidParams.separationWeight += 0.01f;
        if (IsKeyDown(KEY_TWO)) boidParams.separationWeight -= 0.01f;
        if (IsKeyDown(KEY_THREE)) boidParams.alignmentWeight += 0.01f;
        if (IsKeyDown(KEY_FOUR)) boidParams.alignmentWeight -= 0.01f;
        if (IsKeyDown(KEY_FIVE)) boidParams.cohesionWeight += 0.01f;
        if (IsKeyDown(KEY_SIX)) boidParams.cohesionWeight -= 0.01f;
        
        // Build spatial grid for fast neighbor queries
        SpatialGridUpdateSystem(&spatialGrid, positions, entities, entityCount);
        
        AccelerationResetSystem(accelerations, entities, entityCount);
        
        // Boid behaviors now use spatial grid
        BoidSeparationSystem(&spatialGrid, positions, velocities, accelerations, entities, entityCount, boidParams);
        BoidAlignmentSystem(&spatialGrid, positions, velocities, accelerations, entities, entityCount, boidParams);
        BoidCohesionSystem(&spatialGrid, positions, velocities, accelerations, entities, entityCount, boidParams);
        
        PhysicsSystem(positions, velocities, accelerations, entities, entityCount, boidParams.maxSpeed);
        WrapAroundSystem(positions, entities, entityCount, SCREEN_WIDTH, SCREEN_HEIGHT);
        
        BeginDrawing();
        {
            ClearBackground(customBlack);
            
            RenderSystem(tex, positions, velocities, colors, entities, entityCount);
            
            DrawRectangle(0, 0, 400, 140, Fade(RAYWHITE, 0.8f));
            DrawFPS(10, 10);
            DrawText(TextFormat("Separation: %.2f (1/2)", boidParams.separationWeight), 10, 30, 20, BLACK);
            DrawText(TextFormat("Alignment: %.2f (3/4)", boidParams.alignmentWeight), 10, 50, 20, BLACK);
            DrawText(TextFormat("Cohesion: %.2f (5/6)", boidParams.cohesionWeight), 10, 70, 20, BLACK);
            DrawText(TextFormat("Boids: %d", entityCount), 10, 90, 20, BLACK);
            DrawText(TextFormat("Grid: %dx%d cells", GRID_WIDTH, GRID_HEIGHT), 10, 110, 20, BLACK);
        }
        EndDrawing();
    }

    UnloadTexture(tex);
    CloseWindow();

    return 0;
}