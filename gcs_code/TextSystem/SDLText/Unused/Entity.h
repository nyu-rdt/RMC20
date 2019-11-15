#pragma once
#include "ShaderProgram.h"

class Entity
{
public:
    Entity();
    Entity(
        GLuint textureID,
        glm::mat4* projectionMatrix,
        glm::mat4* viewMatrix,
        glm::vec3 position = glm::vec3(0, 0, 0),
        glm::vec3 scale = glm::vec3(1, 1, 1),
        glm::vec3 velocity = glm::vec3(0, 0, 0),
        glm::vec3 acceleration = glm::vec3(0, 0, 0),
        glm::vec4 color = glm::vec4(1, 1, 1, 1)
    );
    Entity(const Entity& rhs);
    ~Entity();

    Entity& operator=(const Entity& rhs);

    void Draw();

    glm::mat4 RecalculateMM();

    void resize(int tris);
    void calculateUVs(int x, int y, int index, int quadIndex = 0);

    glm::vec3 position;
    glm::vec3 scale;
    glm::vec3 velocity;
    glm::vec3 acceleration;
    glm::vec4 color;

    GLuint textureID;
    glm::mat4* projectionMatrix;
    glm::mat4* viewMatrix;

    int vertCount;
    float* verts;
    float* uvs;

    int textureIndex;
};
