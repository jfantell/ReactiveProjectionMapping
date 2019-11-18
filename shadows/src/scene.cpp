#include "scene.h"

Floor::Floor(float vertexScaleFactor) {

  // Floor has support for it's own model matrix. give it a boilerplate one for now.
  // This scale has NOTHING to do with vertexScaleFactor. It was simply put here because we'll need it in the future.
  glm::vec3 scale = glm::vec3(1.5f, 1.5f, 1.5f);
  _model = glm::scale(glm::mat4(1.0f), scale);

  for (int i = 0; i < 12; i++) {
    if (i % 2 != 0) continue; // skip over the normals.

    // Multiply the coplanar vertices by the vertexScaleFactor.
    vertices_normals[3*i] = vertices_normals[3*i] * vertexScaleFactor;
    vertices_normals[(3*i) + 2] = vertices_normals[(3*i) + 2] * vertexScaleFactor;
  }

  // Create a manual vbo for the floor.
  _vboId = 0;
  glGenBuffers(1, &_vboId);
  glBindBuffer(GL_ARRAY_BUFFER, _vboId);
  glBufferData(GL_ARRAY_BUFFER, 3*2*6*sizeof(float), vertices_normals, GL_STATIC_DRAW);

  // Create the subsequent floor vertex array. we need to draw mesh and floor separately.
  _vaoId = 0;
  glGenVertexArrays(1, &_vaoId);
  glBindVertexArray(_vaoId);

  // Tell Opengl how to interpret our floor data. Same as mesh loading.
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0,  // Attribute ZERO (vertices)
                        3, // 3 floats per vertex XYZ
                        GL_FLOAT, // type is float
                        GL_FALSE, // they are not normalized
                        6 * sizeof(float), // stride is size of one whole vertex.
                        nullptr // No offset into the buffer
  );

  glEnableVertexAttribArray(1);
  glVertexAttribPointer(1,  // Attribute ONE (normals)
                        3, // 3 floats per vertex normal
                        GL_FLOAT, // type is float
                        GL_FALSE, // they are not normalized
                        6 * sizeof(float), // stride is size of one whole vertex.
                        (GLvoid*) (3 * sizeof(float)) // offset into the buffer by 3 floats.
  );

}

Floor::~Floor() { }

void Floor::bindVao() {
  glBindVertexArray(_vaoId);
}

void Floor::draw() {
  bindVao();
  glDrawArrays(GL_TRIANGLES, 0, 6);

}