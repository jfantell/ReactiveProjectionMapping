#include "scene.h"
#include <iostream>

Floor::Floor(float vertexScaleFactor) {

  // Floor has support for it's own model matrix. give it a boilerplate one for now.
  // This scale has NOTHING to do with vertexScaleFactor. It was simply put here because we'll need it in the future.
  glm::vec3 scale = glm::vec3(vertexScaleFactor, vertexScaleFactor, vertexScaleFactor);
  _model = glm::scale(glm::mat4(1.0f), scale);


  /* Experimental texture mapping stuff */
  int width, height, nrChannels;
  unsigned char *data = stbi_load("../res/treetex.jpg", &width, &height, &nrChannels, 0);
  glGenTextures(1, &_texId);
  glBindTexture(GL_TEXTURE_2D, _texId);
  // set the texture wrapping/filtering options (on the currently bound texture object)
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, data);
  glGenerateMipmap(GL_TEXTURE_2D);
  if (data)
  {
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, data);
    glGenerateMipmap(GL_TEXTURE_2D);
  }
  else
  {
    std::cerr << "Failed to load texture" << std::endl;
    exit(-1);
  }
  stbi_image_free(data);





  // Create a manual vbo for the floor.
  _vboId = 0;
  glGenBuffers(1, &_vboId);
  glBindBuffer(GL_ARRAY_BUFFER, _vboId);
  glBufferData(GL_ARRAY_BUFFER, 3*2*8*sizeof(float), vertices_normals, GL_STATIC_DRAW);

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
                        8 * sizeof(float), // stride is size of one whole vertex.
                        nullptr // No offset into the buffer
  );

  glEnableVertexAttribArray(1);
  glVertexAttribPointer(1,  // Attribute ONE (normals)
                        3, // 3 floats per vertex normal
                        GL_FLOAT, // type is float
                        GL_FALSE, // they are not normalized
                        8 * sizeof(float), // stride is size of one whole vertex.
                        (GLvoid*) (3 * sizeof(float)) // offset into the buffer by 3 floats.
  );

  glEnableVertexAttribArray(2);
  glVertexAttribPointer(2, // Attribute TWO (Texcoords)
                        2, // 2 floats U,V per vertex
                        GL_FLOAT, // type is float
                        GL_FALSE, // they are not normalized
                        8*sizeof(float), // stride is size of one whole vertex.
                        (GLvoid*) (6 * sizeof(float)) // offset by the size of 6 floats.

  );



}

Floor::~Floor() { }

void Floor::bindVao() {
  glBindVertexArray(_vaoId);
}

void Floor::draw() {
  glBindTexture(GL_TEXTURE_2D, _texId);
  bindVao();
  glDrawArrays(GL_TRIANGLES, 0, 6);

}