#include "r_rabbit.h"
#include "stb_image.h"
#include "vertexbuffer.h"
#include "indexbuffer.h"
#include <iostream>

r_Rabbit::r_Rabbit(GLuint shaderProgramId, Camera *camera) {
  _shaderId = shaderProgramId;
  _camera = camera;
  _transform = Transform();
  _mesh = Mesh();
  _vb = VertexBuffer();
  _ib = IndexBuffer();
}

void r_Rabbit::setup() {
  _shaderMVPId = glGetUniformLocation(_shaderId, "MVP");
  _shaderModelId = glGetUniformLocation(_shaderId, "Model");
  _shaderUseTexId = glGetUniformLocation(_shaderId, "USE_TEX");


  int width, height, nrChannels;
  unsigned char *data = stbi_load("../shaders_and_textures/treetex.jpg", &width, &height, &nrChannels, 0);
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
    std::cerr << "Failed to load texture rabbit" << std::endl;
    exit(-1);
  }
  stbi_image_free(data);

  _mesh.loadOBJ("../res/bunny_uv.obj");
  // with this call, we double our vertex vector size, but every second element is now a normal.
  _mesh.interlaceAll();

  // Set up mesh indices, these are updated because of the call to interlaceVertexandNormal.
  // interlaceVertexandNormal creates unique vertices by vertex/normal index and extrapolates new indices.

  _ib.genBuffer();
  _ib.bind();
  _ib.setBufferPtr(_mesh.getIndexArrayPTR());
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned int) * _mesh.indexCount(), _ib.getBufferPtr(), GL_STATIC_DRAW);

  // Set up mesh vertices. As above, we now have 6 floats per vertex, (x y z), (nx ny nz).
  _vb.genBuffer();
  _vb.bind();
  _vb.setBufferPtr((void *)_mesh.getVertexArrayPTR());

  glBufferData(GL_ARRAY_BUFFER, _mesh.vertexCount() * sizeof(float) * 8, _vb.getBufferPtr(), GL_STATIC_DRAW);


  // Create vertex array, the vehicle from cpu land to graphics land
  glGenVertexArrays(1, &_vaoId);
  glBindVertexArray(_vaoId);

  // Remind OpenGl of how we like our data, every single draw call.
  // Tell OpenGl about our floats XYZ, they come first, as specified in layout.
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0,  // Attribute ZERO (vertices)
                               3, // 3 floats per vertex XYZ
                               GL_FLOAT, // type is float
                               GL_FALSE, // they are not normalized
                               8*sizeof(float), // stride is size of one whole vertex.
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
  glVertexAttribPointer(2,  // Attribute ONE (normals)
                        2, // 3 floats per vertex normal
                        GL_FLOAT, // type is float
                        GL_FALSE, // they are not normalized
                        8*sizeof(float), // stride is size of one whole vertex.
                        (GLvoid*) (6 * sizeof(float)) // offset into the buffer by 3 floats.
  );

}

void r_Rabbit::draw() {
  glm::mat4 mvp = _camera->getProjection() * _camera->getView() * _transform.getModelMatrix();
  glUniformMatrix4fv(_shaderMVPId, 1, GL_FALSE, &mvp[0][0]);
  glUniformMatrix4fv(_shaderModelId, 1, GL_FALSE, &_transform.getModelMatrix()[0][0]);
  glUniform1i(_shaderUseTexId, 1);


  glBindTexture(GL_TEXTURE_2D, _texId);
  glBindVertexArray(_vaoId);
  _ib.bind();
  glDrawElements(GL_TRIANGLES, _mesh.indexCount(), GL_UNSIGNED_INT, nullptr);

  glBindBuffer(GL_ARRAY_BUFFER, 0);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
  glBindVertexArray(0);
}