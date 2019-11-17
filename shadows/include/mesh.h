#ifndef SHADOWS_SHADOWS_INCLUDE_MESH_H_
#define SHADOWS_SHADOWS_INCLUDE_MESH_H_

#include <vector>
#include "glm/vec2.hpp"
#include "glm/vec3.hpp"

class Mesh {
 public:
  Mesh();

  void loadOBJ(std::string pathtofile);
  unsigned int vertexCount();
  float * getVertexArrayPTR();
  float * getNormalArrayPTR();
  unsigned int indexCount();
  unsigned int * getIndexArrayPTR();
  void interlaceVertexAndNormal();
  void printAttributes();


  std::vector<glm::vec3> normals;
  std::vector<glm::vec3> vertices;
  std::vector<glm::vec2> uvs;
  std::vector<unsigned int> indices_vertex;
  std::vector<unsigned int> indices_normal;
  std::vector<unsigned int> indices_uv;

};

#endif //SHADOWS_SHADOWS_INCLUDE_MESH_H_
