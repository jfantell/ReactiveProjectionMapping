#ifndef SHADOWS_SHADOWS_INCLUDE_MESH_H_
#define SHADOWS_SHADOWS_INCLUDE_MESH_H_

#include <vector>
#include "glm/vec2.hpp"
#include "glm/vec3.hpp"

// Needed later for interlacing everything - see interlaceAll()
struct vertex_struct {
  glm::vec3 vertex;
  glm::vec3 normal;
  glm::vec2 uv;
};

// Needed later for interlacing everything - see interlaceAll()
struct indices {
  unsigned int vertex;
  unsigned int uv;
  unsigned int normal;

  // This operator required to use an unordered_map - see interlaceAll()
  bool const operator==(const indices &o) const {
    return vertex == o.vertex && uv == o.uv && normal == o.normal;
  }
};

// This std::hash specialization also required for unordered_map - see interlaceAll()
template<> struct std::hash<indices> {
  std::size_t operator()(const indices& i) const {
    return std::hash<unsigned int>()(i.vertex) + 7 ^
        std::hash<unsigned int>()(i.uv) + 2 ^
        std::hash<unsigned int>()(i.normal) + 3;
  }
};

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
  void interlaceAll();
  void printAttributes();
  bool isInterlaced() {return _interlaced; }


  std::vector<glm::vec3> normals;
  std::vector<glm::vec3> vertices;
  std::vector<glm::vec2> uvs;
  std::vector<unsigned int> indices_vertex;
  std::vector<unsigned int> indices_normal;
  std::vector<unsigned int> indices_uv;

  std::vector<vertex_struct> interlacedVertices;
  std::vector<unsigned int> interlacedIndices;

 private:
  bool _interlaced = false;

};

#endif //SHADOWS_SHADOWS_INCLUDE_MESH_H_
