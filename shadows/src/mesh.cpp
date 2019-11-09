#include "mesh.h"
#include <iostream>
#include <map>
#include <unordered_map>
#include <tuple>


Mesh::Mesh() {}

// Take in a path to an obj file.
void Mesh::loadOBJ(std::string pathtofile) {

  FILE * file = fopen(pathtofile.c_str(), "r");
  if( file == NULL ) {
    std::cerr << "Error loading mesh: " << pathtofile << std::endl;
    exit(-1);
  }

  while (1) {

    char lineHeader[128];
    // read the first word of the line
    int res = fscanf(file, "%s", lineHeader);
    if (res == EOF)
      break; // EOF = End Of File. Quit the loop.

    if (strcmp(lineHeader, "v") == 0) {
      glm::vec3 vertex;
      fscanf(file, "%f %f %f\n", &vertex.x, &vertex.y, &vertex.z);
      vertices.push_back(vertex);

    } else if (strcmp(lineHeader, "vt") == 0) {
      glm::vec2 uv;
      fscanf(file, "%f %f\n", &uv.x, &uv.y);
      uvs.push_back(uv);

    } else if (strcmp(lineHeader, "vn") == 0) {
      glm::vec3 normal;
      fscanf(file, "%f %f %f\n", &normal.x, &normal.y, &normal.z);
      normals.push_back(normal);

    } else if (strcmp(lineHeader, "f") == 0) {
      unsigned int vertexIndex[3];
      unsigned int vertexNormalIndex[3];
      unsigned int vertexTextureIndex[3];
      int matches = fscanf(file,
                     "%d/%d/%d %d/%d/%d %d/%d/%d\n",
                     &vertexIndex[0],
                     &vertexNormalIndex[0],
                     &vertexTextureIndex[0],
                     &vertexIndex[1],
                     &vertexNormalIndex[1],
                     &vertexTextureIndex[1],
                     &vertexIndex[2],
                     &vertexNormalIndex[2],
                     &vertexTextureIndex[2]
                     );
      if (matches == 9){
        indices_vertex.push_back(vertexIndex[0] - 1);
        indices_vertex.push_back(vertexIndex[1] - 1);
        indices_vertex.push_back(vertexIndex[2] - 1);

        indices_normal.push_back(vertexNormalIndex[0] - 1);
        indices_normal.push_back(vertexNormalIndex[1] - 1);
        indices_normal.push_back(vertexNormalIndex[2] - 1);

        indices_uv.push_back(vertexTextureIndex[0] - 1);
        indices_uv.push_back(vertexTextureIndex[1] - 1);
        indices_uv.push_back(vertexTextureIndex[2] - 1);

      } else {

        std::cerr << vertices.size() << std::endl;
        std::cerr << "Mesh cannot be read. Invalid options: " << pathtofile << std::endl;
        exit(-1);
      }
    }


      }
    }

unsigned int Mesh::vertexCount() {
  if (!_interlaced) return vertices.size();
  else return interlacedVertices.size();
}

float * Mesh::getVertexArrayPTR() {
  if (!_interlaced){
    return (float * ) vertices.data();
  }
  else {
    return (float *) interlacedVertices.data();
  }
}

float * Mesh::getNormalArrayPTR() {
  return (float *) normals.data(); }

unsigned int Mesh::indexCount() {
  if (!_interlaced ) return indices_vertex.size();
  else return interlacedIndices.size();
}

unsigned int * Mesh::getIndexArrayPTR() {
  if (!_interlaced) return (unsigned int * ) indices_vertex.data();
  else return (unsigned int * ) interlacedIndices.data();
}

void Mesh::interlaceVertexAndNormal() {
  /*  OpenGL only allows for one single index buffer.
      This makes it necessary to make vertices unique, and
      this becomes problematic when you have separate indices for
      vertices as well as their respective normal. What this does
      is create those unique vertex combos, and recreates vertex
      indices based on those new combos.
      RETURNS: a void pointer to the new buffer. */

  struct vertexcombo {
    glm::vec3 vertex;
    glm::vec3 normal;
  } combo;

  std::map<std::pair<unsigned int, unsigned int>, vertexcombo> vertexMap;
  std::map<std::pair<unsigned int, unsigned int>, unsigned int> indexMap;

  std::pair<unsigned int, unsigned int> indexVertexNormal;

  // This iteration assumes that you have the same number of vertex and normal indices.
  // Check that here.

  if (indices_vertex.size() != indices_normal.size()) {
    std::cerr << "Cannot interlace vertex and normals! Count not equal!" << std::endl;
    exit(-1);
  }

  for (int i = 0; i < indices_vertex.size(); i++) {
    indexVertexNormal.first = indices_vertex[i];
    indexVertexNormal.second = indices_normal[i];
    combo.vertex = vertices[indices_vertex[i]];
    combo.normal  = normals[indices_normal[i]];
    // Insert the unique vertex.
    vertexMap.insert(std::make_pair(indexVertexNormal, combo));
    // Insert the combo of indices, and a pointer to the combo in the first map.
    indexMap.insert(std::make_pair(indexVertexNormal, 0));
  }

  // Everything is inside the map. Zero our vectors to minimize memory usage.
  vertices.clear();
  normals.clear();

  // Allocate a new array with the proper size and create indices as well.
  vertexcombo * newBuf = (vertexcombo*) malloc(sizeof(vertexcombo) * vertexMap.size());
  unsigned int indexCounter = 0;
  // Enumerate all of the new unique vertices.
  for (auto itr = indexMap.begin(); itr != indexMap.end(); itr++) {
    itr->second = indexCounter;
    indexCounter++;
  }

  std::vector<unsigned int> newIndices;
  unsigned int uniqueVertexID = 0;

  // now for every combo of indices, copy over the corresponding vertex to the buffer indexed by the unique id.
  for (unsigned int i = 0; i < indices_vertex.size(); i++) {

    // get the vertex's new ID
    uniqueVertexID = indexMap[std::make_pair(indices_vertex[i], indices_normal[i])];
    newIndices.push_back(uniqueVertexID);

    // copy that vertex to the specified slot (corresponding to ID)
    newBuf[uniqueVertexID] = vertexMap[std::make_pair(indices_vertex[i], indices_normal[i])];

  }

  // get rid of the unnecessary old index data.
  indices_normal.clear();
  indices_vertex.clear();

  // update to the newest index array now.
  indices_vertex = newIndices;
  std::cout << "New size of vertices: " << vertexMap.size() << " * sizeof(float) * 6" << std::endl;

  // Sequentially push vertex data and normal data back into the vertices vector.
  // So now we have double the number of vertices, but second element is really that vertex's normal.
  for (int i = 0; i < vertexMap.size(); i++) {
    vertices.push_back(newBuf[i].vertex);
    vertices.push_back(newBuf[i].normal);
  }
  delete newBuf;

}


void Mesh::interlaceAll() {

  indices i_combo;
  vertex_struct v_combo;

  // Create two unordered maps.
  // vertexMap maps the combo of indices to the vertex data it represents.
  // indexMap is used to pull things back out of the map.
  std::unordered_map<indices, vertex_struct> vertexMap;
  std::unordered_map<indices, unsigned int> indexMap;

  // If sizes are unequal, we can't perform this operation.
  if (indices_vertex.size() != indices_normal.size() || indices_uv.size() != indices_vertex.size()) {
    std::cerr << "Cannot interlace vertex and normals! Count not equal!" << std::endl;
    exit(-1);
  }

  // Push everything to the maps.
  for (int i = 0; i < indices_vertex.size(); i++) {

    i_combo.vertex = indices_vertex[i];
    i_combo.normal = indices_normal[i];
    i_combo.uv = indices_uv[i];

    v_combo.vertex = vertices[indices_vertex[i]];
    v_combo.normal = normals[indices_normal[i]];
    v_combo.uv = uvs[indices_uv[i]];

    vertexMap[i_combo] = v_combo;
    indexMap[i_combo] = 0;

  }
  // reduce our memory footprint
  vertices.clear();
  normals.clear();
  uvs.clear();

  // Allocate a new array with the proper size and create indices as well.
  vertex_struct * newBuf = (vertex_struct *) malloc(sizeof(vertex_struct) * vertexMap.size());

  // Enumerate all of the new unique vertices.
  unsigned int indexCounter = 0;
  for (auto itr = indexMap.begin(); itr != indexMap.end(); itr++) {
    itr->second = indexCounter;
    indexCounter++;
  }

  std::vector<unsigned int> newIndices;
  unsigned int uniqueVertexID = 0;

  // now for every combo of indices, copy over the corresponding vertex to the buffer indexed by the unique id.
  for (unsigned int i = 0; i < indices_vertex.size(); i++) {

    i_combo.vertex = indices_vertex[i];
    i_combo.normal = indices_normal[i];
    i_combo.uv = indices_uv[i];

    // get the vertex's new ID
    uniqueVertexID = indexMap[i_combo];
    newIndices.push_back(uniqueVertexID);

    // copy that vertex to the specified slot (corresponding to ID)
    vertex_struct v = vertexMap[i_combo];
    newBuf[uniqueVertexID] = vertexMap[i_combo];

  }

  // get rid of the unnecessary old index data.
  indices_normal.clear();
  indices_vertex.clear();
  indices_uv.clear();

  // update to the newest index array now.
  interlacedIndices = newIndices;

  // Sequentially push vertex data and normal data back into the vertices vector.
  // So now we have double the number of vertices, but second element is really that vertex's normal.

  for (int i = 0; i < vertexMap.size(); i++) {
      interlacedVertices.push_back(newBuf[i]);
  }

  delete newBuf;
  _interlaced = true;
}

void Mesh::printAttributes() {
  if (!_interlaced) {
    std::cout << "MESH IS NOT INTERLACED." << std::endl;
    std::cout << "Size of vertices: \t\t" << vertices.size() << " \t(glm::vec3)" << std::endl;
    std::cout << "Size of normals: \t\t" << normals.size() << " \t(glm::vec3)" << std::endl;
    std::cout << "Size of uvs: \t\t\t" << uvs.size() << " \t(glm::vec2)" << std::endl;
    std::cout << "Size of indices_vertex: \t" << indices_vertex.size() << " \t(uint)" << std::endl;
    std::cout << "Size of indices_normal: \t" << indices_normal.size() << " \t(uint)" << std::endl;
    std::cout << "Size of indices_uvs: \t\t" << indices_uv.size() << " \t(uint)" << std::endl;
  }
  else {
    std::cout << "MESH IS INTERLACED." << std::endl;
    std::cout << "Size of vertices (vertex, normal, uv): \t" << interlacedVertices.size() << std::endl;
    std::cout << "Size of extrapolated indices \t\t" << interlacedIndices.size() << std::endl;
  }
}




