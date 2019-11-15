#include "mesh.h"
#include <iostream>


Mesh::Mesh() {}

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

      int matches = fscanf(file,
                           "%d %d %d\n",
                           &vertexIndex[0],
                           &vertexIndex[1],
                           &vertexIndex[2]);

        if (matches != 3) {
          std::cerr << vertices.size() << std::endl;
          std::cerr << "Mesh cannot be read. Invalid options: " << pathtofile << std::endl;
          exit(-1);
        }

        indices_vertex.push_back(vertexIndex[0]);
        indices_vertex.push_back(vertexIndex[1]);
        indices_vertex.push_back(vertexIndex[2]);

      }
    }
  }

unsigned int Mesh::vertexCount() { return vertices.size(); }

float * Mesh::getVertexArrayPTR() { return (float * ) vertices.data(); }

unsigned int Mesh::indexCount() { return indices_vertex.size(); }

unsigned int * Mesh::getIndexArrayPTR() { return (unsigned int * ) indices_vertex.data(); }

void Mesh::printAttributes() {
  std::cout << "Size of vertices: \t\t" << vertices.size() << " \t(glm::vec3)" << std::endl;
  std::cout << "Size of normals: \t\t" << normals.size() <<  " \t(glm::vec3)" << std::endl;
  std::cout << "Size of uvs: \t\t\t" << uvs.size() << " \t(glm::vec2)" << std::endl;
  std::cout << "Size of indices_vertex: \t" << indices_vertex.size() << " \t(uint)" << std::endl;
  std::cout << "Size of indices_normal: \t" << indices_normal.size() << " \t(uint)" << std::endl;
  std::cout << "Size of indices_uvs: \t\t" << indices_uv.size() << " \t(uint)" << std::endl;
}




