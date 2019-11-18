// Let's define a macro and a function to catch any OpenGl errors.

#ifdef _DEBUG
#define GLCALL(stmt) do { \
            stmt; \
            CheckOpenGLError(#stmt, __FILE__, __LINE__); \
        } while (0)
#else
#define GLCALL(stmt) stmt
#endif

#define WIDTH 640
#define HEIGHT 480

#include "GL/glew.h"
#include "GLFW/glfw3.h"
#include "glm/gtc/matrix_transform.hpp"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "shader.h"
#include "cubedata.h"
#include "vertexbuffer.h"
#include "indexbuffer.h"
#include "mesh.h"
#include "camera.h"
#include "scene.h"

void CheckOpenGLError(const char* stmt, const char* fname, int line)
{
  GLenum err = glGetError();
  if (err != GL_NO_ERROR)
  {
    printf("OpenGL error %08x, at %s:%i - for %s\n", err, fname, line, stmt);
    exit(-1);
  }
}

int main() {
  // start GL context and O/S window using the GLFW helper library
  if (!glfwInit()) {
    fprintf(stderr, "ERROR: could not start GLFW3\n");
    return 1;
  }

  // uncomment these lines if on Apple OS X
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
  glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

  GLFWwindow* window = glfwCreateWindow(WIDTH, HEIGHT, "ECSE 4750", NULL, NULL);
  if (!window) {
    fprintf(stderr, "ERROR: could not open window with GLFW3\n");
    glfwTerminate();
    return 1;
  }
  glfwMakeContextCurrent(window);

  // start GLEW extension handler
  glewExperimental = GL_TRUE;
  glewInit();

  // get version info
  const GLubyte* renderer = glGetString(GL_RENDERER); // get renderer string
  const GLubyte* version = glGetString(GL_VERSION); // version as a string
  printf("Renderer: %s\n", renderer);
  printf("OpenGL version supported %s\n", version);

  // tell GL to only draw onto a pixel if the shape is closer to the viewer
  GLCALL(glEnable(GL_DEPTH_TEST)); // enable depth-testing
  GLCALL(glDepthFunc(GL_LESS)); // depth-testing interprets a smaller value as "closer"

  /* ******** Set up shaders here ************ */

  Shader vertexShader(GL_VERTEX_SHADER);
  Shader fragmentShader(GL_FRAGMENT_SHADER);
  vertexShader.loadShaderSource("../src/shaders/VERTEX.shader");
  fragmentShader.loadShaderSource("../src/shaders/FRAG.shader");
  vertexShader.compile();
  fragmentShader.compile();
  GLuint program = buildShaderProgram(vertexShader, fragmentShader);

  // Link uniforms
  // TODO change shader to perform per model MVP
  // TODO use a UBO for the uniforms that do not need per model update
  // TODO make an end all be all model class, that contains transform info
  // TODO maybe make a light class to handle a. mesh rendering b. light transform.
  // TODO once robust enough, go for the shadows!
  
  GLuint s_MatrixID = glGetUniformLocation(program, "MVP");
  GLuint s_ModelID = glGetUniformLocation(program, "Model");
  GLuint s_ambientLightStrength = glGetUniformLocation(program, "ambientLightStrength");
  GLuint s_ambientLightColor = glGetUniformLocation(program, "ambientLightColor");
  GLuint s_specularStrength = glGetUniformLocation(program, "specularStrength");
  GLuint s_lightPosition = glGetUniformLocation(program, "lightPosition");
  GLuint s_viewPosition = glGetUniformLocation(program, "viewPosition");


  /* *********** End Shader code ************* */


  // Create the floor. defined in scene.cpp
  Floor floor_mesh = Floor(10.0f);

  // Create a mesh object for the mesh we're going to load.
  Mesh mesh = Mesh();
  mesh.loadOBJ("../res/bunny_new.obj");

  // with this call, we double our vertex vector size, but every second element is now a normal.
  mesh.interlaceVertexAndNormal();
  mesh.printAttributes();

  VertexBuffer mesh_vb;
  IndexBuffer mesh_ib;

  // Set up mesh indices, these are updated because of the call to interlaceVertexandNormal.
  // interlaceVertexandNormal creates unique vertices by vertex/normal index and extrapolates new indices.
  mesh_ib.genBuffer();
  mesh_ib.bind();
  mesh_ib.setBufferPtr(mesh.getIndexArrayPTR());
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned int) * mesh.indexCount(), mesh_ib.getBufferPtr(), GL_STATIC_DRAW);

  // Set up mesh vertices. As above, we now have 6 floats per vertex, (x y z), (nx ny nz).
  mesh_vb.genBuffer();
  mesh_vb.bind();
  mesh_vb.setBufferPtr(mesh.getVertexArrayPTR());
  glBufferData(GL_ARRAY_BUFFER, mesh.vertexCount() * sizeof(float) * 6, mesh_vb.getBufferPtr(), GL_STATIC_DRAW);


  // Create vertex array, the vehicle from cpu land to graphics land
  GLuint mesh_vao = 0;
  GLCALL(glGenVertexArrays(1, &mesh_vao));
  GLCALL(glBindVertexArray(mesh_vao));

  // Remind OpenGl of how we like our data, every single draw call.
  // Tell OpenGl about our floats XYZ, they come first, as specified in layout.
  GLCALL(glEnableVertexAttribArray(0));
  GLCALL(glVertexAttribPointer(0,  // Attribute ZERO (vertices)
                               3, // 3 floats per vertex XYZ
                               GL_FLOAT, // type is float
                               GL_FALSE, // they are not normalized
                               6 * sizeof(float), // stride is size of one whole vertex.
                               nullptr // No offset into the buffer
  ));

  GLCALL(glEnableVertexAttribArray(1));
  GLCALL(glVertexAttribPointer(1,  // Attribute ONE (normals)
                               3, // 3 floats per vertex normal
                               GL_FLOAT, // type is float
                               GL_FALSE, // they are not normalized
                               6 * sizeof(float), // stride is size of one whole vertex.
                               (GLvoid*) (3 * sizeof(float)) // offset into the buffer by 3 floats.
  ));


  // Set up things that need to be updated in the loop.

  int rotationCounter = 0;
  double lightRotationAngle = 0; // degrees.
  double lightRotationRadius = 15;

  // Render loop.
  while(!glfwWindowShouldClose(window)) {

    // Tell OpenGL to clean off the canvas.
    GLCALL(glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT));

    /* ********* Start Matrix code ******/


    Camera camera = Camera(45.0f, (float)WIDTH/(float)HEIGHT, glm::vec3(-8, 8, -4), glm::vec3(0,0,0));
    glm::vec3 scale = glm::vec3(1.5f, 1.5f, 1.5f);
    glm::mat4 Model = glm::scale(glm::mat4(1.0f), scale);
    glm::mat4 mvp = camera.getProjection() * camera.getView() * Model; // Remember, matrix multiplication is the other way around

    GLCALL(glUniformMatrix4fv(s_MatrixID, 1, GL_FALSE, &mvp[0][0]));
    GLCALL(glUniformMatrix4fv(s_ModelID, 1, GL_FALSE, &Model[0][0]));
    GLCALL(glUniform3f(s_viewPosition, camera.getWorldX(), camera.getWorldY(), camera.getWorldZ()));


    /* ******** End matrix code ******* */

    /* ******* Lighting Code ******* */

    float ambientStrength = 0.5f;
    float specularStrength = 0.3f;
    glm::vec3 ambientColor = glm::vec3(1.0f, 1.0f, 1.0f);
    glm::vec3 lightPos = glm::vec3(0,0,0);

    lightRotationAngle += 0.001f;
    float sinXradius = sin(lightRotationAngle) * lightRotationRadius;
    float cosXradius = cos(lightRotationAngle) * lightRotationRadius;
    lightPos = glm::vec3(lightPos.x + cosXradius, lightPos.y, lightPos.z + sinXradius );

    GLCALL(glUniform1f(s_ambientLightStrength, ambientStrength));
    GLCALL(glUniform1f(s_specularStrength, specularStrength));
    GLCALL(glUniform3f(s_ambientLightColor, ambientColor.r, ambientColor.g, ambientColor.b));
    GLCALL(glUniform3f(s_lightPosition, lightPos.x, lightPos.y, lightPos.z));

    /* ****** End lighting code ****** */
    
    // Firstly draw the floor.
    floor_mesh.draw();


    // Finally draw the MESH
    GLCALL(glBindVertexArray(mesh_vao));
    mesh_ib.bind(); // Bind the index buffer
    GLCALL(glDrawElements(GL_TRIANGLES, mesh.indices_vertex.size(), GL_UNSIGNED_INT, nullptr));


    /* ********* END drawing code *********** */

    // Reset all the bound buffers / arrays to unbound.
    GLCALL(glBindBuffer(GL_ARRAY_BUFFER, 0));
    GLCALL(glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0));
    GLCALL(glBindVertexArray(0));

    // update other events like input handling
    glfwPollEvents();
    // put the stuff we've been drawing onto the display
    glfwSwapBuffers(window);
  }

  //GLCALL(glDeleteBuffers(1, &mesh_ib.getBufferId())));
  //GLCALL(glDeleteBuffers(1, &vertices));

  // close GL context and any other GLFW resources
  glfwTerminate();
  return 0;
}



