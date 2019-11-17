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

#include "shader.h"
#include "cubedata.h"
#include "vertexbuffer.h"
#include "indexbuffer.h"
#include "mesh.h"

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

  GLuint vs = vertexShader.compile();
  GLuint fs = fragmentShader.compile();

  GLuint program = glCreateProgram();
  GLCALL(glAttachShader(program, fs));
  GLCALL(glAttachShader(program, vs));
  GLCALL(glLinkProgram(program));
  GLCALL(glUseProgram(program));

  GLuint s_MatrixID = glGetUniformLocation(program, "MVP");
  GLuint s_ModelID = glGetUniformLocation(program, "Model");

  GLuint s_ambientLightStrength = glGetUniformLocation(program, "ambientLightStrength");
  GLuint s_ambientLightColor = glGetUniformLocation(program, "ambientLightColor");
  GLuint s_specularStrength = glGetUniformLocation(program, "specularStrength");
  GLuint s_lightPosition = glGetUniformLocation(program, "lightPosition");
  GLuint s_viewPosition = glGetUniformLocation(program, "viewPosition");


  /* *********** End Shader code ************* */
  

  Mesh teapot = Mesh();
  teapot.loadOBJ("../res/bunny_new.obj");
  teapot.printAttributes();

  // with this call, we double our vertex vector size, but every second element is now a normal.
  teapot.interlaceVertexAndNormal();
  teapot.printAttributes();


  VertexBuffer teapot_vb;
  IndexBuffer teapot_ib;

  teapot_ib.genBuffer();
  teapot_ib.bind();
  teapot_ib.setBufferPtr(teapot.getIndexArrayPTR());
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned int) * teapot.indexCount(), teapot_ib.getBufferPtr(), GL_STATIC_DRAW);

  teapot_vb.genBuffer();
  teapot_vb.bind();
  teapot_vb.setBufferPtr(teapot.getVertexArrayPTR());
  glBufferData(GL_ARRAY_BUFFER, teapot.vertexCount() * sizeof(float) * 6, teapot_vb.getBufferPtr(), GL_STATIC_DRAW);

  // Create vertex array, the vehicle from cpu land to graphics land
  GLuint vao = 0;
  GLCALL(glGenVertexArrays(1, &vao));
  GLCALL(glBindVertexArray(vao));
  GLCALL(glEnableVertexAttribArray(0));
  GLCALL(glEnableVertexAttribArray(1));

  int rotationCounter = 0;

  // Render loop.
  while(!glfwWindowShouldClose(window)) {

    /* ******** Drawing code starts here ************ */

    // Tell OpenGL to clean off the canvas.
    GLCALL(glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT));

    // Tell OpenGL we're using the vertex array,
    // We're going to use it as the vehicle for buffers
    // from application space to hardware space.
    GLCALL(glBindVertexArray(vao));

    // Rebind our vertex data
    teapot_vb.bind();

    // Remind OpenGl of how we like our data, every single draw call.
    // Tell OpenGl about our floats XYZ, they come first, as specified in layout.
    GLCALL(glVertexAttribPointer(0,  // Attribute ZERO (vertices)
                                 3, // 3 floats per vertex XYZ
                                 GL_FLOAT, // type is float
                                 GL_FALSE, // they are not normalized
                                 6 * sizeof(float), // stride is size of one whole vertex.
                                 nullptr // No offset into the buffer
    ));


    GLCALL(glVertexAttribPointer(1,  // Attribute ONE (normals)
                                 3, // 3 floats per vertex normal
                                 GL_FLOAT, // type is float
                                 GL_FALSE, // they are not normalized
                                 6 * sizeof(float), // stride is size of one whole vertex.
                                 (GLvoid*) (3 * sizeof(float)) // offset into the buffer by 3 floats.
    ));


    // Bind the index buffer
    teapot_ib.bind();


    /* ********* END drawing code *********** */

    /* ********* Start Matrix code ******/

    // Projection matrix : 45° Field of View, 4:3 ratio, display range : 0.1 unit <-> 100 units
    glm::mat4 Projection = glm::perspective(glm::radians(45.0f), (float) WIDTH / (float)HEIGHT, 0.1f, 100.0f);

    // Or, for an ortho camera :
    //glm::mat4 Projection = glm::ortho(-10.0f,10.0f,-10.0f,10.0f,0.0f,100.0f); // In world coordinates

    // Camera matrix
    glm::vec3 cameraPosition = glm::vec3(8,6,6);
    glm::mat4 View = glm::lookAt(
        cameraPosition, // Camera is at (4,3,3), in World Space
        glm::vec3(0,0,0), // and looks at the origin
        glm::vec3(0,1,0)  // Head is up (set to 0,-1,0 to look upside-down)
    );

    // Model matrix : an identity matrix (model will be at the origin)
    glm::vec3 scale = glm::vec3(1.5f, 1.5f, 1.5f);
    glm::mat4 Model = glm::scale(glm::mat4(1.0f), scale);

    Model = glm::rotate(Model, .0005f * rotationCounter, glm::vec3(0, 1, 0));
    rotationCounter++;

    // Our ModelViewProjection : multiplication of our 3 matrices
    glm::mat4 mvp = Projection * View * Model; // Remember, matrix multiplication is the other way around

    // Update the uniform MVP matrix every draw call.
    GLCALL(glUniformMatrix4fv(s_MatrixID, 1, GL_FALSE, &mvp[0][0]));
    GLCALL(glUniformMatrix4fv(s_ModelID, 1, GL_FALSE, &Model[0][0]));
    GLCALL(glUniform3f(s_viewPosition, cameraPosition.x, cameraPosition.y, cameraPosition.z));

    /* ******** End matrix code ******* */

    /* ******* Lighting Code ******* */

    float ambientStrength = 0.1f;
    float specularStrength = 0.7f;
    glm::vec3 ambientColor = glm::vec3(1.0f, 1.0f, 1.0f);
    glm::vec3 lightPos = glm::vec3(-4,-4,4);

    GLCALL(glUniform1f(s_ambientLightStrength, ambientStrength));
    GLCALL(glUniform1f(s_specularStrength, specularStrength));
    GLCALL(glUniform3f(s_ambientLightColor, ambientColor.r, ambientColor.g, ambientColor.b));
    GLCALL(glUniform3f(s_lightPosition, lightPos.x, lightPos.y, lightPos.z));

    // Draw our model using index buffering.
    GLCALL(glDrawElements(GL_TRIANGLES, teapot.indices_vertex.size(), GL_UNSIGNED_INT, nullptr));

    // Reset all the bound buffers / arrays to unbound.
    GLCALL(glBindBuffer(GL_ARRAY_BUFFER, 0));
    GLCALL(glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0));
    GLCALL(glBindVertexArray(0));

    // update other events like input handling
    glfwPollEvents();
    // put the stuff we've been drawing onto the display
    glfwSwapBuffers(window);
  }

  //GLCALL(glDeleteBuffers(1, &teapot_ib.getBufferId())));
  //GLCALL(glDeleteBuffers(1, &vertices));

  // close GL context and any other GLFW resources
  glfwTerminate();
  return 0;
}



