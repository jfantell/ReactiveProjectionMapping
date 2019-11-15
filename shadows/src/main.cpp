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

  GLFWwindow* window = glfwCreateWindow(WIDTH, HEIGHT, "Hello Triangle", NULL, NULL);
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

  GLuint MatrixID = glGetUniformLocation(program, "MVP");


  /* *********** End Shader code ************* */

  Mesh * teapot = new Mesh();
  teapot->loadOBJ("../res/teapot.obj");
  teapot->printAttributes();

  //for (int i = 0; i < 15; i++) std::cout << *((unsigned int*) &(teapot->indices_vertex[i])) << std::endl;

  GLuint vertices = 0;
  GLCALL(glGenBuffers(1, &vertices));
  GLCALL(glBindBuffer(GL_ARRAY_BUFFER, vertices));
  GLCALL(glBufferData(GL_ARRAY_BUFFER, 3 * sizeof(float) * teapot->vertexCount(), teapot->getVertexArrayPTR(), GL_STATIC_DRAW));


  GLuint indices = 0;
  GLCALL(glGenBuffers(1, &indices));
  GLCALL(glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indices));
  GLCALL(glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned int) * teapot->indexCount(), teapot->getIndexArrayPTR(), GL_STATIC_DRAW));


  // Create vertex array, the vehicle from cpu land to graphics land
  GLuint vao = 0;
  GLCALL(glGenVertexArrays(1, &vao));
  GLCALL(glBindVertexArray(vao));
  GLCALL(glEnableVertexAttribArray(0));

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

    // Remind OpenGl of how we like our data, every single draw call.
    GLCALL(glBindBuffer(GL_ARRAY_BUFFER, vertices));

    // Tell OpenGl about our floats XYZ, they come first, as specified in layout.
    GLCALL(glVertexAttribPointer(0,  // Attribute ZERO (vertices)
                                 3, // 3 floats per vertex
                                 GL_FLOAT, // type is float
                                 GL_FALSE, // they are not normalized
                                 0, // stride is size of one whole vertex.
                                 NULL // No offset into the buffer
    ));


    // Bind the index buffer
    GLCALL(glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indices));


    /* ********* END drawing code *********** */

    /* ********* Start Matrix/ Input handling code ******/

    // Projection matrix : 45° Field of View, 4:3 ratio, display range : 0.1 unit <-> 100 units
    glm::mat4 Projection = glm::perspective(glm::radians(45.0f), (float) WIDTH / (float)HEIGHT, 0.1f, 100.0f);

    // Or, for an ortho camera :
    //glm::mat4 Projection = glm::ortho(-10.0f,10.0f,-10.0f,10.0f,0.0f,100.0f); // In world coordinates

    // Camera matrix
    glm::mat4 View = glm::lookAt(
        glm::vec3(4,3,3), // Camera is at (4,3,3), in World Space
        glm::vec3(0,0,0), // and looks at the origin
        glm::vec3(0,1,0)  // Head is up (set to 0,-1,0 to look upside-down)
    );

    // Model matrix : an identity matrix (model will be at the origin)
    glm::vec3 scale = glm::vec3(.5f, .5f, .5f);
    glm::mat4 Model = glm::scale(glm::mat4(1.0f), scale);

    Model = glm::rotate(Model, .0005f * rotationCounter, glm::vec3(0, 1, 0));
    rotationCounter++;

    // Our ModelViewProjection : multiplication of our 3 matrices
    glm::mat4 mvp = Projection * View * Model; // Remember, matrix multiplication is the other way around

    // Update the uniform MVP matrix every draw call.
    GLCALL(glUniformMatrix4fv(MatrixID, 1, GL_FALSE, &mvp[0][0]));

    //GLCALL(glDrawArrays(GL_LINE_LOOP, 0, 18690));
    GLCALL(glDrawElements(GL_TRIANGLES, teapot->indices_vertex.size(), GL_UNSIGNED_INT, (GLvoid*)0));

    GLCALL(glBindBuffer(GL_ARRAY_BUFFER, 0));
    GLCALL(glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0));
    GLCALL(glBindVertexArray(0));

    // update other events like input handling
    glfwPollEvents();
    // put the stuff we've been drawing onto the display
    glfwSwapBuffers(window);
  }

  GLCALL(glDeleteBuffers(1, &indices));
  GLCALL(glDeleteBuffers(1, &vertices));

  // close GL context and any other GLFW resources
  glfwTerminate();
  return 0;
}



