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
#include "entity.h"

#include "r_floor.h"

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
  vertexShader.loadShaderSource("../src/shaders/VERTEX_tex.shader");
  fragmentShader.loadShaderSource("../src/shaders/FRAG_tex.shader");
  vertexShader.compile();
  fragmentShader.compile();
  GLuint program = buildShaderProgram(vertexShader, fragmentShader);

  
  GLuint s_MatrixID = glGetUniformLocation(program, "MVP");
  GLuint s_ModelID = glGetUniformLocation(program, "Model");
  GLuint s_ambientLightStrength = glGetUniformLocation(program, "ambientLightStrength");
  GLuint s_ambientLightColor = glGetUniformLocation(program, "ambientLightColor");
  GLuint s_specularStrength = glGetUniformLocation(program, "specularStrength");
  GLuint s_lightPosition = glGetUniformLocation(program, "lightPosition");
  GLuint s_viewPosition = glGetUniformLocation(program, "viewPosition");


  /* *********** End Shader code ************* */


  // Set up things that need to be updated in the loop.

  int rotationCounter = 0;
  double lightRotationAngle = 0; // degrees.
  double lightRotationRadius = 15;


  Camera camera = Camera(45.0f, (float)WIDTH/(float)HEIGHT, glm::vec3(-8, 8, -4), glm::vec3(0,0,0));

  /* The r_ in r_Floor means that the class is derived from Renderable()
   * What this does is:
   *    Allows us to take all the messy setup code and put it into one spot that pertains to that particular mesh.
   *    Allows us to simplify the code
   *    Allows for model specific configuration
   *    Generally makes things easier.
   * There are two methods from Renderable() that the descendant class must implement.
   * These are setup() and draw()
   * setup() is where you declare EVERYTHING you need to render, set up vaos, vbos, ibos
   * draw() is where you use the data that you have set up */

  r_Floor r_floor = r_Floor(program, &camera);
  r_floor.setup();

  /* Doing transforms with this scheme is very easy.
   * Simply grab the entity from the Renderable using getEntity()
   * and do your transformations with the pointer.
   * any transformation defined in entity.h works */
  
  r_floor.getEntity()->setModelScale(5.0f);


  // Render loop.
  while(!glfwWindowShouldClose(window)) {

    // Tell OpenGL to clean off the canvas.
    GLCALL(glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT));

    /* ********* Start Matrix code ******/


    glm::vec3 scale = glm::vec3(1.5f, 1.5f, 1.5f);
    glm::mat4 Model = glm::scale(glm::mat4(1.0f), scale);
    glm::mat4 mvp = camera.getProjection() * camera.getView() * Model; // Remember, matrix multiplication is the other way around

    GLCALL(glUniform3f(s_viewPosition, camera.getWorldX(), camera.getWorldY(), camera.getWorldZ()));

    /* ******** End matrix code ******* */

    /* ******* Lighting Code ******* */

    float ambientStrength = 0.5f;
    float specularStrength = 0.3f;
    glm::vec3 ambientColor = glm::vec3(1.0f, 1.0f, 1.0f);
    glm::vec3 lightPos = glm::vec3(0,0,0);

    lightRotationAngle += 0.01f;
    float sinXradius = sin(lightRotationAngle) * lightRotationRadius;
    float cosXradius = cos(lightRotationAngle) * lightRotationRadius;
    lightPos = glm::vec3(lightPos.x + cosXradius, lightPos.y, lightPos.z + sinXradius );

    GLCALL(glUniform1f(s_ambientLightStrength, ambientStrength));
    GLCALL(glUniform1f(s_specularStrength, specularStrength));
    GLCALL(glUniform3f(s_ambientLightColor, ambientColor.r, ambientColor.g, ambientColor.b));
    GLCALL(glUniform3f(s_lightPosition, lightPos.x, lightPos.y, lightPos.z));

    /* ****** End lighting code ****** */


    r_floor.draw();

    /* ********* END drawing code *********** */

    // Reset all the bound buffers / arrays to unbound.
    GLCALL(glBindBuffer(GL_ARRAY_BUFFER, 0));
    GLCALL(glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0));
    GLCALL(glBindVertexArray(0));

    // update other events like input handling
    glfwPollEvents();
    // put the stuff we've been drawing onto the display
    glfwSwapBuffers(window);

    rotationCounter++;
  }

  //GLCALL(glDeleteBuffers(1, &mesh_ib.getBufferId())));
  //GLCALL(glDeleteBuffers(1, &vertices));

  // close GL context and any other GLFW resources
  glfwTerminate();
  return 0;
}



