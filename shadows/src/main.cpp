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
#include "transform.h"
#include "pointlight.h"

#include "r_floor.h"
#include "r_rabbit.h"
#include "r_lightmarker.h"
#include "inputhandler.h"

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


  // All uniforms required for this to work. They are updated elsewhere (camera, renderable, light)
  GLuint s_MatrixID = glGetUniformLocation(program, "MVP");
  GLuint s_ModelID = glGetUniformLocation(program, "Model");
  GLuint s_ambientLightStrength = glGetUniformLocation(program, "ambientLightStrength");
  GLuint s_ambientLightColor = glGetUniformLocation(program, "ambientLightColor");
  GLuint s_specularStrength = glGetUniformLocation(program, "specularStrength");
  GLuint s_lightPosition = glGetUniformLocation(program, "lightPosition");
  GLuint s_viewPosition = glGetUniformLocation(program, "viewPosition");


  /* *********** End Shader code ************* */


  // Set up things that need to be updated in the loop.

  // Create our camera.
  Camera camera = Camera(window, program, 45.0f, (float)WIDTH/(float)HEIGHT, glm::vec3(-8, 8, -4), glm::vec3(0,0,0));
  InputHandler input = InputHandler(window);

  /* The r_ in r_Floor means that the class is derived from Renderable()
   * What this does is:
   *    Allows us to take all the messy setup code and put it into one spot that pertains to that particular mesh.
   *    Allows us to simplify the code
   *    Allows for model specific configuration
   *    Generally makes things easier.
   * There are two methods from Renderable() that the descendant class must implement.
   * These are setup() and draw()
   * setup() is where you declare EVERYTHING you need to render, set up vaos, vbos, ibos
   * draw() is where you use the data that you have set up, update shader uniforms, etc. */

  // Create an instance of the floor renderable and set it up.
  r_Floor r_floor = r_Floor(program, &camera);
  r_floor.setup();

  /* Doing transforms with this scheme is very easy.
   * Simply grab the entity from the Renderable using getEntity()
   * and do your transformations with the pointer.
   * any transformation defined in entity.h works */

  // Let's scale up the floor.
  r_floor.getTransform()->setModelScale(10.0f);
  r_floor.getTransform()->setY(-.5);

  // Declare our renderable rabbit model and set it up.
  r_Rabbit r_rabbit = r_Rabbit(program, &camera);
  r_rabbit.setup();

  // Let's scale up our rabbit.
  r_rabbit.getTransform()->setModelScale(2.5f);
  r_rabbit.getTransform()->setY(-5);

  PointLight light = PointLight(program);
  light.setAmbientColor(glm::vec3(1.0f, 1.0f, 1.0f));
  light.setAmbientStrength(0.1f);
  light.setDiffuseStrength(1.2f);
  light.setSpecularStrength(0.5f);

  r_LightMarker lightmarker = r_LightMarker(program, &camera, &light);
  lightmarker.setup();

  // Render loop.
  while(!glfwWindowShouldClose(window)) {

    // Tell OpenGL to clean off the canvas.
    GLCALL(glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT));

    // Update the light uniforms.
    light.updateShaderUniforms();

    // Handle input to move the camera.
    unsigned int key = input.handle();
    if (key == UP)
      camera.inputMoveUp();
    else if (key == DOWN)
      camera.inputMoveDown();
    else if (key == LEFT)
      camera.inputMoveLeft();
    else if (key == RIGHT)
      camera.inputMoveRight();
    camera.updateShaderUniforms();

    // Draw the floor, and the rabbit.
    r_floor.draw();
    r_rabbit.draw();
    r_rabbit.getTransform()->rotateYDegrees(0.01);

    // Make the light roll out of the scene to test lighting.
    glm::vec3 lightPosition = light.getWorldLocation();
    //lightPosition.x += 0.01f;
    //lightPosition.y -= 0.01f;
    light.setWorldLocation(lightPosition);

    // Draw the marker of the light source's location.
    lightmarker.draw();

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





