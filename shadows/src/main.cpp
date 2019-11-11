#include "GL/glew.h"
#include "GLFW/glfw3.h"
#include <stdio.h>
#include <stdlib.h>

#include "shader.h"
#include "cubedata.h"
#include "vertexbuffer.h"

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

  GLFWwindow* window = glfwCreateWindow(640, 480, "Hello Triangle", NULL, NULL);
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
  glEnable(GL_DEPTH_TEST); // enable depth-testing
  glDepthFunc(GL_LESS); // depth-testing interprets a smaller value as "closer"

  /* ******** Set up shaders here ************ */

  Shader vertexShader(GL_VERTEX_SHADER);
  Shader fragmentShader(GL_FRAGMENT_SHADER);

  vertexShader.loadShaderSource("../src/shaders/VERTEX.shader");
  fragmentShader.loadShaderSource("../src/shaders/FRAG.shader");

  GLuint vs = vertexShader.compile();
  GLuint fs = fragmentShader.compile();

  GLuint program = glCreateProgram();
  glAttachShader(program, fs);
  glAttachShader(program, vs);
  glLinkProgram(program);
  glUseProgram(program);

  /* *********** End Shader code ************* */


  float points[] = {
      0.0f,  0.5f,  0.0f,
      0.5f, -0.5f,  0.0f,
      -0.5f, -0.5f,  0.0f
  };

  float colors[] = {
      0.583f,  0.771f,  0.014f,
      0.609f,  0.115f,  0.436f,
      0.327f,  0.483f,  0.844f
  };

  VertexBuffer vb;
  vb.genBuffer();
  vb.bind();
  // Push 3 floats for one vertex xyz
  vb.getLayout()->push<float>(3);
  // Push 3 floats for one vertex rgb
  vb.getLayout()->push<float>(3);

  // Push pointers in order
  vb.pushPointer((void *) g_vertex_buffer_data);
  vb.pushPointer((void *) g_color_buffer_data);
  // Interleave the data according to the layout
  vb.interleave(3*2*6);

  // Do glBufferData manually for now
  glBufferData(GL_ARRAY_BUFFER, sizeof(g_vertex_buffer_data) + sizeof(g_color_buffer_data), vb.getBufferPtr(), GL_STATIC_DRAW);

  // Create vertex array, the vehicle from cpu land to graphics land
  GLuint vao = 0;
  glGenVertexArrays(1, &vao);
  glBindVertexArray(vao);
  glEnableVertexAttribArray(0);
  glEnableVertexAttribArray(1);

  // Render loop.
  while(!glfwWindowShouldClose(window)) {

    /* ******** Drawing code starts here ************ */

    // Tell OpenGL to clean off the canvas.
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Tell OpenGL we're using the vertex array,
    // We're going to use it as the vehicle for buffers
    // from application space to hardware space.
    glBindVertexArray(vao);

    // Remind OpenGl of how we like our data, every single draw call.
    glBindBuffer(GL_ARRAY_BUFFER, vb.getBufferId());

    // Tell OpenGl about our floats XYZ, they come first, as specified in layout.
    glVertexAttribPointer(0,  // Attribute ZERO (vertices)
                          3, // 3 floats per vertex
                          GL_FLOAT, // type is float
                          GL_FALSE, // they are not normalized
                          6 * sizeof(float), // stride is size of one whole vertex.
                          NULL // No offset into the buffer
    );

    // Set up an offset because C++ is picky with pointers.
    unsigned char offset = 3 * sizeof(float);

    // Tell OpenGl about our colors RGB, they are second, as specified in the layout.
    glVertexAttribPointer(1,  // Attribute ONE (color)
                          3, // 3 floats per vertex
                          GL_FLOAT, // type is float
                          GL_FALSE, // they are not normalized
                          6 * sizeof(float), // stride is size of one entire vertex.
                          (GLvoid*) 12 // offset into the buffer by sizeof(float) * 3.
    );

    // draw points 0-3 from the currently bound VAO with current in-use shader
    glDrawArrays(GL_TRIANGLES, 0, 3*2*6);


    /* ********* END drawing code *********** */


    // update other events like input handling
    glfwPollEvents();
    // put the stuff we've been drawing onto the display
    glfwSwapBuffers(window);
  }

  // close GL context and any other GLFW resources
  glfwTerminate();
  return 0;
}