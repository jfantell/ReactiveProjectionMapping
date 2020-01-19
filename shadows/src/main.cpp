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
#include "vertexbuffer.h"
#include "indexbuffer.h"
#include "mesh.h"
#include "camera.h"
#include "transform.h"
#include "pointlight.h"

#include "app_state.h"

#include "r_realsense.h"
#include "inputhandler.h"

#include <librealsense2/rs.hpp>

using namespace std;
int textureMode;
const char *textureImage;

void CheckOpenGLError(const char *stmt, const char *fname, int line) {
    GLenum err = glGetError();
    if (err != GL_NO_ERROR) {
        printf("OpenGL error %08x, at %s:%i - for %s\n", err, fname, line, stmt);
        exit(-1);
    }
}

int main(int argc, const char *argv[]) {
    // start GL context and O/S window using the GLFW helper library
    // Ensure that the correct number of command line arguments have been entered
    if (argc < 2) {
        cout
                << "Please provide the following required input arguments: Mode [Texture Path] "
                << endl;
        return 1;
    }

    // Get the texture mode: either depth or color
    // Both depth and color frames are generated by
    // the realsense camera
    textureMode = atoi(argv[1]);
    cout << "Texture Mode: " << textureMode << endl;
    if (textureMode == Image) {
        if (argc == 3) {
            textureImage = argv[2];
            cout << "Texture Image File Path: " << textureImage << endl;
        } else {
            cout << "Since you have selected mode 2, you must supply an image file path for texture mapping" << endl;
            return 1;
        }
    }

    if (!glfwInit()) {
        fprintf(stderr, "ERROR: could not start GLFW3\n");
        return 1;
    }

    // uncomment these lines if on Apple OS X
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    GLFWwindow *window = glfwCreateWindow(WIDTH, HEIGHT, "ECSE 4750", NULL, NULL);
    if (!window) {
        fprintf(stderr, "ERROR: could not open window with GLFW3\n");
        glfwTerminate();
        return 1;
    }

    int width, height;
    glfwGetWindowSize(window, &width, &height);
    glfwMakeContextCurrent(window);

    // Record with and height in global variable
    windowState.height = height;
    windowState.width = width;

    // start GLEW extension handler
    glewExperimental = GL_TRUE;
    glewInit();

    // get version info
    const GLubyte *renderer = glGetString(GL_RENDERER); // get renderer string
    const GLubyte *version = glGetString(GL_VERSION); // version as a string
    printf("Renderer: %s\n", renderer);
    printf("OpenGL version supported %s\n", version);

    // tell GL to only draw onto a pixel if the shape is closer to the viewer
    GLCALL(glEnable(GL_DEPTH_TEST)); // enable depth-testing
    GLCALL(glDepthFunc(GL_LESS)); // depth-testing interprets a smaller value as "closer"

    /* ******** Set up shaders here ************ */

    std::string shadowVertexShader_path = "../shaders/vertex_shadow.shader";
    std::string defaultVertexShader_path = "../shaders/vertex_render.shader";
    std::string shadowFragmentShader_path = "../shaders/frag_shadow.shader";
    std::string defaultFragmentShader_path = "../shaders/frag_render.shader";
    Shader shadowVertexShader(GL_VERTEX_SHADER);
    Shader defaultVertexShader(GL_VERTEX_SHADER);
    Shader shadowFragmentShader(GL_FRAGMENT_SHADER);
    Shader defaultFragmentShader(GL_FRAGMENT_SHADER);
    shadowVertexShader.loadShaderSource(shadowVertexShader_path);
    defaultVertexShader.loadShaderSource(defaultVertexShader_path);
    shadowFragmentShader.loadShaderSource(shadowFragmentShader_path);
    defaultFragmentShader.loadShaderSource(defaultFragmentShader_path);
    shadowVertexShader.compile();
    defaultVertexShader.compile();
    shadowFragmentShader.compile();
    defaultFragmentShader.compile();
    GLuint shadowProgram = buildShaderProgram(shadowVertexShader, shadowFragmentShader);
    GLuint defaultProgram = buildShaderProgram(defaultVertexShader, defaultFragmentShader);

    /* *********** End Shader code ************* */


    // Set up things that need to be updated in the loop.

    // Create our camera.
    Camera camera = Camera(window, defaultProgram,glm::vec3(0,0,0), glm::vec3(0, 0, 1),
                           glm::vec3(0,-1,0));
    InputHandler::set_camera(&camera);


    //Set up callbacks
    glfwSetWindowSizeCallback(window,  InputHandler::window_size_callback);
    glfwSetMouseButtonCallback(window,InputHandler::mouse_button_callback);
    glfwSetScrollCallback(window,InputHandler::scroll_callback);
    glfwSetCursorPosCallback(window,InputHandler::cursor_pos_callback);
    glfwSetKeyCallback(window,InputHandler::key_callback);

    // Initialize the point light container class.
    PointLight light = PointLight(defaultProgram);
    light.setAmbientColor(glm::vec3(1.0f, 1.0f, 1.0f));
    light.setAmbientStrength(0.1f);
    light.setDiffuseStrength(1.2f);
    light.setSpecularStrength(0.5f);


#ifdef REALSENSE
    // If configured to use the realsense, set it up!
     // Create the r_ (renderable) handler.
     r_Realsense r_realsense = r_Realsense(defaultProgram, &camera);

     // add shadow shader to r_realsense
     r_realsense.add_shadow_shader(shadowProgram);
     r_realsense.add_light(&light);

     // finalize the setup and bake the renderable.
     r_realsense.setup();

     //Initialize the RealSense Camera
     rs2::pipeline pipe;
     rs2::config cfg;

     // Realsense initialization constants go here.
     cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
     cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);

     rs2::pipeline_profile profile = pipe.start(cfg);
#endif

    // Set up variables to rotate light source
    int tickCounter = 0;
    float degreesPerTick = .1;
    float lightRadius = 5;

    // Render loop.
    while (!glfwWindowShouldClose(window)) {

        glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);

        camera.reset_aspect_ratio(); // Listen for window updates within camera.

        // Update the light uniforms.
        light.updateShaderUniforms();
        camera.updateShaderUniforms();

#ifdef REALSENSE
        // If we happen to be on the very first loop, or have an update queued: get a new frame.
          if (tickCounter == 0 || windowState.update_realsense) {
            if (windowState.update_realsense) windowState.update_realsense = false;

            auto frames = pipe.wait_for_frames();
            if (!frames) {
              cerr << "Error occured while attempting to get camera frames" << endl;
              return 1;
            }
            // draw(rs2::frameset &f) updates the renderable's mesh and process all the vertex data.
            r_realsense.draw(frames);
          }
          // draw() simply redraws the previously calculated mesh and does not process any vertex data.
          else r_realsense.draw();
#endif


        // Rotate the light in a 180 degree swing behind the camera.
        glm::vec3 lightPosition =  light.getWorldLocation();
        lightPosition.x = -abs(lightRadius * cos(glm::radians((float) tickCounter * degreesPerTick)));
        lightPosition.z = lightRadius * sin(glm::radians((float) tickCounter * degreesPerTick));
        light.setWorldLocation(lightPosition);


        GLenum err = glGetError();
        if (err != GL_NO_ERROR) {
            printf("OpenGL error %08x", err);
        }

        tickCounter += 1;

        // update other events like input handling
        glfwPollEvents();
        // put the stuff we've been drawing onto the display
        glfwSwapBuffers(window);
    }

    GLenum err = glGetError();
    if (err != GL_NO_ERROR) {
        printf("OpenGL error %08x", err);
    }

    // close GL context and any other GLFW resources
    glfwTerminate();
    return 0;
}