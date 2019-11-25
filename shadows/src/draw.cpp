//
// Created by John Fantell on 11/25/19.
//

#include "draw.h"

Draw::Draw(GLuint shaderShadowProgramID, GLuint shaderPrimaryProgramID, Camera* camera, PointLight* light){
    _shadowShaderId = shaderShadowProgramID;
    _shaderId = shaderPrimaryProgramID;
    _camera = camera;
    _light = light;
}

void Draw::setup_shadow_buffers(){
    glGenFramebuffers(1, &_shadowBuffer);

    glGenTextures(1, &_shadowTex);
    glBindTexture(GL_TEXTURE_2D, _shadowTex);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT32,
                 windowState.width, windowState.height, 0, GL_DEPTH_COMPONENT, GL_FLOAT, 0);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_MODE, GL_COMPARE_REF_TO_TEXTURE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_FUNC, GL_LEQUAL);

    // may reduce shadow border artifacts
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
}

void Draw::add_models(const std::vector<Renderable *> renderable_models){
    _renderable_models = renderable_models;
    _num_models = _renderable_models.size();
}

void Draw::pass_one(Renderable * model) {
    glUseProgram(_shadowShaderID);

    glm::mat4 shadowMVP1 = _lightPmatrix * _lightVmatrix * model->getTransform()->getModelMatrix();
    _shadowMVPId = glGetUniformLocation(_shadowShaderID, "Shadow_MVP");
    glUniformMatrix4fv(_shadowMVPId, 1, GL_FALSE, glm::value_ptr(shadowMVP1));

    GLuint *vbo = model->getVBO();

    //Vertex Buffer
    glBindBuffer(GL_ARRAY_BUFFER, vbo[0]);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);
    glEnableVertexAttribArray(0);

    glClear(GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);

    //Element Array Buffer
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vbo[4]);
    glDrawElements(GL_TRIANGLES,model->get_num_vertices(),GL_UNSIGNED_INT, 0);
}

void Draw::pass_two(Renderable * model) {
    glUseProgram(_shaderId);

    _shaderModelId = glGetUniformLocation(_shaderId, "Model");
    _shaderViewId = glGetUniformLocation(_shaderId, "View");
    _shaderModelViewId = glGetUniformLocation(_shaderId, "ModelView");
    _shaderPerpectiveId = glGetUniformLocation(_shaderId, "Perpective");
    _shaderUseTextureId = glGetUniformLocation(_shaderId, "Use_Text");
    _shadowMVPId = glGetUniformLocation(_shaderId, "Shadow_MVP");
    _invTrMVId = glGetUniformLocation(_shaderId, "InvTr_MV");

    glm::mat4 shadowMVP2 = _b * _lightPmatrix * _lightVmatrix * model->getTransform()->getModelMatrix();
    glm::mat4 mvMat = _camera->getView() * model->getTransform()->getModelMatrix();
    glm::mat4 invTrMat = glm::transpose(glm::inverse(mvMat));

    glUniformMatrix4fv(_shaderModelId, 1, GL_FALSE, glm::value_ptr( model->getTransform()->getModelMatrix()));
    glUniformMatrix4fv(_shaderViewId, 1, GL_FALSE, glm::value_ptr( _camera->getView()));
    glUniformMatrix4fv(_shaderModelViewId, 1, GL_FALSE, glm::value_ptr( mvMat));
    glUniformMatrix4fv(_shaderPerpectiveId, 1, GL_FALSE, glm::value_ptr(_camera->getProjection()));
    glUniformMatrix4fv(_shadowMVPId, 1, GL_FALSE, glm::value_ptr(shadowMVP2));
    glUniformMatrix4fv(_invTrMVId, 1, GL_FALSE, glm::value_ptr(invTrMat));
    glUniform1i(_shaderUseTextureId,windowState.useTexture);

    GLuint *vbo = model->getVBO();

    //Vertex buffer
    glBindBuffer(GL_ARRAY_BUFFER,vbo[0]);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE,0,0);
    glEnableVertexAttribArray(0);

    //Norm buffer
    glBindBuffer(GL_ARRAY_BUFFER, vbo[1]);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, 0);
    glEnableVertexAttribArray(1);

    //Texture Buffer
    glBindBuffer(GL_ARRAY_BUFFER, vbo[2]);
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 0, 0);
    glEnableVertexAttribArray(2);

    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, model->get_textureID());
    glUniform1i(glGetUniformLocation(_shaderId, "textureimage"), 1);

    //Color buffer
    glBindBuffer(GL_ARRAY_BUFFER, vbo[3]);
    glVertexAttribPointer(3, 3, GL_FLOAT, GL_FALSE, 0, 0);
    glEnableVertexAttribArray(3);

    glClear(GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);

    // Element array buffer
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vbo[4]);
    glDrawElements(GL_TRIANGLES,model->get_num_vertices(),GL_UNSIGNED_INT, 0);
}

void Draw::draw() {
    // Set up light view matrix and perpective matrix
    glm::vec3 up(0.0f, -1.0f, 0.0f);
    glm::vec3 current_loc_light = _light->getWorldLocation();
    glm::vec3 current_loc_model = glm::vec3(0, 0, 0);
    _lightVmatrix = glm::lookAt(current_loc_light, current_loc_model, up);
    _lightPmatrix = glm::perspective(windowState.get_fov(), windowState.get_aspect_ratio(), windowState.get_z_near(),
                                    windowState.get_z_far());

    glBindFramebuffer(GL_FRAMEBUFFER, _shadowBuffer);
    glFramebufferTexture(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, _shadowTex, 0);

    glDrawBuffer(GL_NONE);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_POLYGON_OFFSET_FILL);    // for reducing
    glPolygonOffset(2.0f, 4.0f);        //  shadow artifacts

    for(int i=0; i<_num_models; i++){
        pass_one(_renderable_models[i]);
    }

    glDisable(GL_POLYGON_OFFSET_FILL);    // artifact reduction, continued

    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, _shadowTex);
    glUniform1i(glGetUniformLocation(_shadowShaderID, "shadowTex"), 0);

    glDrawBuffer(GL_FRONT);

    for(int i=0; i<_num_models; i++){
        pass_two(_renderable_models[i]);
    }
}