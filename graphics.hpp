#ifndef graphics_h
#define graphics_h

#include <GLUT/glut.h>
#include "transform.hpp"
#include "types.hpp"

using namespace mini3d::math;
using namespace mini3d::physics;

void setColor(GLint color) {
    float c[4] = {
        ((color >> 24) & 0xFF) / 255.0f,
        ((color >> 16) & 0xFF) / 255.0f,
        ((color >>  8) & 0xFF) / 255.0f,
        ((color >>  0) & 0xFF) / 255.0f};

    glColor3f(c[0], c[1], c[2]);
}

void applyTransform(Transform transform) {
    Vec3 position = transform.pos;
    Ternion rotation = transform.rot;

    glTranslatef(position.x, position.y, position.z);
    float angleRadians = rotation.GetAngle();
    GLfloat angle = angleRadians * 180.0f / 3.1416f;
    glRotatef(angle, rotation.x * angleRadians, rotation.y * angleRadians, rotation.z * angleRadians);
    glScalef(transform.scale.x, transform.scale.y, transform.scale.z);
}

void drawSphere(Vec3 position, Ternion orientation, float radius, GLint color) {
    setColor(color);
    glPushMatrix();
        glTranslatef(position.x, position.y, position.z);
        float angleRadians = orientation.GetAngle();
        GLfloat angle = angleRadians * 180.0f / 3.1416f;
        glRotatef(angle, orientation.x * angleRadians, orientation.y * angleRadians, orientation.z * angleRadians);
        glutSolidSphere(radius, 12, 12);
    glPopMatrix();
}

void drawPoint(Transform transform, GLint color, const Vec3& point, GLfloat size) {
    glPointSize(size);
    glDisable(GL_LIGHTING);
    setColor(color);
    glPushMatrix();
        applyTransform(transform);
        glBegin(GL_POINTS);
            glVertex3fv(point);
        glEnd();
    glPopMatrix();
    glEnable(GL_LIGHTING);
}

void drawPoint(const Vec3& point, GLint color, GLfloat size) {
    drawPoint(Transform::Identity(), color, point, size);
}


void drawLine(Vec3 v0, Vec3 v1, GLint color, float width) {
    glDisable(GL_LIGHTING);
    setColor(color);
    glLineWidth(width);
    glBegin(GL_LINES);
        glVertex3fv(v0);
        glVertex3fv(v1);
    glEnd();
    glEnable(GL_LIGHTING);
}

void drawLine(Vec3 v0, Vec3 v1, GLint color) {
    drawLine(v0, v1, color, 2.0f);
}

void drawFace(const Hull &hull, int index) {
    const auto& faceVertices = hull.faceVertices[index];
    const Face& face = hull.faces[index];
    glNormal3fv(face.normal);

    glEnable(GL_LIGHTING);
    glBegin(GL_POLYGON);
        for (int i = 0; i < faceVertices.size(); i++) {
            glVertex3fv(hull.vertices[faceVertices[i]]);
        }
    glEnd();

    glDisable(GL_LIGHTING);
    glLineWidth(2.0f);
    glBegin(GL_LINES);
        setColor(0xFFFFFFFF);
        for (int i = 0; i < faceVertices.size(); i++) {
            glVertex3fv(hull.vertices[faceVertices[i]]);
            glVertex3fv(hull.vertices[faceVertices[(i + 1) % faceVertices.size() ]]);
        }
    glEnd();
}

void drawFace(const Hull &hull, int index, Transform &transform, GLint color, float width) {
    setColor(color);
    glLineWidth(width);

    const auto& faceVertices = hull.faceVertices[index];
    const Face& face = hull.faces[index];
    
    glBegin(GL_LINES);
        glNormal3fv(transform.rotate(face.normal));
        for (int i = 0; i < faceVertices.size(); i++) {
            glVertex3fv(transform * hull.vertices[faceVertices[i]]);
        }
    glEnd();
}


void draw(const Body &body) {
    glPushMatrix();
        applyTransform(body.transform);
        for (int i = 0; i < body.hull.faceVertices.size(); ++i) {
            unsigned int color = 0x80A0D0FF; //body.isAwake ? body.sleepTimer == 0.0f ? 0x80A0D0FF : 0x405070FF : 0xC0C0C0FF;
            
            setColor(color);
            drawFace(body.hull, i);
        }
    glPopMatrix();
}

#endif
