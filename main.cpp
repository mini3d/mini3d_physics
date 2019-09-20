
// Copyright (c) <2012> Daniel Peterson
// This file is part of Mini3D <www.mini3d.org>
// It is distributed under the MIT Software License <www.mini3d.org/license.php>

#include <vector>

#include <SFML/Graphics.hpp>
#include "external/imgui/imgui.h"
#include "external/imgui/imgui-SFML.h"
#include "transform.hpp"
#include "geometry.hpp"
#include "physics.hpp"
#include "graphics.hpp"
#include <cstdlib>
#include <chrono>
#include <thread>
#include <functional>
#include <random>
#include <iostream>

using namespace std;
using namespace mini3d::math;
using namespace mini3d::physics;

int xOld = 0;
int yOld = 0;

bool currentBoxIndex = false;
bool paused = false;

Physics physics;

struct Camera {
    Vec3 pos;
    Vec3 target;
    float fov;

    int windowX;
    int windowY;

    float rotationX = 1.0f;
    float rotationZ = -1.0f;
    float distance = 10;
    float aspect;

    float getDistance() { return (pos - target).Length(); }
    Vec3 getRotationZ() { return (pos - target).Normalized().Cross(Vec3(1,0,0)).Length(); };
    Vec3 getRotationY() { return (pos - target).Normalized().Cross(Vec3(0,0,1)).Length(); };
    
    void setViewAndProjectionMatrix() {
        pos.x = target.x + distance * sin(rotationX) * cos(rotationZ);
        pos.y = target.y + distance * sin(rotationX) * sin(rotationZ);
        pos.z = target.z + distance * cos(rotationX);

        aspect = (GLdouble)windowX / (GLdouble)windowY;

        glViewport(0, 0, windowX, windowY);
        gluPerspective(fov / 2, aspect, 0.5, 200);
        gluLookAt(pos.x, pos.y, pos.z, target.x, target.y, target.z, 0, 0, 1);
    }
    
    Vec3 getViewDirection()                                 { return (target - pos).Normalized(); }
    
    Ray getPickingRayFromScerenCoords(const int x, const int y) {
        Vec3 viewDir = getViewDirection();
        Vec3 tangent = viewDir.Cross(Vec3(0,0,1)).Normalized();
        Vec3 binormal = viewDir.Cross(tangent);
        
        float xCoord = 1.343f * (x - windowX / 2.0f) / (float)windowX * aspect;
        float yCoord = 1.343f * (y - windowY / 2.0f) / (float)windowY;
        Vec3 vp = xCoord * tangent + yCoord * binormal;

        float d = tan(fov / 2.0f);

        const Vec3 pointOriginScreenSpace = (viewDir * d + vp);
        return { pos + pointOriginScreenSpace, (pointOriginScreenSpace).Normalized() };
    }
    
};

Camera camera = { Vec3(0, -10.0f, 0), Vec3(0,0,1), 80, 1920, 1200};

Body* getBodyUnderMouse(Vec3 &point) {
    const Ray ray = camera.getPickingRayFromScerenCoords(xOld, yOld);
    return physics.rayPicking(point, ray);
}

struct Callback : Physics::CollisionCallbacks {
    bool contactStarted(const Body* a, const Body* b) const {
//        printf("Collision started: %p, %p\n", a, b);
        return true;
    }
    
    void contactEnded(const Body* a, const Body* b) const {
//        printf("Collision ended: %p, %p\n", a, b);
    }
};

Callback callback;


bool my_tool_active = false;
float my_color[4];

void loadEmptyScene() {
    physics.clearAll();

    Body ground = Box(Vec3(10.0f, 10.0f, 0.5f));
    ground.transform.pos = Vec3( 0, 0, -0.25f);
    ground.invMass = 0.0f;
    ground.invInertia = 0;
    
    physics.add(ground);
}

void loadStackScene() {
    loadEmptyScene();

    int height = 8;
    for (int j = 0; j < height; ++j) {
        Body box = Box(Vec3(0.5f));
        box.transform.pos = Vec3(0, 0, 0.25f + j * 0.52f);
        physics.add(box);
    }
}

void loadPyramidScene() {
    loadEmptyScene();

    int height = 5;
    for (int j = 0; j < height; ++j) {
        for (int i = 0; i <= j; ++i) {
            Body box = Box(Vec3(0.5f));
            box.transform.pos = Vec3(0.52f * (i - 0.5f * j), 0, -0.25f + (height - j) * 0.52f);
            physics.add(box);
        }
    }
}

void loadJengaScene() {
    loadEmptyScene();

    int height = 8;

    for (int j = 0; j < height; j += 2) {
        Body box = Box(Vec3(0.2f, 1.2f, 0.2f));
        box.transform.pos = Vec3(-0.5f, 0, 0.11f + 0.22f * j);
        physics.add(box);

        Body box2 = Box(Vec3(0.2f, 1.2f, 0.2f));
        box2.transform.pos = Vec3(0.5f, 0, 0.11f + 0.22f * j);
        physics.add(box2);
    }

    for (int j = 0; j < height; j += 2) {
        Body box = Box(Vec3(1.2f, 0.2f, 0.2f));
        box.transform.pos = Vec3(0.0f, -0.5f, 0.32f + 0.22f * j);
        physics.add(box);

        Body box2 = Box(Vec3(1.2f, 0.2f, 0.2f));
        box2.transform.pos = Vec3(0.0f, 0.5f, 0.32f + 0.22f * j);
        physics.add(box2);
    }
}

bool enableSleeping;

std::vector<std::string> items = { "empty", "stack", "pyramid", "jenga" };
std::vector<std::function<void()>> functions = { loadEmptyScene, loadStackScene, loadPyramidScene, loadJengaScene };
std::string current_item = items[0];

void displayImgui() {
    // Create a window called "My First Tool", with a menu bar.
    ImGui::SetNextWindowContentWidth(400);
    ImGui::Begin("My First Tool", &my_tool_active, ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoBackground);
    ImGui::SetWindowPos(ImVec2(0, 0));
    
    ImGui::SetNextWindowContentWidth(400);
    if (ImGui::BeginCombo("##combo", current_item.c_str())) {
        for (int n = 0; n < items.size(); n++) {
            bool is_selected = (current_item == items[n]); // You can store your selection however you want, outside or inside your objects
            if (ImGui::Selectable(items[n].c_str(), is_selected)) {
                current_item = items[n];
                functions[n]();
            } if (is_selected) {
                ImGui::SetItemDefaultFocus();
            }
        }
        ImGui::EndCombo();
    }
    ImGui::SameLine();
    if (ImGui::Button("Pause")) {
        // TODO: Implement
    }
    ImGui::SameLine();
    if (ImGui::Button("Step")) {
        // TODO: Implement
    }
    ImGui::SameLine();
    if (ImGui::Checkbox("Sleeping", &enableSleeping)) {
        // TODO: Implement
    }

    ImGui::End();
}

void display() {

    glPolygonMode(GL_FRONT, GL_FILL);
    glCullFace(GL_NONE);
    glEnable(GL_DEPTH_TEST);
    glClearColor(0.3f, 0.3f, 0.3f, 1);
    glClearDepth(1.0);


    glLineWidth(2.0f);
    glPointSize(5.0f);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    
    camera.setViewAndProjectionMatrix();
    
    glMatrixMode(GL_MODELVIEW);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();

    glEnable(GL_COLOR_MATERIAL);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
    
    GLfloat position[] = { 3.0f, -4.0f, 5.0f, 1.0f };
    glLighti(GL_LIGHT0, GL_AMBIENT, 0xFFFFFFFF);
    glLighti(GL_LIGHT0, GL_DIFFUSE, 0xFFFFFFFF);
    glLighti(GL_LIGHT0, GL_SPECULAR, 0xFFFFFFFF);
    glLightfv(GL_LIGHT0, GL_POSITION, position);

    auto start = std::chrono::high_resolution_clock::now();
    physics.step(1.0f, callback);
    auto end = std::chrono::high_resolution_clock::now();
    auto microseconds = std::chrono::duration_cast<std::chrono::microseconds>(end-start).count();
    
    cout << "Frame time: " << microseconds << endl;
    for (auto& body : physics.bodies) {
        draw(body);
    }

    //physics.drawManifolds();

    glFlush();
}

// DRAWING
sf::RenderWindow window;

bool mouseLeftButton = false;

// EVENTS
void doEvents() {
    sf::Event event;
    while (window.pollEvent(event)) {
        ImGui::SFML::ProcessEvent(event);

        if (event.type == sf::Event::Closed) {
            window.close();
        } else if (event.type == sf::Event::KeyPressed) {
        /*
            if (event.key.code == sf::Keyboard::W) {
                physics.bodies[currentBoxIndex].transform.pos += UP * 0.1f;
            } else if (event.key.code == sf::Keyboard::X) {
                physics.bodies[currentBoxIndex].transform.pos += DOWN * 0.1f;
            } else if (event.key.code == sf::Keyboard::A) {
                physics.bodies[currentBoxIndex].transform.pos += LEFT * 0.1f;
            } else if (event.key.code == sf::Keyboard::D) {
                physics.bodies[currentBoxIndex].transform.pos += RIGHT * 0.1f;
            } else if (event.key.code == sf::Keyboard::R) {
                physics.bodies[currentBoxIndex].transform.pos += FORWARD * 0.1f;
            } else if (event.key.code == sf::Keyboard::V) {
                physics.bodies[currentBoxIndex].transform.pos += BACK * 0.1f;
            } else if (event.key.code == sf::Keyboard::Num1) {
                physics.bodies[currentBoxIndex].transform.rot *= Ternion(0.1f, 0, 0);
            } else if (event.key.code == sf::Keyboard::Num3) {
                physics.bodies[currentBoxIndex].transform.rot *= Ternion(-0.1f, 0, 0);
            } else if (event.key.code == sf::Keyboard::Q) {
                physics.bodies[currentBoxIndex].transform.rot *= Ternion(0, 0.1f, 0);
            } else if (event.key.code == sf::Keyboard::E) {
                physics.bodies[currentBoxIndex].transform.rot *= Ternion(0, -0.1f, 0);
            } else if (event.key.code == sf::Keyboard::Z) {
                physics.bodies[currentBoxIndex].transform.rot *= Ternion(0, 0, 0.1f);
            } else if (event.key.code == sf::Keyboard::C) {
                physics.bodies[currentBoxIndex].transform.rot *= Ternion(0, 0, -0.1f);
            } else */if (event.key.code == sf::Keyboard::B) {
                currentBoxIndex = !currentBoxIndex;
            } else if (event.key.code == sf::Keyboard::P) {
                physics.setPaused(paused = !paused);
            } else if (event.key.code == sf::Keyboard::S) {
                if (paused) {
                    physics.setPaused(false);
                    physics.step(1.0f, callback);
                    physics.setPaused(true);
                }
            } else if (event.key.code == sf::Keyboard::Y) {
                if (physics.joints.size() > 0) {
                    physics.joints.clear();
                } else {
                    Vec3 pickPoint;
                    Body* body = getBodyUnderMouse(pickPoint);
                    if (body != nullptr) {
                        MouseConstraint mouseConstraint;
                        mouseConstraint.body = body;
                        mouseConstraint.contactPoint = (-body->transform) * pickPoint;
                        mouseConstraint.mousePoint = pickPoint;
                        physics.joints.push_back(mouseConstraint);
                    }
                }
                break;
            } else if (event.key.code == sf::Keyboard::H) {
                camera.distance *= 0.95f;
            } else if (event.key.code == sf::Keyboard::J) {
                camera.distance *= 1.05f;
            } else if (event.key.code == sf::Keyboard::Comma) {
            } else if (event.key.code == sf::Keyboard::Period) {
            } else if (event.key.code == sf::Keyboard::Num0) {
                std::random_device rng;
                std::mt19937 urng(rng());
                std::uniform_real_distribution<> dist(-0.05f,0.05f);
                Box box(Vec3(0.4f, 0.4f, 0.4f));
                //box.transform.pos = Vec3(0.8f + dist(rng), 0 + dist(rng), 5.0f);
                box.transform.pos = Vec3(0.8f, 0, 5.0f);
                box.velocity = Vec3(0.0f, 0, 0);
                physics.add(box);
            } else if (event.key.code == sf::Keyboard::Num9) {
                Box box(Vec3(0.2f, 2.0f, 0.2f));
                box.transform.pos = Vec3(0.0f, -3.0, 2.0f);
                box.velocity = Vec3(0.0f, 0.2f, -0.05f);
                physics.add(box);
            }

        } else if (event.type == sf::Event::MouseButtonPressed) {
            if (event.mouseButton.button == sf::Mouse::Left) {
                mouseLeftButton = true;
            }
        } else if (event.type == sf::Event::MouseButtonReleased) {
            if (event.mouseButton.button == sf::Mouse::Left) {
                mouseLeftButton = false;
            }
        } else if (event.type == sf::Event::MouseMoved) {
            if (mouseLeftButton) {
                camera.rotationZ -= (event.mouseMove.x - xOld) / 100.0f;
                camera.rotationX -= (event.mouseMove.y - yOld) / 100.0f;
            }

            xOld = event.mouseMove.x;
            yOld = event.mouseMove.y;
        } else if (event.type == sf::Event::MouseWheelMoved) {
            camera.distance = pow(camera.distance, 1 - event.mouseWheel.delta * 0.01f);
        } else if (event.type == sf::Event::Resized) {
            camera.windowX = event.size.width;
            camera.windowY = event.size.height;
            glViewport(0, 0, camera.windowX, camera.windowY);
        }
    }
}

int main (int argc, char **argv) {
    loadEmptyScene();

    window.create(sf::VideoMode(camera.windowX, camera.windowY), "Mini3d Physics", sf::Style::Default, sf::ContextSettings(32));
    window.setFramerateLimit(60);
    ImGui::SFML::Init(window);

    sf::Clock deltaClock;
    while (window.isOpen()) {
        doEvents();

        // DRAW
        display();

        // DISPLAY IMGUI
        ImGui::SFML::Update(window, deltaClock.restart());
        displayImgui();

        // DISPLAY
        glClear(GL_DEPTH_BUFFER_BIT);
        ImGui::SFML::Render(window);
        window.display();

        // SLEEP
        sf::sleep(sf::seconds(0.01f));
    };

    ImGui::DestroyContext();

    return 0;
}














