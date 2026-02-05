#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <iostream>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <chrono>
#include <cmath>

struct IMUData {
    float roll, pitch, yaw;
};

// Thread-safe queue
std::queue<IMUData> imuQueue;
std::mutex mtx;
std::condition_variable cv;
bool done = false;

// Producer: generates IMU data
void imuProducer() {
    for (int i = 0; i < 1000; ++i) {
        IMUData data = {
            .roll = std::sin(i * 0.01f),
            .pitch = std::cos(i * 0.01f),
            .yaw = std::sin(i * 0.005f)
        };

        {
            std::lock_guard<std::mutex> lock(mtx);
            imuQueue.push(data);
        }
        cv.notify_one();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    {
        std::lock_guard<std::mutex> lock(mtx);
        done = true;
    }
    cv.notify_one();
}

// Consumer: non-blocking poll for latest IMU data
bool getLatestIMU(IMUData& out) {
    std::lock_guard<std::mutex> lock(mtx);
    if (!imuQueue.empty()) {
        while (imuQueue.size() > 1) imuQueue.pop();  // Keep only most recent
        out = imuQueue.front();
        imuQueue.pop();
        return true;
    }
    return false;
}

// 3D axis drawing
void drawAxes3D(const IMUData& data) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);

    glm::mat4 projection = glm::perspective(glm::radians(45.0f), 800.0f / 600.0f, 0.1f, 100.0f);
    glm::mat4 view = glm::lookAt(
        glm::vec3(0.0f, 0.0f, 5.0f),  // camera position
        glm::vec3(0.0f, 0.0f, 0.0f),  // target
        glm::vec3(0.0f, 1.0f, 0.0f)   // up vector
    );
    glm::mat4 model = glm::mat4(1.0f);

    // Apply rotations from IMU
    model = glm::rotate(model, data.yaw, glm::vec3(0, 1, 0));
    model = glm::rotate(model, data.pitch, glm::vec3(1, 0, 0));
    model = glm::rotate(model, data.roll, glm::vec3(0, 0, 1));

    glm::mat4 mvp = projection * view * model;
    glLoadMatrixf(glm::value_ptr(mvp));

    glRotatef(data.yaw * 180 / M_PI,   0, 1, 0); // Yaw
    glRotatef(data.pitch * 180 / M_PI, 1, 0, 0); // Pitch
    glRotatef(data.roll * 180 / M_PI,  0, 0, 1); // Roll

    glBegin(GL_LINES);
    // X axis (red)
    glColor3f(1, 0, 0); glVertex3f(0, 0, 0); glVertex3f(1, 0, 0);
    // Y axis (green)
    glColor3f(0, 1, 0); glVertex3f(0, 0, 0); glVertex3f(0, 1, 0);
    // Z axis (blue)
    glColor3f(0, 0, 1); glVertex3f(0, 0, 0); glVertex3f(0, 0, 1);
    glEnd();
}

int main() {
    if (!glfwInit()) return -1;

    GLFWwindow* window = glfwCreateWindow(800, 600, "IMU Visualizer (Live Stream)", nullptr, nullptr);
    if (!window) {
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);

    glEnable(GL_DEPTH_TEST);
    glClearColor(0.1f, 0.1f, 0.1f, 1.0f);

    std::thread imuThread(imuProducer);
    IMUData currentData{0, 0, 0};

    while (!glfwWindowShouldClose(window)) {
        if (getLatestIMU(currentData)) {
            drawAxes3D(currentData);
        }

        glfwSwapBuffers(window);
        glfwPollEvents();
        std::this_thread::sleep_for(std::chrono::milliseconds(16));  // ~60 FPS
    }

    imuThread.join();
    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}
