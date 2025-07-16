#define NOMINMAX
#include <iostream>
#include <memory>
#include <string>
#include <algorithm>

// OpenGL and GLFW
#include <GL/gl3w.h>
#include <GLFW/glfw3.h>

// ImGui
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

// Our classes
#include "ImageProcessor.h"
#include "PerformanceTimer.h"

// Global variables
std::unique_ptr<ImageProcessor> imageProcessor;
std::unique_ptr<PerformanceTimer> timer;

// GUI state variables
static char imagePathBuffer[256] = "";
static int blurKernelSize = 15;
static float blurSigma = 2.0f;
static int rotationAngle = 0; // 0=90deg, 1=180deg, 2=270deg
static float brightnessAlpha = 1.0f;
static int brightnessBeta = 0;
static int selectedMethod = 0; // 0=Sequential, 1=Pthread, 2=OpenMP
static int threadCount = std::thread::hardware_concurrency(); // Default Available Threads
static int openmpThreadCount = std::thread::hardware_concurrency(); // Default Available Threads

// Performance results display
static std::string performanceResults = "";

// Texture related variables
static GLuint originalTexture = 0;
static GLuint processedTexture = 0;
static bool hasOriginalImage = false;
static bool hasProcessedImage = false;

// Function to convert image to OpenGL texture
GLuint createTextureFromImage(const Image& image) {
    GLuint textureId;
    glGenTextures(1, &textureId);
    glBindTexture(GL_TEXTURE_2D, textureId);
    
    // Set texture parameters
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    
    // Upload image data to texture
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, image.width, image.height, 0, GL_RGB, GL_UNSIGNED_BYTE, image.data.get());
    
    return textureId;
}

// Function to update existing texture
void updateTexture(GLuint textureId, const Image& image) {
    glBindTexture(GL_TEXTURE_2D, textureId);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, image.width, image.height, 0, GL_RGB, GL_UNSIGNED_BYTE, image.data.get());
}

// Function to delete texture
void deleteTexture(GLuint& textureId) {
    if (textureId != 0) {
        glDeleteTextures(1, &textureId);
        textureId = 0;
    }
}

void renderGUI() {
    static bool openLoadPopup = false;
    static bool openSavePopup = false;
    // Menu Bar
    if (ImGui::BeginMainMenuBar()) {
        if (ImGui::BeginMenu("File")) {
            if (ImGui::MenuItem("Load Image")) {
                openLoadPopup = true;
            }
            if (ImGui::MenuItem("Save Image")) {
                openSavePopup = true;
            }
            ImGui::EndMenu();
        }
        ImGui::EndMainMenuBar();
    }
    // === Main Window ===
    ImGui::Begin("OPENMP IMAGE PROCESSOR");
    if (openLoadPopup) {
        ImGui::OpenPopup("Load Image Popup");
        openLoadPopup = false;
    }
    if (openSavePopup) {
        ImGui::OpenPopup("Save Image Popup");
        openSavePopup = false;
    }
    // === Load Popup ===
    if (ImGui::BeginPopupModal("Load Image Popup", NULL, ImGuiWindowFlags_AlwaysAutoResize)) {
        static char popupImagePath[256] = "";

        ImGui::Text("Enter image path to load:");
        ImGui::InputText("##popupImagePath", popupImagePath, sizeof(popupImagePath));
        ImGui::Spacing();

        if (ImGui::Button("Load")) {
            if (strlen(popupImagePath) > 0) {
                if (imageProcessor->loadImage(std::string(popupImagePath))) {
                    deleteTexture(originalTexture);
                    deleteTexture(processedTexture);
                    originalTexture = createTextureFromImage(imageProcessor->getCurrentImage());
                    hasOriginalImage = true;
                    hasProcessedImage = false;
                    performanceResults = "Image loaded successfully!";
                } else {
                    performanceResults = "Failed to load image! Check console for details.";
                }
            } else {
                performanceResults = "Please enter a valid path.";
            }
            ImGui::CloseCurrentPopup();
        }
        ImGui::SameLine();
        if (ImGui::Button("Cancel")) {
            ImGui::CloseCurrentPopup();
        }
        ImGui::EndPopup();
    }

    // === Save Popup ===
    if (ImGui::BeginPopupModal("Save Image Popup", NULL, ImGuiWindowFlags_AlwaysAutoResize)) {
        static char popupSavePath[256] = "output.jpg";

        ImGui::Text("Enter path to save the image:");
        ImGui::InputText("##popupSavePath", popupSavePath, sizeof(popupSavePath));
        ImGui::Spacing();

        if (ImGui::Button("Save")) {
            if (imageProcessor->saveImage(std::string(popupSavePath))) {
                performanceResults += " | Saved successfully!";
            } else {
                performanceResults += " | Save failed!";
            }
            ImGui::CloseCurrentPopup();
        }

        ImGui::SameLine();
        if (ImGui::Button("Cancel")) {
            ImGui::CloseCurrentPopup();
        }   
        ImGui::EndPopup();
    }

    ImGui::BeginChild("SideBarPanel", ImVec2(200, 500), true, ImGuiWindowFlags_NoScrollbar);
    ImGui::Text("Processing Methods");
    ImGui::Separator();
    ImGui::RadioButton("Sequential", &selectedMethod, 0);
    ImGui::RadioButton("Pthread", &selectedMethod, 1);
    ImGui::RadioButton("OpenMP", &selectedMethod, 2);

    // Thread count input
    if (selectedMethod == 1) {
        ImGui::InputInt("Threads", &threadCount);
        threadCount = std::max(1, std::min(threadCount, 10));
        imageProcessor->setThreadCount(threadCount);
    } else if (selectedMethod == 2) {
        ImGui::InputInt("Threads", &openmpThreadCount);
        openmpThreadCount = std::max(1, std::min(openmpThreadCount, 10));
        imageProcessor->setOpenMPThreadCount(openmpThreadCount);
    }

    ImGui::Dummy(ImVec2(0.0f, 15.0f));
    ImGui::Separator();
    // Gaussian Blur Section
    ImGui::Text("Gaussian Blur");
    int oldKernel = blurKernelSize;
    if (ImGui::InputInt("Kernel", &blurKernelSize)) {
        blurKernelSize = std::max(3, std::min(blurKernelSize, 31));
        // Make it odd by rounding *towards* the closest odd number
        if (blurKernelSize % 2 == 0) {
            if (blurKernelSize > oldKernel)
                blurKernelSize += 1; // round up
            else
                blurKernelSize -= 1; // round down
        }
    }
    ImGui::SliderFloat("Sigma", &blurSigma, 0.1f, 10.0f);
    if (ImGui::Button("Apply###Blur")) {
        if (!imageProcessor->getCurrentImage().empty()) {
            auto method = static_cast<ProcessingMethod>(selectedMethod);
            auto blurFunc = [&]() {
                imageProcessor->applyGaussianBlur(imageProcessor->getCurrentImage(), blurKernelSize, blurSigma, method);
            };
            timer->measureExecutionTime(blurFunc);
             if (processedTexture == 0) {
                processedTexture = createTextureFromImage(imageProcessor->getProcessedImage());
            } else {
                updateTexture(processedTexture, imageProcessor->getProcessedImage());
            }
            hasProcessedImage = true;
            performanceResults = "Blur done - " + timer->getFormattedResults();
        }
    }


    ImGui::Dummy(ImVec2(0.0f, 15.0f));
    ImGui::Separator();
    // Rotation section
    ImGui::Text("Image Rotation:");
    const char* rotationItems[] = { "90", "180", "270" };
    ImGui::Combo("Angle", &rotationAngle, rotationItems, 3);
    
    if (ImGui::Button("Apply###Rotation")) {
        if (!imageProcessor->getCurrentImage().empty()) {
            ProcessingMethod method = static_cast<ProcessingMethod>(selectedMethod);
            RotationAngle angle = static_cast<RotationAngle>(rotationAngle);
            auto rotateFunc = [&]() {
                imageProcessor->applyRotation(imageProcessor->getCurrentImage(), angle, method);
            };
            double executionTime = timer->measureExecutionTime(rotateFunc);            
            if (processedTexture == 0) {
                processedTexture = createTextureFromImage(imageProcessor->getProcessedImage());
            } else {
                updateTexture(processedTexture, imageProcessor->getProcessedImage());
            }
            hasProcessedImage = true;
            performanceResults = "Rotation completed - " + timer->getFormattedResults();
        }
    }

    ImGui::Dummy(ImVec2(0.0f, 15.0f));
    ImGui::Separator();
    ImGui::Text("Brightness Adjustment:");
    ImGui::SliderFloat("Alpha", &brightnessAlpha, 0.1f, 3.0f);
    ImGui::SliderInt("Beta", &brightnessBeta, -100, 100);
    
    if (ImGui::Button("Apply###Brightness")) {
        if (!imageProcessor->getCurrentImage().empty()) {
            ProcessingMethod method = static_cast<ProcessingMethod>(selectedMethod);
            auto brightnessFunc = [&]() {
                imageProcessor->adjustBrightness(imageProcessor->getCurrentImage(), 
                                               static_cast<double>(brightnessAlpha), brightnessBeta, method);
            };
            double executionTime = timer->measureExecutionTime(brightnessFunc);
            if (processedTexture == 0) {
                processedTexture = createTextureFromImage(imageProcessor->getProcessedImage());
            } else {
                updateTexture(processedTexture, imageProcessor->getProcessedImage());
            }
            hasProcessedImage = true;
            performanceResults = "Brightness adjustment completed - " + timer->getFormattedResults();
        }
    }
    

    ImGui::Dummy(ImVec2(0.0f, 15.0f));
    ImGui::Separator();
    ImGui::Text("Apply Every Filter");
    if (ImGui::Button("Apply###Everything")) {
        if (!imageProcessor->getCurrentImage().empty()) {
            ProcessingMethod method = static_cast<ProcessingMethod>(selectedMethod);
            RotationAngle angle = static_cast<RotationAngle>(rotationAngle);

            ImageProcessor::PresetParams params {
                imageProcessor->getCurrentImage(),
                blurKernelSize,
                static_cast<double>(blurSigma),
                angle,
                static_cast<double>(brightnessAlpha),
                brightnessBeta,
                method
            };

            auto presetFunc = [&]() {
                return imageProcessor->applyAllFilter(params);
            };

            double executionTime = timer->measureExecutionTime(presetFunc);
            auto results = presetFunc();
            
            // Update processed image texture with the brightness result
            if (processedTexture == 0) {
                processedTexture = createTextureFromImage(results.image);
            } else {
                updateTexture(processedTexture, results.image);
            }
            hasProcessedImage = true;
            
            // Format detailed performance results
            char detailedResults[512];
            snprintf(detailedResults, sizeof(detailedResults), 
                    "Operations Completed - Total: %.2f ms | Blur: %.2f ms | Rotate: %.2f ms | Brightness: %.2f ms",
                    results.totalTime, results.blurTime, results.rotateTime, results.brightnessTime);
            performanceResults = std::string(detailedResults);
        }
    }
    ImGui::EndChild();
        
    // --- Image Preview: Right panel ---
    ImGui::SameLine();
    ImGui::BeginGroup(); // Start right section

    ImGui::Text("Image Preview:");
    ImGui::Separator();

    if (hasOriginalImage || hasProcessedImage) {
        float maxDisplayWidth = 300.0f;
        float displayWidth = maxDisplayWidth;
        float displayHeight = maxDisplayWidth;

        if (hasOriginalImage) {
            const Image& img = imageProcessor->getCurrentImage();
            float aspectRatio = static_cast<float>(img.height) / static_cast<float>(img.width);
            displayHeight = displayWidth * aspectRatio;
        }

        if (hasOriginalImage && hasProcessedImage) {
            ImGui::BeginGroup();
            ImGui::Text("Original");
            ImGui::Image(reinterpret_cast<void*>(originalTexture), ImVec2(displayWidth, displayHeight));
            ImGui::EndGroup();

            ImGui::SameLine();
            
            ImGui::BeginGroup();
            ImGui::Text("Processed");
            ImGui::Image(reinterpret_cast<void*>(processedTexture), ImVec2(displayWidth, displayHeight));
            ImGui::EndGroup();
        } else if (hasOriginalImage) {
            ImGui::Text("Original Image");
            ImGui::Image(reinterpret_cast<void*>(originalTexture), ImVec2(displayWidth, displayHeight));
        } else if (hasProcessedImage) {
            ImGui::Text("Processed Image");
            ImGui::Image(reinterpret_cast<void*>(processedTexture), ImVec2(displayWidth, displayHeight));
        }
    } else {
        ImGui::Text("No image loaded.");
    }

    ImGui::EndGroup(); // End right section


    ImGui::Separator();
    ImGui::Spacing();
    ImGui::Text("Performance Results:");
    ImGui::TextWrapped("%s", performanceResults.c_str());

    ImGui::End(); // End main window
}

int main() {
    // Initialize GLFW
    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW" << std::endl;
        return -1;
    }
    // OpenGL version configuration
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
        
    // Create window
    GLFWwindow* window = glfwCreateWindow(900, 700, "Image Processing Performance Comparison", NULL, NULL);
    if (!window) {
        std::cerr << "Failed to create window" << std::endl;
        glfwTerminate();
        return -1;
    }
    
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1); // Enable V-Sync
    
    // Initialize gl3w
    if (gl3wInit() != 0) {
        std::cerr << "Failed to initialize GL3W" << std::endl;
        return -1;
    }
    
    // ImGui initialization
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    
    ImGui::StyleColorsDark();

    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 330");
    
    // Initialize our classes
    imageProcessor = std::make_unique<ImageProcessor>();
    timer = std::make_unique<PerformanceTimer>();

    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();
        
        // Start ImGui frame
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();
        
        // GUI rendering
        renderGUI();
        
        // Rendering
        ImGui::Render();
        int display_w, display_h;
        glfwGetFramebufferSize(window, &display_w, &display_h);
        glViewport(0, 0, display_w, display_h);
        glClearColor(0.45f, 0.55f, 0.60f, 1.00f);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        
        glfwSwapBuffers(window);
    }
    // Cleanup
    deleteTexture(originalTexture);
    deleteTexture(processedTexture);

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    glfwDestroyWindow(window);
    glfwTerminate();
    
    return 0;
}