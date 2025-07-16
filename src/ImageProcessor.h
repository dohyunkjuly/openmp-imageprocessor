#pragma once

#include <string>
#include <thread>
#include <vector>
#include <memory>
#include <pthread.h>
#include <mutex>

// Simple image structure
struct Image {
    int width;
    int height;
    int channels;  // RGB = 3
    std::unique_ptr<unsigned char[]> data;
    
    Image() : width(0), height(0), channels(0) {}
    Image(int w, int h, int c) : width(w), height(h), channels(c) {
        data = std::make_unique<unsigned char[]>(w * h * c);
    }
    // Copy constructor
    Image(const Image& other) : width(other.width), height(other.height), channels(other.channels) {
        if (other.data) {
            data = std::make_unique<unsigned char[]>(width * height * channels);
            std::copy(other.data.get(), other.data.get() + width * height * channels, data.get());
        }
    }
    // Assignment operator
    Image& operator=(const Image& other) {
        if (this != &other) {
            width = other.width;
            height = other.height;
            channels = other.channels;
            if (other.data) {
                data = std::make_unique<unsigned char[]>(width * height * channels);
                std::copy(other.data.get(), other.data.get() + width * height * channels, data.get());
            } else {
                data.reset();
            }
        }
        return *this;
    }
    
    bool empty() const { return !data || width == 0 || height == 0; }
    unsigned char* ptr() { return data.get(); }
    const unsigned char* ptr() const { return data.get(); }
};

enum class ProcessingMethod {
    SEQUENTIAL,
    PTHREAD,
    OPENMP
};

enum class RotationAngle {
    ROTATE_90,
    ROTATE_180,
    ROTATE_270
};

class ImageProcessor {
private:
    Image currentImage;
    Image processedImage;
    int threadCount;
    int openmpThreadCount;
    
    // Thread safety
    mutable pthread_mutex_t pthreadMutex;
    
    // Sequential implementation
    Image gaussianBlurSequential(const Image& image, int kernelSize, double sigmaX);
    Image rotateSequential(const Image& image, RotationAngle angle);
    Image brightnessSequential(const Image& image, double alpha, int beta);
    
    // pthread implementation
    Image gaussianBlurPthread(const Image& image, int kernelSize, double sigmaX);
    Image rotatePthread(const Image& image, RotationAngle angle);
    Image brightnessPthread(const Image& image, double alpha, int beta);
    
    // OpenMP implementation
    Image gaussianBlurOpenMP(const Image& image, int kernelSize, double sigmaX);
    Image rotateOpenMP(const Image& image, RotationAngle angle);
    Image brightnessOpenMP(const Image& image, double alpha, int beta);
    
    // Pthread helper structures
    struct PthreadBlurData {
        const Image* input;
        Image* output;
        std::vector<std::vector<float>>* kernel;
        int startRow, endRow, halfKernel;
    };
    
    struct PthreadRotateData {
        const Image* input;
        Image* output;
        RotationAngle angle;
        int startRow, endRow;
    };
    
    struct PthreadBrightnessData {
        const Image* input;
        Image* output;
        double alpha;
        int beta;
        int startPixel, endPixel;
    };
    
    // Pthread worker functions
    static void* pthreadBlurWorker(void* arg);
    static void* pthreadRotateWorker(void* arg);
    static void* pthreadBrightnessWorker(void* arg);
    
    // Preset mode worker structures
    struct PresetWorkerData {
        ImageProcessor* processor;
        Image* sharedImage; // single shared output image
        pthread_mutex_t* mutex;
        ProcessingMethod method;
        // Blur params
        int kernelSize;
        double sigmaX;
        // Rotate params
        RotationAngle angle;
        // Brightness params
        double alpha;
        int beta;
        // Timing
        double* blurTime;
        double* rotateTime;
        double* brightnessTime;
    };

    static void* pthreadBlurWorkerMutex(void* arg);
    static void* pthreadRotateWorkerMutex(void* arg);
    static void* pthreadBrightnessWorkerMutex(void* arg);

    // Gaussian kernel creation
    std::vector<std::vector<float>> createGaussianKernel(int size, double sigma);

public:
    ImageProcessor();
    ~ImageProcessor();
    
    // Image load/save
    bool loadImage(const std::string& filename);
    bool saveImage(const std::string& filename);
    Image& getCurrentImage();
    Image& getProcessedImage();
    
    // Thread count configuration
    void setThreadCount(int count);
    int getThreadCount() const;
    void setOpenMPThreadCount(int count);
    int getOpenMPThreadCount() const;
    
    Image applyGaussianBlur(const Image& image, int kernelSize, double sigmaX, ProcessingMethod method);    
    Image applyRotation(const Image& image, RotationAngle angle, ProcessingMethod method);    
    Image adjustBrightness(const Image& image, double alpha, int beta, ProcessingMethod method);
    
    // Preset Mode - applies all three operations in parallel
    struct PresetResults {
        Image image;
        double blurTime;
        double rotateTime;
        double brightnessTime;
        double totalTime;
    };

    struct PresetParams {
        const Image& image;
        int kernelSize;
        double sigmaX;
        RotationAngle angle;
        double alpha;
        int beta;
        ProcessingMethod method;
    };
    PresetResults applyAllFilter(const PresetParams& params);
};