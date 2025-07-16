#define STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <stb_image.h>
#include <stb_image_write.h>

#include "ImageProcessor.h"
#include <string>
#include <thread>
#include <omp.h>
#include <algorithm>
#include <future>
#include <iostream>
#include <cmath>
#include <climits>
#include <chrono>

ImageProcessor::ImageProcessor() : threadCount(std::thread::hardware_concurrency()), openmpThreadCount(std::thread::hardware_concurrency()) {
    pthread_mutex_init(&pthreadMutex, nullptr);
}

ImageProcessor::~ImageProcessor() {
    pthread_mutex_destroy(&pthreadMutex);
}

bool ImageProcessor::loadImage(const std::string& filename) {
    int width, height, channels;
    unsigned char* img_data = stbi_load(filename.c_str(), &width, &height, &channels, 3); // Force RGB conversion

    if (!img_data) {
        std::cerr << "Failed to load image: " << filename << std::endl;
        std::cerr << "STB Error: " << stbi_failure_reason() << std::endl;
        return false;
    }
    
    std::cout << "Image loaded successfully: " << width << "x" << height << " (" << channels << " channels)" << std::endl;
    
    currentImage = Image(width, height, 3);
    std::copy(img_data, img_data + width * height * 3, currentImage.ptr());
    
    stbi_image_free(img_data);
    return true;
}

bool ImageProcessor::saveImage(const std::string& filename) {
    if (processedImage.empty()) {
        std::cerr << "No image to save." << std::endl;
        return false;
    }
    
    int result = stbi_write_jpg(filename.c_str(), processedImage.width, processedImage.height, 
                               processedImage.channels, processedImage.ptr(), 90);
    
    return result != 0;
}

Image& ImageProcessor::getCurrentImage() {
    return currentImage;
}

Image& ImageProcessor::getProcessedImage() {
    return processedImage;
}

// ========== Thread Configuration ==========
void ImageProcessor::setThreadCount(int count) {
    threadCount = std::max(1, std::min(count, 10));
}
int ImageProcessor::getThreadCount() const {
    return threadCount;
}
void ImageProcessor::setOpenMPThreadCount(int count) {
    openmpThreadCount = std::max(1, std::min(count, 10));
}
int ImageProcessor::getOpenMPThreadCount() const {
    return openmpThreadCount;
}

// ========== Common Interface Functions ==========
Image ImageProcessor::applyGaussianBlur(const Image& image, int kernelSize, double sigmaX, ProcessingMethod method) {
    switch (method) {
        case ProcessingMethod::SEQUENTIAL:
            return gaussianBlurSequential(image, kernelSize, sigmaX);
        case ProcessingMethod::PTHREAD:
            return gaussianBlurPthread(image, kernelSize, sigmaX);
        case ProcessingMethod::OPENMP:
            return gaussianBlurOpenMP(image, kernelSize, sigmaX);
        default:
            return Image(image);
    }
}
Image ImageProcessor::applyRotation(const Image& image, RotationAngle angle, ProcessingMethod method) {
    switch (method) {
        case ProcessingMethod::SEQUENTIAL:
            return rotateSequential(image, angle);
        case ProcessingMethod::PTHREAD:
            return rotatePthread(image, angle);
        case ProcessingMethod::OPENMP:
            return rotateOpenMP(image, angle);
        default:
            return Image(image);
    }
}
Image ImageProcessor::adjustBrightness(const Image& image, double alpha, int beta, ProcessingMethod method) {
    switch (method) {
        case ProcessingMethod::SEQUENTIAL:
            return brightnessSequential(image, alpha, beta);
        case ProcessingMethod::PTHREAD:
            return brightnessPthread(image, alpha, beta);
        case ProcessingMethod::OPENMP:
            return brightnessOpenMP(image, alpha, beta);
        default:
            return Image(image);
    }
}

// ========== Sequential Implementation ==========
Image ImageProcessor::gaussianBlurSequential(const Image& image, int kernelSize, double sigmaX) {
    Image result(image.width, image.height, image.channels);
    auto kernel = createGaussianKernel(kernelSize, sigmaX);
    int halfKernel = kernelSize / 2;
    
    // Copy border pixels directly from original image
    for (int y = 0; y < image.height; ++y) {
        for (int x = 0; x < image.width; ++x) {
            if (y < halfKernel || y >= image.height - halfKernel || 
                x < halfKernel || x >= image.width - halfKernel) {
                for (int c = 0; c < image.channels; ++c) {
                    int idx = (y * image.width + x) * image.channels + c;
                    result.data[idx] = image.data[idx];
                }
            }
        }
    }

    for (int y = 0; y< image.height; ++y){
        for(int x = 0; x <image.width; ++x){
            if (y < halfKernel || y>= image.height - halfKernel ||
                x < halfKernel || x>= image.width - halfKernel){
                for (int c =0; c<image.channels; c++){
                    int idx = (y * image.width + x) * image.channels + c;
                    result.data[idx] = image.data[idx];
                }
            }
        }
    }
    
    // Apply Gaussian blur to inner region
    for (int y = halfKernel; y < image.height - halfKernel; ++y) {
        for (int x = halfKernel; x < image.width - halfKernel; ++x) {
            for (int c = 0; c < image.channels; ++c) {
                float sum = 0.0f;
                
                for (int ky = -halfKernel; ky <= halfKernel; ++ky) {
                    for (int kx = -halfKernel; kx <= halfKernel; ++kx) {
                        int pixelIdx = ((y + ky) * image.width + (x + kx)) * image.channels + c;
                        float weight = kernel[ky + halfKernel][kx + halfKernel];
                        sum += image.data[pixelIdx] * weight;
                    }
                }
                
                int resultIdx = (y * result.width + x) * result.channels + c;
                result.data[resultIdx] = static_cast<unsigned char>(std::clamp(sum, 0.0f, 255.0f));
            }
        }
    }
    
    processedImage = result;
    return result;
}

Image ImageProcessor::rotateSequential(const Image& image, RotationAngle angle) {
    Image result;
    
    switch (angle) {
        case RotationAngle::ROTATE_90:
            result = Image(image.height, image.width, image.channels);
            for (int y = 0; y < image.height; ++y) {
                for (int x = 0; x < image.width; ++x) {
                    for (int c = 0; c < image.channels; ++c) {
                        int srcIdx = (y * image.width + x) * image.channels + c;
                        int dstIdx = (x * result.width + (image.height - 1 - y)) * result.channels + c;
                        result.data[dstIdx] = image.data[srcIdx];
                    }
                }
            }
            break;
            
        case RotationAngle::ROTATE_180:
            result = Image(image.width, image.height, image.channels);
            for (int y = 0; y < image.height; ++y) {
                for (int x = 0; x < image.width; ++x) {
                    for (int c = 0; c < image.channels; ++c) {
                        int srcIdx = (y * image.width + x) * image.channels + c;
                        int dstIdx = ((image.height - 1 - y) * result.width + (image.width - 1 - x)) * result.channels + c;
                        result.data[dstIdx] = image.data[srcIdx];
                    }
                }
            }
            break;
            
        case RotationAngle::ROTATE_270:
            result = Image(image.height, image.width, image.channels);
            for (int y = 0; y < image.height; ++y) {
                for (int x = 0; x < image.width; ++x) {
                    for (int c = 0; c < image.channels; ++c) {
                        int srcIdx = (y * image.width + x) * image.channels + c;
                        int dstIdx = ((image.width - 1 - x) * result.width + y) * result.channels + c;
                        result.data[dstIdx] = image.data[srcIdx];
                    }
                }
            }
            break;
    }
    
    processedImage = result;
    return result;
}
Image ImageProcessor::brightnessSequential(const Image& image, double alpha, int beta) {
    Image result(image.width, image.height, image.channels);
    
    for (int i = 0; i < image.width * image.height * image.channels; ++i) {
        int newValue = static_cast<int>(alpha * image.data[i] + beta);
        result.data[i] = static_cast<unsigned char>(std::clamp(newValue, 0, 255));
    }
    
    processedImage = result;
    return result;
}

// ========== Pthread Implementation ==========
Image ImageProcessor::gaussianBlurPthread(const Image& image, int kernelSize, double sigmaX) {
    Image result(image.width, image.height, image.channels);
    auto kernel = createGaussianKernel(kernelSize, sigmaX);
    int halfKernel = kernelSize / 2;
    
    // Copy border pixels directly from original image
    for (int y = 0; y < image.height; ++y) {
        for (int x = 0; x < image.width; ++x) {
            if (y < halfKernel || y >= image.height - halfKernel || 
                x < halfKernel || x >= image.width - halfKernel) {
                for (int c = 0; c < image.channels; ++c) {
                    int idx = (y * image.width + x) * image.channels + c;
                    result.data[idx] = image.data[idx];
                }
            }
        }
    }
    
    int numThreads = threadCount;
    int rowsPerThread = (image.height - 2 * halfKernel) / numThreads;
    
    std::vector<pthread_t> threads(numThreads);
    std::vector<PthreadBlurData> threadData(numThreads);
    
    for (int t = 0; t < numThreads; ++t) {
        int startRow = halfKernel + t * rowsPerThread;
        int endRow = (t == numThreads - 1) ? image.height - halfKernel : startRow + rowsPerThread;
        
        threadData[t] = {&image, &result, &kernel, startRow, endRow, halfKernel};
        pthread_create(&threads[t], nullptr, pthreadBlurWorker, &threadData[t]);
    }
    
    for (int t = 0; t < numThreads; ++t) {
        pthread_join(threads[t], nullptr);
    }
    
    processedImage = result;
    return result;
}
Image ImageProcessor::rotatePthread(const Image& image, RotationAngle angle) {
    Image result;
    
    switch (angle) {
        case RotationAngle::ROTATE_90:
            result = Image(image.height, image.width, image.channels);
            break;
        case RotationAngle::ROTATE_180:
            result = Image(image.width, image.height, image.channels);
            break;
        case RotationAngle::ROTATE_270:
            result = Image(image.height, image.width, image.channels);
            break;
    }

    int numThreads = threadCount;
    int rowsPerThread = image.height / numThreads;
    
    std::vector<pthread_t> threads(numThreads);
    std::vector<PthreadRotateData> threadData(numThreads);
    
    for (int t = 0; t < numThreads; ++t) {
        int startRow = t * rowsPerThread;
        int endRow = (t == numThreads - 1) ? image.height : (t + 1) * rowsPerThread;
        
        threadData[t] = {&image, &result, angle, startRow, endRow};
        pthread_create(&threads[t], nullptr, pthreadRotateWorker, &threadData[t]);
    }
    for (int t = 0; t < numThreads; ++t) {
        pthread_join(threads[t], nullptr);
    }
    processedImage = result;
    return result;
}
Image ImageProcessor::brightnessPthread(const Image& image, double alpha, int beta) {
    Image result(image.width, image.height, image.channels);
    
    int numThreads = threadCount;
    int pixelsPerThread = (image.width * image.height) / numThreads;
    
    std::vector<pthread_t> threads(numThreads);
    std::vector<PthreadBrightnessData> threadData(numThreads);
    
    for (int t = 0; t < numThreads; ++t) {
        int startPixel = t * pixelsPerThread;
        int endPixel = (t == numThreads - 1) ? image.width * image.height : (t + 1) * pixelsPerThread;
        
        threadData[t] = {&image, &result, alpha, beta, startPixel, endPixel};
        pthread_create(&threads[t], nullptr, pthreadBrightnessWorker, &threadData[t]);
    }
    for (int t = 0; t < numThreads; ++t) {
        pthread_join(threads[t], nullptr);
    }
    processedImage = result;
    return result;
}

// ========== Pthread Worker Functions ==========
void* ImageProcessor::pthreadBlurWorker(void* arg) {
    PthreadBlurData* data = static_cast<PthreadBlurData*>(arg);
    
    for (int y = data->startRow; y < data->endRow; ++y) {
        for (int x = data->halfKernel; x < data->input->width - data->halfKernel; ++x) {
            for (int c = 0; c < data->input->channels; ++c) {
                float sum = 0.0f;
                
                for (int ky = -data->halfKernel; ky <= data->halfKernel; ++ky) {
                    for (int kx = -data->halfKernel; kx <= data->halfKernel; ++kx) {
                        int pixelIdx = ((y + ky) * data->input->width + (x + kx)) * data->input->channels + c;
                        float weight = (*data->kernel)[ky + data->halfKernel][kx + data->halfKernel];
                        sum += data->input->data[pixelIdx] * weight;
                    }
                }
                
                int resultIdx = (y * data->output->width + x) * data->output->channels + c;
                data->output->data[resultIdx] = static_cast<unsigned char>(std::clamp(sum, 0.0f, 255.0f));
            }
        }
    }
    return nullptr;
}
void* ImageProcessor::pthreadRotateWorker(void* arg) {
    PthreadRotateData* data = static_cast<PthreadRotateData*>(arg);
    
    for (int y = data->startRow; y < data->endRow; ++y) {
        for (int x = 0; x < data->input->width; ++x) {
            for (int c = 0; c < data->input->channels; ++c) {
                int srcIdx = (y * data->input->width + x) * data->input->channels + c;
                int dstIdx;
                
                switch (data->angle) {
                    case RotationAngle::ROTATE_90:
                        dstIdx = (x * data->output->width + (data->input->height - 1 - y)) * data->output->channels + c;
                        break;
                    case RotationAngle::ROTATE_180:
                        dstIdx = ((data->input->height - 1 - y) * data->output->width + (data->input->width - 1 - x)) * data->output->channels + c;
                        break;
                    case RotationAngle::ROTATE_270:
                        dstIdx = ((data->input->width - 1 - x) * data->output->width + y) * data->output->channels + c;
                        break;
                }                
                data->output->data[dstIdx] = data->input->data[srcIdx];
            }
        }
    }
    return nullptr;
}
void* ImageProcessor::pthreadBrightnessWorker(void* arg) {
    PthreadBrightnessData* data = static_cast<PthreadBrightnessData*>(arg);
    
    for (int i = data->startPixel * data->input->channels; i < data->endPixel * data->input->channels; ++i) {
        int newValue = static_cast<int>(data->alpha * data->input->data[i] + data->beta);
        data->output->data[i] = static_cast<unsigned char>(std::clamp(newValue, 0, 255));
    }
    return nullptr;
}

// ========== OpenMP Implementation ==========
Image ImageProcessor::gaussianBlurOpenMP(const Image& image, int kernelSize, double sigmaX) {
    Image result(image.width, image.height, image.channels);
    auto kernel = createGaussianKernel(kernelSize, sigmaX);
    int halfKernel = kernelSize / 2;
    
    // Copy border pixels directly from original image
    for (int y = 0; y < image.height; ++y) {
        for (int x = 0; x < image.width; ++x) {
            if (y < halfKernel || y >= image.height - halfKernel || 
                x < halfKernel || x >= image.width - halfKernel) {
                for (int c = 0; c < image.channels; ++c) {
                    int idx = (y * image.width + x) * image.channels + c;
                    result.data[idx] = image.data[idx];
                }
            }
        }
    }
    omp_set_num_threads(openmpThreadCount);
    #pragma omp parallel for
    for (int y = halfKernel; y < image.height - halfKernel; ++y) {
        for (int x = halfKernel; x < image.width - halfKernel; ++x) {
            for (int c = 0; c < image.channels; ++c) {
                float sum = 0.0f;
                
                for (int ky = -halfKernel; ky <= halfKernel; ++ky) {
                    for (int kx = -halfKernel; kx <= halfKernel; ++kx) {
                        int pixelIdx = ((y + ky) * image.width + (x + kx)) * image.channels + c;
                        float weight = kernel[ky + halfKernel][kx + halfKernel];
                        sum += image.data[pixelIdx] * weight;
                    }
                }
                
                int resultIdx = (y * result.width + x) * result.channels + c;
                result.data[resultIdx] = static_cast<unsigned char>(std::clamp(sum, 0.0f, 255.0f));
            }
        }
    }
    processedImage = result;
    return result;
}
Image ImageProcessor::rotateOpenMP(const Image& image, RotationAngle angle) {
    Image result;
    
    switch (angle) {
        case RotationAngle::ROTATE_90:
            result = Image(image.height, image.width, image.channels);
            omp_set_num_threads(openmpThreadCount);
            #pragma omp parallel for
            for (int y = 0; y < image.height; ++y) {
                for (int x = 0; x < image.width; ++x) {
                    for (int c = 0; c < image.channels; ++c) {
                        int srcIdx = (y * image.width + x) * image.channels + c;
                        int dstIdx = (x * result.width + (image.height - 1 - y)) * result.channels + c;
                        result.data[dstIdx] = image.data[srcIdx];
                    }
                }
            }
            break;
            
        case RotationAngle::ROTATE_180:
            result = Image(image.width, image.height, image.channels);
            omp_set_num_threads(openmpThreadCount);
            #pragma omp parallel for
            for (int y = 0; y < image.height; ++y) {
                for (int x = 0; x < image.width; ++x) {
                    for (int c = 0; c < image.channels; ++c) {
                        int srcIdx = (y * image.width + x) * image.channels + c;
                        int dstIdx = ((image.height - 1 - y) * result.width + (image.width - 1 - x)) * result.channels + c;
                        result.data[dstIdx] = image.data[srcIdx];
                    }
                }
            }
            break;
            
        case RotationAngle::ROTATE_270:
            result = Image(image.height, image.width, image.channels);
            omp_set_num_threads(openmpThreadCount);
            #pragma omp parallel for
            for (int y = 0; y < image.height; ++y) {
                for (int x = 0; x < image.width; ++x) {
                    for (int c = 0; c < image.channels; ++c) {
                        int srcIdx = (y * image.width + x) * image.channels + c;
                        int dstIdx = ((image.width - 1 - x) * result.width + y) * result.channels + c;
                        result.data[dstIdx] = image.data[srcIdx];
                    }
                }
            }
            break;
    }
    processedImage = result;
    return result;
}
Image ImageProcessor::brightnessOpenMP(const Image& image, double alpha, int beta) {
    Image result(image.width, image.height, image.channels);
    
    omp_set_num_threads(openmpThreadCount);
    #pragma omp parallel for 
    for (int i = 0; i < image.width * image.height * image.channels; ++i) {
        int newValue = static_cast<int>(alpha * image.data[i] + beta);
        result.data[i] = static_cast<unsigned char>(std::clamp(newValue, 0, 255));
    }
    processedImage = result;
    return result;
}
std::vector<std::vector<float>> ImageProcessor::createGaussianKernel(int size, double sigma) {
    std::vector<std::vector<float>> kernel(size, std::vector<float>(size));
    int halfSize = size / 2;
    float sum = 0.0f;
    
    for (int y = -halfSize; y <= halfSize; ++y) {
        for (int x = -halfSize; x <= halfSize; ++x) {
            float value = std::exp(-(x*x + y*y) / (2.0 * sigma * sigma));
            kernel[y + halfSize][x + halfSize] = value;
            sum += value;
        }
    }
    // Normalization
    for (int y = 0; y < size; ++y) {
        for (int x = 0; x < size; ++x) {
            kernel[y][x] /= sum;
        }
    }
    return kernel;
}

// ========== Preset Mode Implementation ==========
ImageProcessor::PresetResults ImageProcessor::applyAllFilter(const PresetParams& params) {
    PresetResults results;
    
    results.image = Image(params.image.width, params.image.height, params.image.channels);
    switch (params.angle) {
        case RotationAngle::ROTATE_90:
        case RotationAngle::ROTATE_270:
            results.image = Image(params.image.height, params.image.width, params.image.channels);
            break;
        case RotationAngle::ROTATE_180:
            results.image = Image(params.image.width, params.image.height, params.image.channels);
            break;
    }
        
    auto startTime = std::chrono::high_resolution_clock::now();
    if (params.method == ProcessingMethod::SEQUENTIAL){
        // Blurring Sequentially
        Image inputImage(params.image); // Shared Image Data

        auto blurStart = std::chrono::high_resolution_clock::now();
        Image blurred = gaussianBlurSequential(inputImage, params.kernelSize, params.sigmaX);
        auto blurEnd = std::chrono::high_resolution_clock::now();
        results.blurTime = std::chrono::duration<double, std::milli>(blurEnd - blurStart).count();

        // Rotating Sequentially
        auto rotateStart = std::chrono::high_resolution_clock::now();
        Image rotated = rotateSequential(blurred, params.angle);
        auto rotateEnd = std::chrono::high_resolution_clock::now();
        results.rotateTime = std::chrono::duration<double, std::milli>(rotateEnd - rotateStart).count();

        // Adjusting Brightness Sequentially
        auto brightStart = std::chrono::high_resolution_clock::now();
        results.image = brightnessSequential(rotated, params.alpha, params.beta);
        auto brightEnd = std::chrono::high_resolution_clock::now();
        results.brightnessTime = std::chrono::duration<double, std::milli>(brightEnd - brightStart).count();

    } else if (params.method == ProcessingMethod::PTHREAD) {
        // Use pthread for preset mode
        pthread_t threads[3];

        Image sharedImage = params.image;

        PresetWorkerData workerData;
        workerData = {
            this,
            &sharedImage,
            &pthreadMutex,
            params.method,
            params.kernelSize,
            params.sigmaX,
            params.angle,
            params.alpha,
            params.beta,
            &results.blurTime,
            &results.rotateTime,
            &results.brightnessTime
        };

        // Create threads
        pthread_create(&threads[0], nullptr, pthreadBlurWorkerMutex, &workerData);
        pthread_create(&threads[1], nullptr, pthreadRotateWorkerMutex, &workerData);
        pthread_create(&threads[2], nullptr, pthreadBrightnessWorkerMutex, &workerData);
        
        // Wait for completion
        for (int i = 0; i < 3; ++i) {
            pthread_join(threads[i], nullptr);
        }
        results.image = sharedImage;

    } else if (params.method == ProcessingMethod::OPENMP) {
        Image sharedImage = params.image; // shared input/output image
        #pragma omp parallel sections num_threads(3)
        {
            #pragma omp section
            {
                #pragma omp critical
                {
                    auto start = std::chrono::high_resolution_clock::now();
                    sharedImage = gaussianBlurSequential(sharedImage, params.kernelSize, params.sigmaX);
                    auto end = std::chrono::high_resolution_clock::now();
                    results.blurTime = std::chrono::duration<double, std::milli>(end - start).count();
                }
            }
            #pragma omp section
            {
                #pragma omp critical
                {
                    auto start = std::chrono::high_resolution_clock::now();
                    sharedImage = rotateSequential(sharedImage, params.angle);
                    auto end = std::chrono::high_resolution_clock::now();
                    results.rotateTime = std::chrono::duration<double, std::milli>(end - start).count();
                }
            }
            #pragma omp section
            {
                #pragma omp critical
                {
                    auto start = std::chrono::high_resolution_clock::now();
                    sharedImage = brightnessSequential(sharedImage, params.alpha, params.beta);
                    auto end = std::chrono::high_resolution_clock::now();
                    results.brightnessTime = std::chrono::duration<double, std::milli>(end - start).count();
                }
            }
        }
        results.image = sharedImage;
    }
    auto endTime = std::chrono::high_resolution_clock::now();
    results.totalTime = std::chrono::duration<double, std::milli>(endTime - startTime).count();
    
    processedImage = results.image;    
    return results;
}

// ========== Preset Mode Worker Functions ==========
void* ImageProcessor::pthreadBlurWorkerMutex(void* arg) {
    PresetWorkerData* data = static_cast<PresetWorkerData*>(arg);
    
    pthread_mutex_lock(data->mutex);
    auto start = std::chrono::high_resolution_clock::now();

    *(data->sharedImage) = data->processor->gaussianBlurSequential(*(data->sharedImage), data->kernelSize, data->sigmaX);
    auto end = std::chrono::high_resolution_clock::now();
    pthread_mutex_unlock(data->mutex);
    
    *(data->blurTime) = std::chrono::duration<double, std::milli>(end - start).count();
    
    return nullptr;
}
void* ImageProcessor::pthreadRotateWorkerMutex(void* arg) {
    PresetWorkerData* data = static_cast<PresetWorkerData*>(arg);
    
    pthread_mutex_lock(data->mutex);
    auto start = std::chrono::high_resolution_clock::now();

    *(data->sharedImage) = data->processor->rotateSequential(*(data->sharedImage), data->angle);
    auto end = std::chrono::high_resolution_clock::now();
    pthread_mutex_unlock(data->mutex);

    *(data->rotateTime) = std::chrono::duration<double, std::milli>(end - start).count();
    
    return nullptr;
}
void* ImageProcessor::pthreadBrightnessWorkerMutex(void* arg) {
    PresetWorkerData* data = static_cast<PresetWorkerData*>(arg);

    pthread_mutex_lock(data->mutex);
    auto start = std::chrono::high_resolution_clock::now();

    *(data->sharedImage) = data->processor->brightnessSequential(*(data->sharedImage), data->alpha, data->beta);
    auto end = std::chrono::high_resolution_clock::now();
    pthread_mutex_unlock(data->mutex);
    
    *(data->brightnessTime) = std::chrono::duration<double, std::milli>(end - start).count();
    
    return nullptr;
}