#define _CRT_SECURE_NO_WARNINGS
#include <iostream>
#include <cmath>
#include <vector>
#include <algorithm>
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
#define M_PI 3.14159265358979323846


// проста€ структура дл€ представлени€ пиксел€
struct Pixel {
    unsigned char r, g, b;
};

// функци€ дл€ загрузки изображени€ из файла
std::vector<std::vector<Pixel>> loadImage(const char* filename, int& width, int& height) {
    // загружаем изображение с помощью библиотеки stb_image.h
    int channels;
    unsigned char* img_data = stbi_load(filename, &width, &height, &channels, 3); // загружаем только RGB каналы
    if (!img_data) {
        std::cerr << "ошибка при загрузке изображени€." << std::endl;
        exit(1);
    }

    // конвертируем данные изображени€ в вектор векторов пикселей
    std::vector<std::vector<Pixel>> image(height, std::vector<Pixel>(width));
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            int idx = (y * width + x) * 3; // RGB каналы
            image[y][x] = { img_data[idx], img_data[idx + 1], img_data[idx + 2] };
        }
    }

    // освобождаем пам€ть, выделенную дл€ данных изображени€
    stbi_image_free(img_data);

    return image;
}

// функци€ дл€ поворота изображени€ на заданный угол и сохранени€ его в файл с использованием интерпол€ции
void rotateImage(const std::vector<std::vector<Pixel>>& image, float angle, const char* filename) {
    int width = image[0].size();
    int height = image.size();
    float radians = angle * M_PI / 180.0f;

    float cos_theta = std::cos(radians);
    float sin_theta = std::sin(radians);
    int new_width = static_cast<int>(std::ceil(std::abs(width * cos_theta) + std::abs(height * sin_theta)));
    int new_height = static_cast<int>(std::ceil(std::abs(width * sin_theta) + std::abs(height * cos_theta)));

    std::vector<Pixel> rotatedPixels(new_width * new_height, {255, 255, 255});

    float cx = width / 2.0f;
    float cy = height / 2.0f;

    for (int y = 0; y < new_height; ++y) {
        for (int x = 0; x < new_width; ++x) {
            float src_x = (x - new_width / 2.0f) * cos_theta + (y - new_height / 2.0f) * sin_theta + cx;
            float src_y = -(x - new_width / 2.0f) * sin_theta + (y - new_height / 2.0f) * cos_theta + cy;

            if (src_x >= 0 && src_x < width - 1 && src_y >= 0 && src_y < height - 1) {
                int x0 = static_cast<int>(src_x);
                int y0 = static_cast<int>(src_y);
                int x1 = x0 + 1;
                int y1 = y0 + 1;
                float dx = src_x - x0;
                float dy = src_y - y0;

                // билинейна€ интерпол€ци€
                Pixel p00 = image[y0][x0];
                Pixel p01 = image[y0][x1];
                Pixel p10 = image[y1][x0];
                Pixel p11 = image[y1][x1];

                Pixel interpolatedPixel;
                interpolatedPixel.r = static_cast<unsigned char>((1 - dx) * (1 - dy) * p00.r + dx * (1 - dy) * p01.r + (1 - dx) * dy * p10.r + dx * dy * p11.r);
                interpolatedPixel.g = static_cast<unsigned char>((1 - dx) * (1 - dy) * p00.g + dx * (1 - dy) * p01.g + (1 - dx) * dy * p10.g + dx * dy * p11.g);
                interpolatedPixel.b = static_cast<unsigned char>((1 - dx) * (1 - dy) * p00.b + dx * (1 - dy) * p01.b + (1 - dx) * dy * p10.b + dx * dy * p11.b);

                int destOffset = (y * new_width + x);
                rotatedPixels[destOffset] = interpolatedPixel;
            }
        }
    }

    stbi_write_png(filename, new_width, new_height, 3, reinterpret_cast<const void*>(rotatedPixels.data()), new_width * 3);
}


float findRotationAngle(const std::vector<std::vector<Pixel>>& image) {
    int height = image.size();
    int width = image[0].size();

    // преобразование изображени€ в оттенки серого
    std::vector<std::vector<float>> gray_image(height, std::vector<float>(width, 0.0f));
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            const Pixel& pixel = image[y][x];
            // формула дл€ расчета €ркости пиксел€ (весовое среднее каналов RGB)
            gray_image[y][x] = (0.3f * pixel.r + 0.59f * pixel.g + 0.11f * pixel.b) / 255.0f;
        }
    }

    // вычисление угла поворота
    float sum_x = 0.0f, sum_y = 0.0f;
    float sum_x2 = 0.0f, sum_y2 = 0.0f, sum_xy = 0.0f;
    int count = 0;

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            // исключаем "белые" пиксели
            if (gray_image[y][x] > 0.5f) continue;

            sum_x += x;
            sum_y += y;
            sum_x2 += x * x;
            sum_y2 += y * y;
            sum_xy += x * y;
            ++count;
        }
    }

    float denominator = count * sum_x2 - sum_x * sum_x;
    if (denominator == 0) return 0.0f; // избегаем делени€ на ноль

    float slope = (count * sum_xy - sum_x * sum_y) / denominator;
    float angle = ((std::atan(slope) * 180.0f) * 1.4) / M_PI;
    if (std::abs(angle) > 1.0f) {
        angle = -angle;
        return angle;
    }
     if (std::abs(angle) < 2.0f) {
        angle = 0;
        return angle;
    }
    return angle;
}

int main() {
    setlocale(LC_ALL, "ru");
    int width, height;
    // загружаем изображение
    std::vector<std::vector<Pixel>> image = loadImage("input_image.jpeg", width, height);
    // находим угол поворота
    float angle = findRotationAngle(image);
    // поворачиваем изображение и сохран€ем его в файл
    rotateImage(image, angle, "output_image.jpeg");
    // выводим результат найденного угла
    std::cout << "”гол поворота: " << angle << " градусов" << std::endl;
    return 0;
}
