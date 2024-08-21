
#ifndef CAMERA_H
#define CAMERA_H
#include "esp_camera.h"

// typedef struct {
//     int width;
//     int height;
//     int format;
//     uint8_t* pDataBuf;
//     size_t len;
// } camera_frame_t;

/**
 * @brief Initialize the camera
 * @param frame_size The frame size to use
 * @param jpeg_quality The JPEG quality to use
 * @return ESP_OK on success
 * @note This function should be called before any other camera functions
 * @note This function should be called only once
 * @note The frame size and JPEG quality can be changed later using the camera_set_framesize and camera_set_quality functions
*/
esp_err_t camera_init(/*int frame_size, int jpeg_quality)*/);

esp_err_t camera_frame_take(camera_fb_t** ppFrame_out);

esp_err_t camera_frame_release(camera_fb_t* pFrame);

/**
 * @brief Set the frame size
 * @param frame_size The frame size to use
 * @return ESP_OK on success
*/
esp_err_t camera_set_framesize(int frame_size);

/**
 * @brief Set the JPEG quality
 * @param jpeg_quality The JPEG quality to use
 * @return ESP_OK on success
*/
esp_err_t camera_set_quality(int jpeg_quality);

#endif // CAMERA_H