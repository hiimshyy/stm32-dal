/*
 * filter.h
 *
 *  Created on: Sep 30, 2025
 *      Author: tiensy
 */

#ifndef INC_FILTER_H_
#define INC_FILTER_H_

#include <string.h>  // memset

#define MA_FILTER_SIZE 10   // số mẫu trung bình (ví dụ 10 mẫu)

typedef struct {
    float buffer[MA_FILTER_SIZE];
    int index;
    int count;
    float sum;
} MAFilter_t;

/**
 * @brief Khởi tạo bộ lọc trung bình
 */
void MAFilter_Init(MAFilter_t *f);

/**
 * @brief Cập nhật giá trị mới và lấy ra giá trị đã lọc
 * @param f: con trỏ đến filter
 * @param newValue: giá trị mới (float)
 * @return float: giá trị trung bình sau khi lọc
 */
float MAFilter_Update(MAFilter_t *f, float newValue);



#endif /* INC_FILTER_H_ */
