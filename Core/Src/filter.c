/*
 * filter.c
 *
 *  Created on: Sep 30, 2025
 *      Author: tiensy
 */
#include "filter.h"

void MAFilter_Init(MAFilter_t *f) {
	memset(f, 0, sizeof(MAFilter_t));
}

float MAFilter_Update(MAFilter_t *f, float newValue) {
	// Trừ giá trị cũ ra khỏi tổng
	f->sum -= f->buffer[f->index];
	// Thêm giá trị mới vào buffer và tổng
	f->buffer[f->index] = newValue;
	f->sum += newValue;

	// Cập nhật chỉ số vòng tròn
	f->index = (f->index + 1) % MA_FILTER_SIZE;
	if (f->count < MA_FILTER_SIZE) {
		f->count++;
	}

	// Trả về giá trị trung bình
	return f->sum / f->count;
}



