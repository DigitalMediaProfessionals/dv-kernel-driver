#pragma once

#include <stdint.h>

int allocate_ion_buf(int fd, uint32_t mask, size_t size);
