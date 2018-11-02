#include <stdint.h>
#include <string.h>
#include <sys/ioctl.h>
#include "ion.h"
#include "common.h"

int allocate_ion_buf(int fd, uint32_t mask, size_t size) {
	struct ion_allocation_data ion_alloc;

	memset(&ion_alloc, 0, sizeof(ion_alloc));
	ion_alloc.len = size;
	ion_alloc.heap_id_mask = mask;
	ioctl(fd, ION_IOC_ALLOC, &ion_alloc);

	return ion_alloc.fd;
}
