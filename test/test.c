#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include "dmp-dv.h"
#include "dmp_dv_cmdraw_v0.h"
#include "ion.h"

#define CMD_SIZE(N) \
  sizeof(dmp_dv_kcmdraw_v0) - (32 - N) * sizeof(dmp_dv_kcmdraw_v0_conv_run)

dmp_dv_kcmdraw_v0 cmd0 = {
    .size = CMD_SIZE(1),
    .version = 0,
    .topo = 0x1,
    .w = 8,
    .h = 8,
    .z = 1,
    .c = 8,
    .output_mode = 0,
    .run =
        {
            [0] =
                {
                    .conv_pad = 0x01010101,
                    .pool_pad = 0,
                    .m = 8,
                    .conv_enable = 1,
                    .p = 3,
                    .pz = 1,
                    .conv_stride = 0x0101,
                    .weight_fmt = 1,
                    .pool_enable = 0,
                    .actfunc = 0,
                    .rectifi_en = 0,
                    .lrn = 0,
                },
        },
};

static int allocate_ion_buf(int fd, uint32_t mask, size_t size) {
  struct ion_allocation_data ion_alloc;
  
  memset(&ion_alloc, 0, sizeof(ion_alloc));
  ion_alloc.len = size;
  ion_alloc.heap_id_mask = mask;
  ioctl(fd, ION_IOC_ALLOC, &ion_alloc);
  
  return ion_alloc.fd;
}

int main(void) {
  int i, ret, kick_count = 0;
  int ionfd, dvfd;
  struct ion_heap_data *ion_heap;
  struct ion_heap_query ion_query;
  struct dmp_dv_kcmd_impl dv_cmd;
  uint32_t ion_heap_id_mask = 0;
  
  ionfd = open("/dev/ion", O_RDONLY | O_CLOEXEC);
  if (ionfd < 0) {
    printf("Open ion device failed!\n");
    return -1;
  }
  dvfd = open("/dev/dv_conv", O_RDONLY | O_CLOEXEC);
  if (dvfd < 0) {
    printf("Open dv_conv device failed!\n");
    return -1;
  }
  
  // query ion heaps
  memset(&ion_query, 0, sizeof(ion_query));
  ret = ioctl(ionfd, ION_IOC_HEAP_QUERY, &ion_query);
  if (ret < 0) {
    printf("ion heap query failed!\n");
    return -1;
  }

  ion_heap = malloc(sizeof(*ion_heap) * ion_query.cnt);
  ion_query.heaps = (__u64)ion_heap;
  ret = ioctl(ionfd, ION_IOC_HEAP_QUERY, &ion_query);
  if (ret < 0) {
    printf("ion heap query failed!\n");
    return -1;
  }
  
  for (i = 0; i < ion_query.cnt; ++i) {
    if (ion_heap[i].type == ION_HEAP_TYPE_DMA)
	    ion_heap_id_mask |= (1 << ion_heap[i].heap_id);
  }
  free(ion_heap);
  
  if (ion_heap_id_mask == 0) {
    printf("No suitable ion heap found!\n");
    return -1;
  }
  
  // allocate buffers
  cmd0.input_buf.fd = allocate_ion_buf(ionfd, ion_heap_id_mask, 8 * 8 * 8 * 4);
  if (cmd0.input_buf.fd < 0) {
    printf("Failed to allocate ion buffer!\n");
    return -1;
  }
  cmd0.output_buf.fd = cmd0.input_buf.fd;
  cmd0.output_buf.offs = 8 * 8 * 8 * 2;
  cmd0.eltwise_buf.fd = -1;
  cmd0.run[0].weight_buf.fd = allocate_ion_buf(ionfd, ion_heap_id_mask,
    3 * 3 * 8 * 8 * 2 + 8 * 2);
  if (cmd0.run[0].weight_buf.fd < 0) {
    printf("Failed to allocate ion buffer!\n");
    return -1;
  }
  
  // kick command
  ret = ioctl(dvfd, DMP_DV_IOC_GET_KICK_COUNT, &kick_count);
  if (ret < 0) {
    printf("Failed to get dv kick count!\n");
    return -1;
  }
  printf("Conv kick count before kick = %d.\n", kick_count);
  
  dv_cmd.cmd_num = 1;
  dv_cmd.cmd_pointer = (__u64)&cmd0;
  ret = ioctl(dvfd, DMP_DV_IOC_APPEND_CMD, &dv_cmd);
  if (ret < 0) {
    printf("Failed to append command!\n");
    return -1;
  }
  
  ret = ioctl(dvfd, DMP_DV_IOC_RUN, NULL);
  if (ret < 0) {
    printf("Failed to start run!\n");
    return -1;
  }
  
  ret = ioctl(dvfd, DMP_DV_IOC_WAIT, NULL);
  if (ret < 0) {
    printf("Failed to wait dv HW!\n");
    return -1;
  }
  
  ret = ioctl(dvfd, DMP_DV_IOC_GET_KICK_COUNT, &kick_count);
  if (ret < 0) {
    printf("Failed to get dv kick count!\n");
    return -1;
  }
  printf("Conv kick count after kick = %d.\n", kick_count);

  close(cmd0.input_buf.fd);
  close(cmd0.output_buf.fd);
  close(cmd0.run[0].weight_buf.fd);
  close(ionfd);
  close(dvfd);
  return 0;
}
