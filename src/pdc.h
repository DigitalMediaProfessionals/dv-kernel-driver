
#ifndef PDC_H_
#define PDC_H_

void pdc_config(void __iomem *pdc_addr, int* dims, unsigned int* fbPA);
void pdc_start(void __iomem *pdc_addr);
void pdc_stop(void __iomem *pdc_addr);

#define PDC_REG_FBADDR  0x0068
#define PDC_REG_SWAP    0x0078
#define PDC_REG_STATUS  0x007C

#endif // PDC_H_

