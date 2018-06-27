
#define DRM_DEV_NAME     "DMP_drm"
#define DRM_NUM_SUBDEV   3

#ifdef DMP_ZC706
#define USE_DEVTREE
//#define DMP_COHDMA
// map {0=CNV,1=PDC,2=FC} to irq Offset:
#ifdef USE_DEVTREE
static int sd2iMap[3]={0,3,2};       // {0,1,2} for KINT
#else
static int sd2iMap[3]={61,64,63};       // {61,62,63} for KINT
#endif
// map {0=CNV,1=PDC,2=FC} to regBaseAddr:
static unsigned int sd2rb[3]={0x43c00000,0x43c10000,0x43c20000};
#endif
#ifdef DMP_ARRIA10
//#define USE_DEVTREE
//#define DMP_COHDMA
#define HPS_REG_ADDR     0xffd05000   // rst manager section
#define HPS_REG_SIZE     0x00001000
#define F2SD_RST_ADDR    0x2c
// map {0=CNV,1=PDC,2=FC} to irq Offset:
#ifdef USE_DEVTREE
static int sd2iMap[3]={2,1,3};       // {2,0,3} for KINT
#else
static int sd2iMap[3]={53,52,54};       // {2,0,3} for KINT
#endif
// map {0=CNV,1=PDC,2=FC} to regBaseAddr:
//static unsigned int sd2rb[3]={0xff210000,0xff200000,0xff220000};
static unsigned int sd2rb[3]={0xff210000,0xff220000,0xff200000};
#endif
// map {0=CNV,1=PDC,2=FC} to regSize:
static unsigned int sd2rs[3]={0x2000,0x100,0x100};

#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/cdev.h>
#include <linux/io.h>  
#include <linux/err.h>   
#include <linux/stddef.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/types.h>    
#include <linux/spinlock.h> 
#include <linux/wait.h>     
#include <linux/uaccess.h>  
#include <linux/ioctl.h>    
#include <linux/mm.h>       
#include <linux/sched.h>    
#include <linux/dma-mapping.h>
#ifdef USE_DEVTREE
#include <linux/of.h>
#include <linux/of_irq.h>
#endif
#include "pdc.h"



#define CNV_IOC_MAJOR 0x82
#define CNV_WAITPDC _IOR(CNV_IOC_MAJOR, 3, unsigned int)
#define CNV_MEMSEC  _IOR(CNV_IOC_MAJOR, 4, unsigned int)
#define CNV_WAITINT _IO(CNV_IOC_MAJOR, 6)

#define REG_IO_ADDR(DV, OF) ((void __iomem *)(DV->bar_logical) + OF)

// module arguments: these can be modified on module loading:
// e.g: insmod **_km.ko width=1080 height=720

static int width = 640;
static int height = 480;
static int pStart = 1;
static int srcW = 0;
static int srcH = 0;
static int memGuess = 0;
static int workmemMB = 620;
static int vrb = 0;
static int pol = 0;
module_param(width, int, 0644);
module_param(height, int, 0644);
module_param(pStart, int, 0644);
//module_param(srcW, int, 0644);
//module_param(srcH, int, 0644);
module_param(memGuess, int, 0644);
module_param(workmemMB, int, 0644);
module_param(vrb, int, 0644);
module_param(pol, int, 0644);
MODULE_PARM_DESC(width,  "set the frame buffer width");
MODULE_PARM_DESC(height, "set the frame buffer height");
MODULE_PARM_DESC(pStart, "enable or disble auto-start");
//MODULE_PARM_DESC(srcW, "set the input buffer width");
//MODULE_PARM_DESC(srcH, "set the input buffer height");
MODULE_PARM_DESC(memGuess, "use unallocated mem setup based on actual previous alloc");
MODULE_PARM_DESC(workmemMB, "set the amount of workmem to allocate");
MODULE_PARM_DESC(vrb, "enable verbose prints");
MODULE_PARM_DESC(pol, "sync polarity setting");

#define MEM_SEC_N 16
unsigned int memSec[(2+MEM_SEC_N)*2];

struct dmp_dev
{
  spinlock_t         int_exclusive;
  wait_queue_head_t  int_status_wait;
  int                irqno;
  int                initDone;
  int                int_status;
  int                bank;    // meaning user-side write bank
  int                type;

  unsigned int       bar_physical;
  unsigned int       bar_size;
  void*              bar_logical;
};

struct drm_dev
{
  struct device* dev;
  dev_t          devt;
  struct cdev    cdev;

  struct dmp_dev subdev[DRM_NUM_SUBDEV];
};

static struct class* dddrm_class = NULL;


static irqreturn_t handleInt_p(int irq, void* dev_id)
{
  //unsigned int sts;
  unsigned int rsvStat;
  struct dmp_dev* subdev = dev_id;
  
  spin_lock(&subdev->int_exclusive);
  //printk(KERN_INFO "==> VINT!");
  //sts = ioread32(REG_IO_ADDR(subdev, PDC_REG_STATUS));
  //if(sts & 0x8000) printk(KERN_INFO "==> VINT: WARNING: UINT! sts = 0x%08x\n",sts);

  // for VINT version: clr VINT|UNIT (bits [17],[18]) 
  iowrite32(0x060000, REG_IO_ADDR(subdev, PDC_REG_SWAP));

  // for KINT version: clr KINT|UINT (bits [18],[21])
  //iowrite32(0x240000, REG_IO_ADDR(subdev, PDC_REG_SWAP));
  // note: we cannot use KINT with the below reg consumption polling method since 
  // consumption itself causes KINT, so we just end up waiting for, and processing
  // new interrupts in an infinite loop (never return to userland).
  
  if(subdev->int_status==1) {     // user waiting for int/swap
    // update FB addr using one shot rsrv:
    iowrite32(0x8|1, REG_IO_ADDR(subdev, 0x94));
    iowrite32(memSec[subdev->bank?2:0], REG_IO_ADDR(subdev, PDC_REG_FBADDR));
    // wait for FB addr update consume:
    rsvStat = 1;
    while(rsvStat & 1) rsvStat = ioread32(REG_IO_ADDR(subdev, 0x94));
    subdev->bank = subdev->bank?0:1;
    subdev->int_status = 2;
    wake_up_interruptible(&subdev->int_status_wait);
  }

  spin_unlock(&subdev->int_exclusive);
  return IRQ_HANDLED;
}

static unsigned int waitInt_p(struct dmp_dev* subdev)
{
  unsigned int r=0;
  unsigned long irq_save=0;

  spin_lock(&subdev->int_exclusive);
  subdev->int_status = 1;
  spin_unlock(&subdev->int_exclusive);

  r = wait_event_interruptible(subdev->int_status_wait, (subdev->int_status & 2));
  if (!r) {
    spin_lock_irqsave(&subdev->int_exclusive, irq_save);
    subdev->int_status = 0;
    r = memSec[subdev->bank?2:0];
    spin_unlock_irqrestore(&subdev->int_exclusive, irq_save);
  }

  return r;
}

static irqreturn_t handleInt_c(int irq, void *dev_id)
{
  struct dmp_dev* subdev=dev_id;

  spin_lock(&subdev->int_exclusive);
  iowrite32(0, REG_IO_ADDR(subdev, 0x420));

  subdev->int_status = 2;
  wake_up_interruptible(&subdev->int_status_wait);
  spin_unlock(&subdev->int_exclusive);

  return IRQ_HANDLED;
}

static void waitInt_c(struct dmp_dev* subdev)
{
  long ret=0;
  int wStat=0;
  unsigned long irq_save = 0;

  spin_lock(&subdev->int_exclusive);
  wStat = subdev->int_status;  
  subdev->int_status = (wStat==2) ? 0:1;
  spin_unlock(&subdev->int_exclusive);

  if (wStat!=2) {
    ret = wait_event_interruptible(subdev->int_status_wait, (subdev->int_status & 2));
    if (!ret) {
      spin_lock_irqsave(&subdev->int_exclusive, irq_save);
      subdev->int_status = 0;
      spin_unlock_irqrestore(&subdev->int_exclusive, irq_save);
    }
  }
}

static irqreturn_t handleInt_f(int irq, void *dev_id)
{
  struct dmp_dev* subdev=dev_id;

  spin_lock(&subdev->int_exclusive);
  iowrite32(0, REG_IO_ADDR(subdev, 0x20));

  subdev->int_status = 2;
  wake_up_interruptible(&subdev->int_status_wait);
  spin_unlock(&subdev->int_exclusive);

  return IRQ_HANDLED;
}

static unsigned int dmaAlloc(unsigned int size, unsigned int* sizeO,
			     struct drm_dev* drm_dev)
{
#ifdef DMP_COHDMA
  char* VA;
  dma_addr_t HND;
  *sizeO = 0;

  if(dma_set_mask_and_coherent(drm_dev->dev, DMA_BIT_MASK(32))) {
    printk(KERN_INFO "==>DMP_drm: no suitable DMA available(?!?)\n");
    return 0;
  }
  VA = dma_alloc_coherent(drm_dev->dev, size, &HND, GFP_USER);
  if(dma_mapping_error(drm_dev->dev, HND)) {
    printk(KERN_INFO "==>DMP_drm: DMA mapping error\n");
    return 0;
  }

  *sizeO = size;
  if(vrb==1) {
    printk(KERN_INFO "==>DMP_drm: cohDMA alloc @ PA 0x%08x VA 0x%08x (0x%08x)\n", 
	   (unsigned int)HND, (unsigned int)VA, (unsigned int)(__va((void*)HND)));
  }
  return (unsigned int)HND;

#else
  void* kmR;
  unsigned int VA,SZ;

  kmR=kmalloc(size, GFP_DMA);
  if(kmR==NULL) {
    printk(KERN_INFO "==>DMP_drm: kmalloc failed\n");
    return 0;
  }

  VA=(unsigned int)kmR;
  SZ=ksize((unsigned char*)VA);  
  if(vrb==1) {
    printk(KERN_INFO "==>DMP_drm: DMA alloc @ PA 0x%08x VA 0x%08x\n", 
	   (unsigned int)(__pa(VA)),VA);
  }
  if(SZ!=size) printk(KERN_INFO "==>DMP_drm: warning! kmalloc'd size<requested\n");
  *sizeO=SZ;
  return __pa(VA);

#endif
}


static int memSetup(struct drm_dev* drm_dev)
{
  int k,j,fbSize,allocT,allocTGT;
  unsigned int tmpA,tmpS,reqS;

  for(k=0; k<2*(2+MEM_SEC_N); k++) memSec[k]=0;
  fbSize=srcW*srcH*4;
  while(fbSize&0xf) fbSize++;
  
  if (memGuess!=0) {
    memSec[0]=0x2dc00000;
    memSec[1]=0x00400000;
    memSec[2]=0x2d800000;
    memSec[3]=0x00400000;
    memSec[4]=0x06c00000;
    memSec[5]=0x26c00000;    

    printk(KERN_INFO "==>using guessed mem alloc... (unsafe!)\n");
    printk(KERN_INFO "==>INFOm0: FB0 allocated: addr=0x%08x size=0x%08x bytes\n",
	   memSec[0],memSec[1]);
    printk(KERN_INFO "==>INFOm1: FB1 allocated: addr=0x%08x size=0x%08x bytes\n",
	   memSec[2],memSec[3]);
    printk(KERN_INFO "==>INFOm2: Workmem %dMB allocated in 1 sector: addr=0x%08x\n", 
	   memSec[5]/(1024*1024),memSec[4]);
    return 0;
  }
  else {
    if(fbSize > KMALLOC_MAX_SIZE) {
      printk(KERN_INFO "==>ERROR! framebuf size must be < KMALLOC_MAX_SIZE for dynamic alloc\n");
      return 1;
    }
    reqS = KMALLOC_MAX_SIZE;

    j=0;
    while(j<4) {
      tmpA=dmaAlloc(reqS,&tmpS,drm_dev);
      if(!tmpA) return 1;
      memSec[j++]=tmpA;
      memSec[j++]=tmpS;
    }

    allocTGT=workmemMB*1024*1024;
    allocT=0;
    j=0;
    k=0;
    while(j+1 < MEM_SEC_N) {
      tmpA=dmaAlloc(reqS,&tmpS,drm_dev);

      if(k==0) {
	memSec[4]=tmpA;
	memSec[5]=tmpS;
      }
      else if(tmpA+reqS != memSec[2*(2+j)]) {
	j++;
	memSec[2*(2+j)+0]=tmpA;
	memSec[2*(2+j)+1]=tmpS;
      }
      else {
	memSec[2*(2+j)+0]=tmpA;
	memSec[2*(2+j)+1]+=tmpS;
      }

      k++;
      allocT+=tmpS;
      if((tmpS!=reqS) || (allocT >= allocTGT)) break;
    }
    if(memSec[2*(2+j)+0] != 0) j++;

    if(allocT < allocTGT) {
      printk(KERN_INFO "==>ERROR! could note allocate work area target\n");
      return 1;
    }
    else {
      // sort in descending size order:
      int secOrd[MEM_SEC_N];
      unsigned int memSecCP[MEM_SEC_N*2];
      for(k=0; k<MEM_SEC_N*2; k++) memSecCP[k]=memSec[k+4];

      k=0;
      while((memSecCP[k*2]!=0) && (k<MEM_SEC_N)) {
	if(k==0) secOrd[0]=0;
	else {
	  int p=0,q=0;
	  for(q=0; q<k; q++) if(memSecCP[k*2+1] < memSecCP[2*secOrd[q]+1]) p=q+1;
	  for(q=k-1; q>=p; q--) secOrd[q+1]=secOrd[q];
	  secOrd[p]=k;
	}
	k++;
      }
      if(k!=j) {
	printk(KERN_INFO "==>ERROR! mem sector sorting problem %d!=%d\n",k,j);
	return 1;
      }
      for(k=0; k<j; k++) {
	memSec[4+2*k]   = memSecCP[2*secOrd[k]];
	memSec[4+2*k+1] = memSecCP[2*secOrd[k]+1];
      }

      printk(KERN_INFO "==>INFOm0: FB0 allocated: addr=0x%08x size=0x%08x bytes\n",
	     memSec[0],memSec[1]);
      printk(KERN_INFO "==>INFOm1: FB1 allocated: addr=0x%08x size=0x%08x bytes\n",
	     memSec[2],memSec[3]);
      printk(KERN_INFO "==>INFOm2: Workmem %dMB allocated in %d sectors:\n", workmemMB,j);
      for(k=0; k<j; k++) {
	printk(KERN_INFO "==>  %02d: addr=0x%08x size=0x%08x bytes\n", k,
	       memSec[2*(2+k)],memSec[2*(2+k)+1]);
      }
      printk(KERN_INFO "\n");
    }
    return 0;
  }
}

static void memClose(void)
{
  //  int k;
  //for(k=0; k<(2+MEM_SEC_N); k++) {
  //if(memSec[2*k]!=0) kfree((unsigned char*)memSec[2*k]);
  //}
}

static int drm_open(struct inode *inode, struct file *file)
{
  struct drm_dev* drm_dev;
  struct dmp_dev* subdev;
  int ret = 0;
  unsigned int minor;
	
  minor = iminor(inode);
  drm_dev = container_of(inode->i_cdev, struct drm_dev, cdev);
  subdev = &drm_dev->subdev[minor];
  file->private_data = subdev;

  return ret;
}

static int drm_release(struct inode *inode, struct file *file)
{
  return 0;
}

static long drm_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
  long ret = 0;
  long res = 0;
  unsigned long usrA;
  int k;
  struct dmp_dev* subdev = file->private_data;

  //printk(KERN_INFO "IOCTL cmd=%d\n", cmd);
  switch(cmd) {
  case CNV_WAITINT: {
    waitInt_c(subdev);
    break;
  }
  case CNV_WAITPDC: {
    res = waitInt_p(subdev);
    ret = put_user(res, (unsigned int __user*)arg);
    if (ret) ret = -EFAULT;
    break;
  }
  case CNV_MEMSEC: {
    usrA = arg;
    for(k=0; (ret!=-EFAULT) && (k<MEM_SEC_N); k++) {
      ret = put_user(memSec[(2+k)*2], (unsigned int __user*)usrA);
      if (ret) ret = -EFAULT;
      else {
	ret = put_user(memSec[(2+k)*2+1], (unsigned int __user*)(usrA+4));
	if (ret) ret = -EFAULT;
      }      
      if(memSec[(2+k)*2]==0) break;
      usrA+=8;
    }
  }

    //case CNV_FB0ADDR_PDC: {
    //res = pdc_get_fb0_addr();
    //ret = put_user(res, (unsigned int __user*)arg);
    //if (ret) ret = -EFAULT;
    //break;
    //}
  default: break;
  }

  return ret;
}

static int drm_mmap(struct file *file, struct vm_area_struct *vma)
{
  int err = 0;
  struct dmp_dev* subdev;
  unsigned long map_size;
  unsigned long pfn;

  subdev = file->private_data;
  map_size = vma->vm_end - vma->vm_start;
  if (subdev->bar_size < map_size) {
    err = -EINVAL;
    goto fail_size;
  }

  vma->vm_flags |= VM_IO  | VM_DONTCOPY | VM_DONTEXPAND | VM_SHARED;
  vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

  pfn = subdev->bar_physical >> PAGE_SHIFT;
  err = io_remap_pfn_range(vma, vma->vm_start, pfn, map_size, vma->vm_page_prot);
  if (err) {
    return -EAGAIN;
    goto fail_ioremap;
  }

  return  0;

fail_ioremap:
fail_size:
  return err;
}


static struct file_operations drm_file_operations =
{
  .owner	   = THIS_MODULE,
  .open		   = drm_open,
  .release	   = drm_release,
  .unlocked_ioctl  = drm_ioctl,
  .mmap		   = drm_mmap,
};


int drm_register_chrdev(struct drm_dev* drm_dev)
{
  int err=0, rIRQ=0, i=0;
  int pdcDims[5];
  unsigned int fbPA[2];
  struct device	*dev;
  unsigned int driver_major;

  dev_dbg(drm_dev->dev, "drm_register_chrdev\n");

  err = alloc_chrdev_region(&drm_dev->devt, 0, DRM_NUM_SUBDEV, DRM_DEV_NAME);
  if (err) {
    dev_err(drm_dev->dev, "alloc_chrdev_region fail\n");
    goto fail_alloc_chrdev_region;
  }

  dddrm_class = class_create(THIS_MODULE, DRM_DEV_NAME);
  if (IS_ERR(dddrm_class)) {
    err = PTR_ERR(dddrm_class);
    dev_err(drm_dev->dev, "class_create fail\n");
    goto fail_class_create;
  }
	
  driver_major = MAJOR(drm_dev->devt);

  // create char device:
  cdev_init(&drm_dev->cdev, &drm_file_operations);
  err = cdev_add(&drm_dev->cdev, drm_dev->devt, DRM_NUM_SUBDEV);
  if (err) {
    dev_err(drm_dev->dev, "cdev_add fail\n");
    goto fail_cdev_add;
  }
  
  for(i=0; i<DRM_NUM_SUBDEV; i++) {
    // Create device:
    dev = device_create(dddrm_class, NULL, MKDEV(driver_major, i), drm_dev, DRM_DEV_NAME"%d", i);
    if (IS_ERR(dev)) {
      err = PTR_ERR(dev);
      dev_err(drm_dev->dev, "device_create fail %d\n", i);
      goto fail_device_init;
    }

    rIRQ = drm_dev->subdev[i].irqno;
    if (i==0) {
      if (!err) err = request_irq(rIRQ, handleInt_c,
				  IRQF_SHARED, DRM_DEV_NAME, &(drm_dev->subdev[i]));
    }
    else if(i==1) {
      if (!err) err = request_irq(rIRQ, handleInt_p,
				  IRQF_SHARED, DRM_DEV_NAME, &(drm_dev->subdev[i]));
    }
    else {
      if (!err) err = request_irq(rIRQ, handleInt_f,
				  IRQF_SHARED, DRM_DEV_NAME, &(drm_dev->subdev[i]));
    }

    if (err) {
      device_destroy(dddrm_class, MKDEV(driver_major, i));
      dev_err(drm_dev->dev, "request_irq FAIL: IRQ=%d ERR=%d\n", rIRQ, err);
      goto fail_device_init;
    }
    
    // initialize:
    init_waitqueue_head(&(drm_dev->subdev[i].int_status_wait));
    spin_lock_init(&(drm_dev->subdev[i].int_exclusive));
    drm_dev->subdev[i].initDone= 1;
  }

  if(srcW==0) srcW=width;
  if(srcH==0) srcH=height;
  pdcDims[0]=width;
  pdcDims[1]=height;
  pdcDims[2]=srcW;
  pdcDims[3]=srcH;
  pdcDims[4]=pol;

  if (memSetup(drm_dev)) {
      dev_err(drm_dev->dev, "memSetup failed\n");
      goto fail_device_init;
  }

  fbPA[0]=memSec[0];
  fbPA[1]=memSec[2];

  pdc_config((void __iomem*)drm_dev->subdev[1].bar_logical, pdcDims, fbPA);
  if (pStart) pdc_start((void __iomem*)drm_dev->subdev[1].bar_logical);

  return 0;

fail_device_init:
  for(i=0; i<DRM_NUM_SUBDEV; i++) {
    if (drm_dev->subdev[i].initDone) {
      free_irq(drm_dev->subdev[i].irqno, &(drm_dev->subdev[i]));
      device_destroy(dddrm_class, MKDEV(driver_major, i));
      drm_dev->subdev[i].initDone=0;
    }
  }
  cdev_del(&drm_dev->cdev);
fail_cdev_add:
  class_destroy(dddrm_class);
fail_class_create:
  unregister_chrdev_region(drm_dev->devt, DRM_NUM_SUBDEV);
fail_alloc_chrdev_region:

  return err;
}

int drm_unregister_chrdev(struct drm_dev* drm_dev)
{
  int i;
  unsigned int driver_major = MAJOR(drm_dev->devt);
  dev_dbg(drm_dev->dev, "drm_unregister_chrdev\n");

  for(i=0; i<DRM_NUM_SUBDEV; i++) {
    if (drm_dev->subdev[i].initDone) {
      free_irq(drm_dev->subdev[i].irqno, &(drm_dev->subdev[i]));
      device_destroy(dddrm_class, MKDEV(driver_major, i));
      drm_dev->subdev[i].initDone=0;
    }
  }

  pdc_stop((void __iomem *)drm_dev->subdev[1].bar_logical);
  memClose();

  cdev_del(&drm_dev->cdev);
  class_destroy(dddrm_class);
  unregister_chrdev_region(drm_dev->devt, DRM_NUM_SUBDEV);
  return 0;
}

static void drm_dev_release(struct device *dev)
{
}

static struct platform_device drm_platform_device = {
  .name = DRM_DEV_NAME,
  .id = -1,
  .num_resources = 0,
  .resource = NULL,
  .dev = {
    .release = drm_dev_release,
  },
};

static int drm_dev_probe(struct platform_device *pdev)
{
  int i=0,err=0;
  struct drm_dev *drm_dev;  
#ifdef USE_DEVTREE
  struct device_node* devNode;
#endif	

  dev_dbg(&pdev->dev, "probe begin\n");
  drm_dev = kzalloc(sizeof(struct drm_dev), GFP_KERNEL);
  if (!drm_dev) {
    err = -ENOMEM;
    dev_err(&pdev->dev, "kzalloc fail\n");
    goto fail_probe_kzalloc;
  }
	
  platform_set_drvdata(pdev, drm_dev);
  drm_dev->dev = &pdev->dev;

#ifdef USE_DEVTREE
  devNode = of_find_compatible_node(NULL, NULL, "DMP_drm,DMP_drm");
  if(devNode == NULL) {
    err = -ENODEV;
    dev_err(&pdev->dev, "no compatible node!\n");
    goto fail_get_iomap;
  }
  else {
    of_node_put(devNode);
  }
#endif
	
  for(i=0 ; i<DRM_NUM_SUBDEV; i++) {
#ifdef USE_DEVTREE
    drm_dev->subdev[i].irqno = of_irq_get(devNode, sd2iMap[i]);
    // NOTE: we could(should) get the reg. address by this method too ...
#else
    drm_dev->subdev[i].irqno = sd2iMap[i];
#endif

    drm_dev->subdev[i].bar_physical = sd2rb[i];
    drm_dev->subdev[i].bar_size = sd2rs[i];
    drm_dev->subdev[i].bar_logical = ioremap_nocache(drm_dev->subdev[i].bar_physical,
						     drm_dev->subdev[i].bar_size);
    if(!drm_dev->subdev[i].bar_logical) {
      err = -EBUSY;
      dev_err(&pdev->dev, "ioremap_nocache fail %d\n", i);
      goto fail_get_iomap;
    }

    drm_dev->subdev[i].initDone = 0;
    drm_dev->subdev[i].int_status = 0;
    drm_dev->subdev[i].type = 0;   // unused 
    drm_dev->subdev[i].bank = 1;
  }

  err = drm_register_chrdev(drm_dev);
  if (err) {
    dev_err(&pdev->dev, "register chrdev fail\n");
    goto fail_register_chrdev;
  }	
  dev_dbg(&pdev->dev, "probe successful\n");

  return 0;

fail_register_chrdev:

fail_get_iomap:
  for (i = 0; i < DRM_NUM_SUBDEV; i++) {
    if (drm_dev->subdev[i].bar_logical) {
      iounmap(drm_dev->subdev[i].bar_logical);
    }
  }
  platform_set_drvdata(pdev, NULL);
  kfree(drm_dev);

fail_probe_kzalloc:
  return err;
}

static int drm_dev_remove(struct platform_device *pdev)
{
  struct drm_dev *drm_dev;
  int i;

  dev_dbg(&pdev->dev, "remove begin\n");
  drm_dev = platform_get_drvdata(pdev);
	
  if (drm_dev) {
    drm_unregister_chrdev(drm_dev);
    for(i=0; i<DRM_NUM_SUBDEV; i++) {
      if (drm_dev->subdev[i].bar_logical) {
	iounmap(drm_dev->subdev[i].bar_logical);
      }
    }

    platform_set_drvdata(pdev, NULL);
    kfree(drm_dev);
  }

  dev_dbg(&pdev->dev, "remove successful\n");
  return 0;
}


static struct platform_driver drm_platform_driver =
{
  .probe  = drm_dev_probe,
  .remove = drm_dev_remove,
  .driver = {
    .name  = DRM_DEV_NAME,
    .owner = THIS_MODULE,
  },
};


static int __init drm_init(void)
{
  int ret;  
  ret = platform_driver_register(&drm_platform_driver);
  if (ret) return ret;

  ret = platform_device_register(&drm_platform_device);
  if (ret) {
    platform_driver_unregister(&drm_platform_driver);
  }
  return ret;
}

static void __exit drm_exit(void)
{
  platform_device_unregister(&drm_platform_device);
  platform_driver_unregister(&drm_platform_driver);
}

module_init(drm_init);
module_exit(drm_exit);

MODULE_DESCRIPTION("CNV int controller");
MODULE_AUTHOR("Digital Media Professionals Inc.");
MODULE_VERSION("1.0");
MODULE_LICENSE("GPL");
