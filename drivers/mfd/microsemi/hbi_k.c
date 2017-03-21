/*
* hbi_k.c - kernel space driver for hbi over linux platform
*
* Copyright (c) 2016, Microsemi Corporation
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*     * Redistributions of source code must retain the above copyright
*       notice, this list of conditions and the following disclaimer.
*     * Neither the name of the <organization> nor the
*       names of its contributors may be used to endorse or promote products
*       derived from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
* DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/


#include "typedefs.h"
#include "chip.h"
#include "hbi.h"
#include "hbi_k.h"

#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/module.h>
#include <linux/version.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/slab.h>
#include <asm/uaccess.h>
#include <linux/proc_fs.h>


#define HBI_DEV_NAME "hbi" /* TODO: change it to "zl380xx" ?*/

#define HBI_DRV_MINOR_NUM 1 
#define HBI_DRV_DEV_CNT   1 /* TODO: Take this from configuration file */

/* Take these defines from chip specific make options instead */
#define HBI_CFGREC_MAX_SIZE     0xE00
#define HBI_CFGREC_BASE         0x200
#define HBI_LNX_BUF_MAX_LEN     (512*1024)/*TODO: can be made reconfigurable through config file */

#define HBI_LNX_DBG(msg, args...) printk(KERN_DEBUG"%s:%d "msg,__FUNCTION__,__LINE__,##args);


#define HBI_CMD(dev,cmd,args)    HBI_set_command((dev->hbi_handle),cmd,args)

#if (HBI_ENABLE_PROCFS)
ssize_t hbi_proc_reg_rd_dump(struct file *,  char __user *, size_t , loff_t *);
ssize_t hbi_proc_reg_rd(struct file *filp, const char __user *buf, size_t size, loff_t *offset);
ssize_t hbi_proc_reg_wr(struct file *filp, const char __user *buf, size_t size, loff_t *offset);
ssize_t hbi_proc_open_dev_wr(struct file *filp, const char __user *buf, size_t size, loff_t *offset);
ssize_t hbi_proc_open_dev_rd(struct file *filp,  char __user *buf, size_t size, loff_t *offset);
ssize_t hbi_proc_close_dev_wr(struct file *filp, const char __user *buf, size_t size, loff_t *offset);
ssize_t hbi_proc_wr_cfgrec(struct file *filp, const char __user *buf, size_t size, loff_t *offset);
ssize_t hbi_proc_rd_cfgrec(struct file *filp,  char __user *buf, size_t size, loff_t *offset);
ssize_t hbi_proc_load_fw(struct file *filp, const char __user *buf, size_t size, loff_t *offset);
ssize_t hbi_proc_start_fw(struct file *filp,  char __user *buf, 
                           size_t size, loff_t *offset);
ssize_t hbi_proc_save_fwrcfgrec_to_flash(struct file *filp,  char __user *buf, size_t size, loff_t *offset);
ssize_t hbi_proc_load_fwrcfgrec_from_flash(struct file *filp, const char __user *buf, size_t size, loff_t *offset);
ssize_t hbi_proc_erase_img_from_flash(struct file *filp, const char __user *buf, size_t size, loff_t *offset);
ssize_t hbi_proc_erase_flash(struct file *filp,  char __user *buf, size_t size, loff_t *offset);
ssize_t hbi_proc_drv_init(struct file *filp,  char __user *buf, size_t size, loff_t *offset);
ssize_t hbi_proc_drv_term(struct file *filp,  char __user *buf, size_t size, loff_t *offset);

struct hbi_lnx_drv_proc_entries {
    const char *name;
    umode_t    mode;
    struct proc_dir_entry *proc_entry;
    struct file_operations ops;
};
typedef enum
{
   HBI_PROC_ENTRY_INIT_DRV,
   HBI_PROC_ENTRY_TERM_DRV,
   HBI_PROC_ENTRY_OPEN_DEV,
   HBI_PROC_ENTRY_CLOSE_DEV,
   HBI_PROC_ENTRY_LAST,
   HBI_PROC_ENTRY_FIRST = HBI_PROC_ENTRY_INIT_DRV
}HBI_PROC_ENTRIES;

typedef enum
{
    HBI_DEV_PROC_ENTRY_READ_REG,
    HBI_DEV_PROC_ENTRY_FIRST=HBI_DEV_PROC_ENTRY_READ_REG,
    HBI_DEV_PROC_ENTRY_WRITE_REG,
    HBI_DEV_PROC_ENTRY_CFGREC,
    HBI_DEV_PROC_ENTRY_LOAD_FW,
    HBI_DEV_PROC_ENTRY_START_FW,
#ifdef FLASH_PRESENT
    HBI_DEV_PROC_ENTRY_FLASH_SAVE_FWCFGREC,
    HBI_DEV_PROC_ENTRY_FLASH_LOAD_FWCFGREC,
     HBI_DEV_PROC_ENTRY_FLASG_ERASE,
#endif
    HBI_DEV_PROC_ENTRY_LAST
}HBI_DEV_PROC_ENTRIES;

/* device specific proc entries */
struct hbi_lnx_drv_proc_entries dev_proc_entry[] = 
{
    {
        .name = "read_reg",
        .mode = (S_IFREG|S_IRUGO|S_IWUGO|S_IRWXU),
        .proc_entry = NULL,
        .ops = {
            .owner = THIS_MODULE,
            .write = hbi_proc_reg_rd,
            .read = hbi_proc_reg_rd_dump
        }
    },
    {
        .name = "write_reg",
        .mode = (S_IFREG|S_IRUGO|S_IWUGO),
        .proc_entry = NULL,
        .ops = {
            .owner = THIS_MODULE,
            .write = hbi_proc_reg_wr,
        }
    },
    {
        .name = "cfgrec",
        .mode = (S_IFREG|S_IRUGO|S_IWUGO),
        .proc_entry = NULL,
        .ops = {
            .owner = THIS_MODULE,
            .write = hbi_proc_wr_cfgrec,
            .read = hbi_proc_rd_cfgrec
        }
    },
    {
        .name = "load_fw",
        .mode = (S_IFREG|S_IRUGO|S_IWUGO),
        .proc_entry = NULL,
        .ops = {
        .owner = THIS_MODULE,
        .write = hbi_proc_load_fw,
        }
    },
    {
        .name = "start_fw",
        .mode = (S_IFREG|S_IRUGO|S_IWUGO),
        .proc_entry = NULL,
        .ops = {
        .owner = THIS_MODULE,
        .read = hbi_proc_start_fw,
        }
    },
    #ifdef FLASH_PRESENT
    {
        .name = "flash_save_fwrcfgrec",
        .mode = (S_IFREG|S_IRUGO|S_IWUGO),
        .proc_entry = NULL,
        .ops = {
        .owner = THIS_MODULE,
        .read = hbi_proc_save_fwrcfgrec_to_flash,
        }
    },
    {
        .name = "flash_load_fwrcfgrec",
        .mode = (S_IFREG|S_IRUGO|S_IWUGO),
        .proc_entry = NULL,
        .ops = {
        .owner = THIS_MODULE,
        .write = hbi_proc_load_fwrcfgrec_from_flash,
        }
    },
    {
        .name = "flash_erase",
        .mode = (S_IFREG|S_IRUGO|S_IWUGO),
        .proc_entry = NULL,
        .ops = {
        .owner = THIS_MODULE,
        .write = hbi_proc_erase_img_from_flash,
        .read = hbi_proc_erase_flash,
        }
    },
    #endif
};

/* HBI Driver Procfs entry */
struct hbi_lnx_drv_proc_entries drv_proc_entry[] = {
   {
        .name = "init_driver",
        .proc_entry = NULL,
        .ops = {
            .owner = THIS_MODULE,
            .read = hbi_proc_drv_init
        }
    },
    {
        .name = "term_driver",
        .proc_entry = NULL,
        .ops = {
            .owner = THIS_MODULE,
            .read = hbi_proc_drv_term
        }
    },
    {
        .name = "open_device",
        .proc_entry = NULL,
        .ops = {
            .owner = THIS_MODULE,
            .write = hbi_proc_open_dev_wr,
            .read = hbi_proc_open_dev_rd
        }
    },
    { 
        .name = "close_device",
        .proc_entry = NULL,
        .ops = {
            .owner = THIS_MODULE,
            .write = hbi_proc_close_dev_wr,
        }
    }
};
static unsigned max_rw_size=ZL380xx_MAX_ACCESS_SIZE_IN_BYTES;
struct {
   unsigned char       buf[ZL380xx_MAX_ACCESS_SIZE_IN_BYTES];/* buffer to store reg_rd transaction */
   size_t        len; /* length of data read*/
}hbi_rw;

module_param(max_rw_size,uint,S_IRUGO);
MODULE_PARM_DESC(max_rw_size,"maximum size of 1 read/write transaction\n");

#endif /* HBI_ENABLE_PROCFS */


struct dev_cfgrec
{
    int size;
};

typedef struct{
    unsigned char  buf[HBI_LNX_BUF_MAX_LEN];
    int32_t  len;
}HBI_LNX_BUF;

struct hbi_dev{
    hbi_handle_t            hbi_handle;/* HBI handle of current device */
    hbi_dev_cfg_t             devcfg;    /* user passed device configuration */
    struct list_head        list;       /* pointer to next opened device */
    struct dev_cfgrec       cfgrec;
    HBI_LNX_BUF             hbi_buf;
#if (HBI_ENABLE_PROCFS)
    struct proc_dir_entry   *dev_proc_entry[HBI_DEV_PROC_ENTRY_LAST];
    struct proc_dir_entry   *dev_proc_dir;
#endif /*HBI_ENABLE_PROCFS */
};

struct hbi_lnx_drv
{
    struct hbi_dev      *dev; /* pointer to list of opened devices */
    struct cdev         *cdev; /* pointer to HBI character device driver correspond to */
    dev_t                dev_t; /* device number of registered char devices */
    struct class        *dev_class;/* pointer to device class this driver correspond to */
    struct device       *device; /* pointer to device file of this driver */
#if (HBI_ENABLE_PROCFS)
    struct proc_dir_entry *drv_proc_dir;
#endif
}hbi_lnx_drv_priv;


int internal_hbi_drv_open (struct inode *node, struct file *filp)
{
    return 0;
}

static hbi_status_t internal_hbi_open(void *device,hbi_dev_cfg_t *devcfg)
{
    struct hbi_dev *dev=NULL;
    hbi_status_t      status;
#if (HBI_ENABLE_PROCFS)
    int             i;
    uint8_t         dir_name[32];
#endif    
    HBI_LNX_DBG("Opening device with addr : 0x%x bus num %d\n",devcfg->dev_addr,devcfg->bus_num);
 
    dev = kmalloc(sizeof(struct hbi_dev),GFP_KERNEL);
    if(dev==NULL)
    {
        printk("Resource Error \n");
        return -1;
    }
 
    memset(dev,0,sizeof(struct hbi_dev));
    memcpy(&(dev->devcfg),devcfg,sizeof(hbi_dev_cfg_t));
    
    status = HBI_open(&(dev->hbi_handle),&(dev->devcfg));
    if(status !=  HBI_STATUS_SUCCESS)
    {
        printk("HBI_open failed\n");
        kfree(dev);
        return HBI_STATUS_RESOURCE_ERR;
    }

    INIT_LIST_HEAD(&(dev->list));
 
    if(hbi_lnx_drv_priv.dev == NULL)
    {
        hbi_lnx_drv_priv.dev = dev;
    }
    else
    {
        list_add(&(dev->list),&(hbi_lnx_drv_priv.dev->list));
    }
    *((struct hbi_dev **)device) = dev;
#if (HBI_ENABLE_PROCFS)
    /* create a proc entry for each device */
    memset(dir_name,0,sizeof(dir_name));
    sprintf(dir_name,"dev_%x%x",dev->devcfg.bus_num,dev->devcfg.dev_addr);

    dev->dev_proc_dir = proc_mkdir(dir_name,hbi_lnx_drv_priv.drv_proc_dir);
    for(i=HBI_DEV_PROC_ENTRY_FIRST;i<HBI_DEV_PROC_ENTRY_LAST;i++)
    {
        HBI_LNX_DBG("Creating Device 0x%x Proc Entry %s\n",(unsigned int)dev,dev_proc_entry[i].name);

        dev->dev_proc_entry[i] = proc_create_data(dev_proc_entry[i].name,
                                                  dev_proc_entry[i].mode,
                                                  dev->dev_proc_dir,
                                                  &(dev_proc_entry[i].ops),
                                                  (void *)dev);
    }
#endif /* HBI_ENABLE_PROCFS */
    return HBI_STATUS_SUCCESS;
}

static hbi_status_t internal_hbi_close(struct hbi_dev *device)
{
    hbi_status_t      status = HBI_STATUS_SUCCESS;
#if (HBI_ENABLE_PROCFS)
    int i;
#endif
    if(device == NULL)
    {
        HBI_LNX_DBG("NULL device handle passed\n");
        return HBI_STATUS_BAD_HANDLE;
    }

    if((status = HBI_close(device->hbi_handle)) != HBI_STATUS_SUCCESS)
    {
        HBI_LNX_DBG("Failed to close device\n");
        return status;
    }

    /* if list isn't empty and this is 1st device in list, adjust head node ?*/
    if(!list_empty(&(hbi_lnx_drv_priv.dev->list)) && 
         (device == hbi_lnx_drv_priv.dev))
        hbi_lnx_drv_priv.dev = list_next_entry(device,list);
    else if(list_empty(&(hbi_lnx_drv_priv.dev->list)))
        hbi_lnx_drv_priv.dev = NULL;

    HBI_LNX_DBG("list head 0x%x\n",(unsigned int)(hbi_lnx_drv_priv.dev));


    list_del(&(device->list));
#if (HBI_ENABLE_PROCFS)
    for(i=HBI_DEV_PROC_ENTRY_FIRST;i<HBI_DEV_PROC_ENTRY_LAST;i++)
    {
        if(device->dev_proc_entry[i] != NULL)
        {
            proc_remove(device->dev_proc_entry[i]);
            device->dev_proc_entry[i] = NULL;
        }
    }
    if(device->dev_proc_dir != NULL)
    {
        proc_remove(device->dev_proc_dir);
        device->dev_proc_dir = NULL;
    }
#endif /* HBI_ENABLE_PROCFS */
    kfree(device);

    return status;
}


#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0)
long  internal_hbi_drv_ioctl(
#else
int internal_hbi_drv_ioctl(struct inode *inode,
#endif 
                         struct file *filp, unsigned int cmd, unsigned long pArgs)
{
    int ret=0;
    struct hbi_dev *dev=NULL;

    switch(cmd)
    {
        case HBI_INIT:
        {
           hbi_lnx_drv_cfg_t cfg;
           hbi_init_cfg_t    kCfg;

           printk("HBI Init called\n");

           ret = copy_from_user((void *)&cfg,
                                (const void __user *)pArgs,
                                sizeof(hbi_lnx_drv_cfg_t));
           if(ret)
           {
             HBI_LNX_DBG("copy_from_user failed\n");
             return -1;
           }
           if(cfg.pCfg != NULL)
           {
              ret = copy_from_user((void *)&kCfg,
                                   (const void __user *)(cfg.pCfg),
                                   sizeof(hbi_init_cfg_t));
           }
           cfg.status = HBI_init(&(kCfg));

           ret = copy_to_user((void __user *)pArgs,
                              (const void *)&cfg,
                              sizeof(hbi_lnx_drv_cfg_t));
           if(ret)
           {
             HBI_LNX_DBG("copy_to_user failed\n");
             return -1;
           }

           return ret;
        }
        case HBI_TERM:
        {
            hbi_lnx_drv_term_arg_t termArg;

            ret = copy_from_user((void *)&termArg,
                                 (const void *__user)pArgs,
                                  sizeof(hbi_lnx_drv_term_arg_t));
            if(ret)
            {
               HBI_LNX_DBG("copy_from_user failed\n");
               return -1;
            }

            termArg.status = HBI_term();

            ret = copy_to_user((void* __user)pArgs,
                               (const void *)&termArg,
                               sizeof(termArg));
            if(ret)
            {
               HBI_LNX_DBG("copy_to_user failed\n");
               return -1;
            }
            return 0;
        }
        case HBI_OPEN:
        {
            hbi_lnx_open_arg_t openArg;
            hbi_dev_cfg_t devcfg;

            printk(" HBI_OPEN called\n");

            if(!pArgs)
            {
                HBI_LNX_DBG("NULL param passed\n");
                return -EINVAL;
            }

            ret = copy_from_user(&openArg,
                                 (const void __user *)pArgs,
                                 sizeof(hbi_lnx_open_arg_t));
            if(ret)
            {
                HBI_LNX_DBG("copy_from_user failed\n");
                return -1;
            }

            if(openArg.pDevCfg == NULL ||
               (openArg.pHandle == NULL))
            {
               HBI_LNX_DBG("NULL device config pointer passed \n");
               return -EINVAL;
            }

            ret = copy_from_user(&devcfg,
                                 (const void __user*)(openArg.pDevCfg),
                                 sizeof(hbi_dev_cfg_t));
            if(ret)
            {
                HBI_LNX_DBG("copy_from_user failed\n");
                return -1;
            }

            HBI_LNX_DBG("Opening device 0x%x\n",devcfg.dev_addr);

            openArg.status = internal_hbi_open((void *)&dev,&devcfg);
            
            printk("opened device 0x%x\n",(unsigned int)dev);

            
            *(openArg.pHandle) = (hbi_handle_t )dev;
            
            ret = copy_to_user((void __user*)pArgs,
                               (const void *)&openArg,
                               sizeof(hbi_lnx_open_arg_t));
                               
            
            if(openArg.status == HBI_STATUS_SUCCESS)
                return 0;
            else
                return -1;
        }
        case HBI_CLOSE:
        {
            hbi_lnx_close_arg_t closeArg;

            if(!pArgs)
            {
                HBI_LNX_DBG("NULL param passed\n");
                return -EINVAL;
            }

            ret = copy_from_user((void *)&closeArg,
                                 (const void* __user)pArgs,
                                 sizeof(hbi_lnx_close_arg_t));
            
            dev = (struct hbi_dev *)closeArg.handle;
            
            printk("Received handle 0x%x\n",(unsigned int)dev);
            
            closeArg.status = internal_hbi_close(dev);
            
            ret = copy_to_user((void *__user)pArgs,
                                (const void *)&closeArg,
                                sizeof(hbi_lnx_close_arg_t));

            if(ret)
               return -1;
               
            return 0;
        }
        case HBI_READ:
        case HBI_WRITE:
        {
            hbi_lnx_drv_rw_arg_t args;
             /* TODO: 256 some magical number took to avoid malloc n free 
                  for small bytes read.can be changed or set to max limit
             */
            uint8_t buffer[256];
            uint8_t *tmp = NULL;
            int      bmalloced = FALSE;
            int i;
            
            if(!pArgs)
            {
                HBI_LNX_DBG("NULL param passed\n");
                return -EINVAL;
            }
            
            ret = copy_from_user(&args,
                                (const void __user *)pArgs,
                                sizeof(hbi_lnx_drv_rw_arg_t));
            if(ret)
            {
                HBI_LNX_DBG("copy_from_user failed!!\n");
                return -EAGAIN;
            }
            
            dev = (struct hbi_dev *)(args.handle);
            
            /* use static for read length up to 256 */
            tmp = buffer;
            if(args.len > sizeof(buffer))
            {
                tmp = NULL;
                tmp = kmalloc(args.len, GFP_KERNEL);
                if(tmp == NULL)
                {
                    args.status = HBI_STATUS_RESOURCE_ERR;
                    ret = copy_to_user((void __user*)pArgs,
                                       (void *)&args,
                                       sizeof(hbi_lnx_drv_rw_arg_t));
                    return -EFAULT;
                }
                bmalloced = TRUE;
            }

            if(cmd == HBI_READ)
            {
                HBI_LNX_DBG("Calling HBI_read reg 0x%x , length %d\n",
                        args.reg,args.len);

                args.status = HBI_read((dev->hbi_handle),
                                       args.reg,
                                       tmp,
                                       (size_t)(args.len));

                if(args.status == HBI_STATUS_SUCCESS)
                {
                    /*copy read buffer */
                    args.len = copy_to_user(args.pData,tmp,args.len);
                }
            }
            else
            {
               HBI_LNX_DBG("Calling HBI_write reg 0x%x , length %d\n",
                       args.reg,args.len);

                ret = copy_from_user((void *)tmp,
                                          (const void * __user)(args.pData),
                                          args.len);
               if(!ret)
               {
                  HBI_LNX_DBG("copied %d bytes ..\n",args.len);
                  for (i=0;i<args.len;i++)
                  {
                     HBI_LNX_DBG("0x%x ",tmp[i]);
                  }

                  args.status = HBI_write(dev->hbi_handle,
                                         args.reg,
                                         tmp,
                                         args.len);
              }
              else
              {
                  HBI_LNX_DBG("copy_from_user failed\n");
              }
            }

            if(bmalloced && tmp)
            {
                kfree(tmp);
                tmp=NULL;
            }
            ret = copy_to_user((void __user *)pArgs,
                                 &args,
                                 sizeof(hbi_lnx_drv_rw_arg_t));
            if(ret)
            {
               HBI_LNX_DBG("copy_to_user failed\n");
               return -1;
            }
            return 0;
        }
        case HBI_LOAD_FW:
        {
            hbi_lnx_send_data_arg_t args;
            hbi_data_t              data;

            if(!pArgs)
            {
                HBI_LNX_DBG("NULL ioctl arg passed\n");
                return -EFAULT;
            }
            
            
            ret = copy_from_user(&args,
                                 (const void __user*)pArgs,
                                 sizeof(hbi_lnx_send_data_arg_t));
            if(ret)
            {
                HBI_LNX_DBG("copy_from_user failed\n");
                return -EAGAIN;
            }

            dev = (struct hbi_dev *)(args.handle);

            HBI_LNX_DBG("copy from user buffer 0x%x size %d\n",
                        (unsigned int)(args.data.pData),
                        (int)(args.data.size));


            if(sizeof(dev->hbi_buf.buf) < args.data.size)
            {
                HBI_LNX_DBG("insufficient buffer\n");
                args.status = HBI_STATUS_RESOURCE_ERR;
                if(copy_to_user((void __user*)pArgs,
                              &args,
                              sizeof(hbi_lnx_send_data_arg_t)))
                {
                  HBI_LNX_DBG("copy_to_user performed an incomplete copy\n");
                }
                return -ENOMEM;
            }

            memcpy((dev->hbi_buf.buf),args.data.pData,args.data.size);

            data.pData = (dev->hbi_buf.buf);
            data.size = args.data.size;
            
            args.status = HBI_CMD(dev,HBI_CMD_LOAD_FWR_FROM_HOST,&data);



            ret = copy_to_user((void __user *)pArgs,
                                 &args,
                                 sizeof(hbi_lnx_send_data_arg_t));
            return ret;
        }
        case HBI_LOAD_FW_COMPLETE:
        {
            hbi_lnx_ldfw_done_arg_t args;

            ret = copy_from_user((void *)&args,
                               (const void* __user)pArgs,
                               sizeof(hbi_lnx_ldfw_done_arg_t));

            dev = (struct hbi_dev *)(args.handle);

            args.status = HBI_CMD(dev,HBI_CMD_LOAD_FWR_COMPLETE,NULL);

            ret = copy_to_user((void * __user)pArgs,
                               (const void*)&args,
                               sizeof(hbi_lnx_ldfw_done_arg_t));

            return ret;
        }
        case HBI_START_FW:
        {
            hbi_lnx_start_fw_arg_t  args;
            if(!pArgs)
            {
                HBI_LNX_DBG("NULL ioctl arg passed\n");
                return -EFAULT;
            }

            ret = copy_from_user(&args,
                                 (const void __user *)pArgs,
                                 sizeof(hbi_lnx_start_fw_arg_t));
            if(ret)
            {
                HBI_LNX_DBG("copy_from_user failed\n");
                return -EAGAIN;
            }
            dev = (struct hbi_dev *)(args.handle);
            args.status = HBI_CMD(dev,HBI_CMD_START_FWR,NULL);
            ret = copy_to_user((void __user *)pArgs,
                                 &args,
                                 sizeof(hbi_lnx_start_fw_arg_t));
            return ret;
        }
        case HBI_FLASH_SAVE_FWR_CFGREC:
        {
#ifdef FLASH_PRESENT
            hbi_lnx_flash_save_fwrcfg_arg_t  args;
            
            if(!pArgs)
            {
                HBI_LNX_DBG("NULL ioctl arg passed\n");
                return -EFAULT;
            }

            ret = copy_from_user(&args,
                                 (const void __user*)pArgs,
                                 sizeof(hbi_lnx_flash_save_fwrcfg_arg_t));
            if(ret)
            {
                HBI_LNX_DBG("copy_from_user failed\n");
                return -EAGAIN;
            }

            dev = (struct hbi_dev *)(args.handle);

            args.status = HBI_CMD(dev,
                                  HBI_CMD_SAVE_FWRCFG_TO_FLASH,
                                  &(args.image_num));

            ret = copy_to_user((void __user *)pArgs,
                                 &args,
                                 sizeof(hbi_lnx_flash_save_fwrcfg_arg_t));
#endif
            return ret;
        }
        case HBI_FLASH_LOAD_FWR_CFGREC:
        {
#ifdef FLASH_PRESENT
            hbi_lnx_flash_load_fwrcfg_arg_t args;
            char image_num[2]={0};
            if(!pArgs)
            {
                HBI_LNX_DBG("NULL ioctl arg passed\n");
                return -EFAULT;
            }

            ret = copy_from_user(&args,
                                 (const void __user*)pArgs,
                                 sizeof(hbi_lnx_flash_load_fwrcfg_arg_t));
            if(ret)
            {
                HBI_LNX_DBG("copy_from_user failed\n");
                return -EAGAIN;
            }

            dev = (struct hbi_dev *)(args.handle);

            image_num[0] = (args.image_num >> 8) & 0xFF;
            image_num[1] = args.image_num & 0xFF;
            
            HBI_LNX_DBG("Writing image num %d\n",*((int *)image_num));
            
            args.status = HBI_CMD(dev,
                                 HBI_CMD_LOAD_FWRCFG_FROM_FLASH,
                                 image_num);

            ret = copy_to_user((void __user *)pArgs,
                              &args,
                              sizeof(hbi_lnx_flash_load_fwrcfg_arg_t));
#endif
                return ret;
            }
            case HBI_FLASH_ERASE_FWRCFGREC:
            case HBI_FLASH_ERASE_WHOLE:
            {
#ifdef FLASH_PRESENT
                hbi_lnx_flash_erase_fwcfg_arg_t args;
                char image_num[2];
                if(!pArgs)
                {
                    HBI_LNX_DBG("NULL ioctl arg passed\n");
                    return -EFAULT;
                }
            
                ret = copy_from_user(&args,
                                    (const void __user*)pArgs,
                                    sizeof(hbi_lnx_flash_erase_fwcfg_arg_t));
                if(ret)
                {
                    HBI_LNX_DBG("copy_from_user failed\n");
                    return -EAGAIN;
                }
            
                dev = (struct hbi_dev *)(args.handle);

                image_num[0]=(args.image_num >> 8) & 0xFF;
                image_num[1] = (args.image_num) & 0xFF;
                if(cmd == HBI_FLASH_ERASE_FWRCFGREC)
                    args.status = HBI_CMD(dev,
                                          HBI_CMD_ERASE_FWRCFG_FROM_FLASH,
                                          image_num);
                else
                    args.status = HBI_CMD(dev,HBI_CMD_ERASE_WHOLE_FLASH,NULL);

                ret = copy_to_user((void __user*)pArgs,
                                    &args,
                                    sizeof(hbi_lnx_flash_erase_fwcfg_arg_t));
#endif
                return ret;
            }
    }

    return ret;
}

struct file_operations fops = {
    .owner = THIS_MODULE,
    .open = internal_hbi_drv_open,
    .unlocked_ioctl = internal_hbi_drv_ioctl,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0)
    unlocked_ioctl:   internal_hbi_drv_ioctl,
#else
    ioctl  : internal_hbi_lnx_ioctl,
#endif
};


static inline int atoi(char *s,int size)
{
    uint32_t value=0;
    int i;
    for(i=0;i<size;i++)
    {
        if(s[i]>='0' && s[i]<='9')
        {
            value = (value * 10) + (s[i]-'0');
        }
    }
    return value;
}

static inline int atoh(char *s,int size)
{
    unsigned int val = 0;
    char c;
    unsigned char i = 0;
    for (i = 0; i< size; i++)
    {
        c = *s++; 
        if (c >= '0' && c <= '9')
        {
            val = (val << 4) +( c & 0x0F );
            continue;
        }
        c &= 0xDF;
        if (c >= 'A' && c <= 'F')
        {
            val = (val << 4) + ((c & 0x07) + 9);
            continue;
        }
    }
    return val;
}
#if (HBI_ENABLE_PROCFS)

int itoa(unsigned int num,unsigned char *buf, int index)
{
    unsigned int  tmp;
    int k;
    int len=0;

    tmp = num;

    if(!num)
        len++;
    else
    {
        while(tmp)
        {
            len++;
            tmp=tmp/10;
        }
    }

    tmp = len;

    while(len)
    {
        k = (len+index)-1;
        if(num < 10)
            buf[k] = num+'0';
        else
            buf[k] = (num%10)+'0';
        num = num/10;
        len--;
    }

    index+=tmp;

    return index;
}

int htoa(unsigned int num,unsigned char *buf, int index)
{
    unsigned int tmp;
    int k;
    int len=0;

    tmp = num;

    if(!num)
        len++;
    else
    {
        while(tmp)
        {
            len++;
            tmp=tmp/16;
        }
    }

    tmp = len;
    while(len)
    {
        k=(len+index)-1;
        if((num & 0xf) >= 10)
            buf[k]= ((num & 0xf)-10) + 'A';
        else
            buf[k] = (num & 0xf) + '0';
        len--;
        num >>=4;
    }
    index += tmp;

    return index;
}

static hbi_status_t hbi_wr_bin_fw(struct hbi_dev *dev,
                                 unsigned char *buf,
                                 size_t size,
                                 size_t block_size)
{
   hbi_status_t         status = HBI_STATUS_SUCCESS;
   int                  i=0;
   size_t               chunk_len;
   hbi_data_t           fwrimg;


   /* convert to bytes */
   chunk_len = block_size << 1;

   HBI_LNX_DBG("Retrieved Block size of %d\n",block_size);

   do
   {
     if(size  < chunk_len || !chunk_len)
     {
         printk("Incomplete data read.requested %d, read %d\n",chunk_len,size);
         return HBI_STATUS_RESOURCE_ERR;
     }

     fwrimg.pData = &buf[i];
     fwrimg.size = chunk_len;

     status = HBI_set_command(dev->hbi_handle,HBI_CMD_LOAD_FWR_FROM_HOST, &fwrimg);
     if ((status != HBI_STATUS_SUCCESS &&  status != HBI_STATUS_OP_INCOMPLETE)) 
      {
         printk("HBI_write failed\n");
         return status;
      }
      i+=chunk_len;
   }while(i<size);
   printk("2- Conclude boot image loading....\n");

   status = HBI_set_command((dev->hbi_handle),HBI_CMD_LOAD_FWR_COMPLETE,NULL);
   if (status != HBI_STATUS_SUCCESS) 
   {
     printk("Error %d:HBI_CMD_BOOT_COMPLETE failed!\n", status);
   }

   return status;
}

ssize_t hbi_proc_reg_rd_dump(struct file *filp,  
                              char __user *buf, size_t size, loff_t *offset)
{
    int i,j;

    unsigned int num;
    unsigned char kbuf[512];

    if(hbi_rw.len)
    {
        if(size > hbi_rw.len)
            size = hbi_rw.len;

        for(i=0,j=0; ((i<size) && (j<(sizeof(kbuf)-1))); i++)
        {
            num=hbi_rw.buf[i];
            printk("num %d, j %d\n",num,j);
            j=htoa(num,kbuf,j);
            kbuf[j++] = '\t';
        }

        kbuf[j]='\n';

        if(copy_to_user((void __user*)buf,kbuf,j))
        {
            /* issue just a warning */
           HBI_LNX_DBG("copy_to_user failed with incomplete copy\n");
        }
        

        printk(KERN_DEBUG"Read %d bytes to user buf\n",j);

        for(i=0;i<=j;i++)
            printk(KERN_DEBUG"%c\n",kbuf[i]);

        *offset+=j;
        hbi_rw.len -= size;

        return j;
    }

    printk(KERN_DEBUG"return EOF\n");
    return 0;
}

ssize_t hbi_proc_reg_rd(struct file *filp,
                        const char __user *buf, size_t size, loff_t *offset)
{
    struct hbi_dev *dev=NULL;
    uint8_t         val[max_rw_size];
    uint32_t        len;
    uint32_t        reg;
    hbi_status_t      status;
    int             i=0;

    dev = (struct hbi_dev *)(PDE_DATA(filp->f_inode));
    if(dev == NULL)
    {
        printk("No device opened\n");
        return -1;
    }
    
    /*expected format dev_addr reg size */
    if(copy_from_user((void *)val,buf,size))
    {
        HBI_LNX_DBG("Couldn't copy whole user buffer\n");
        return -1;
    }
    
    /* Get register to access */
    while(val[i++] != ' ');
    if(i >= size)
    {
        printk(KERN_CRIT"To read a register , give command "\
                        "'echo <register addr in hex> <number of bytes in" \
                        " dec(should be multiple of 2)>'\n");
        return -1;
    }

    reg = atoh(val,i-1);
    len = atoi(&val[i],size-i);

    if(len > sizeof(val))
    {
        printk("Cannot read more than %d bytes in one shot\n",sizeof(val));
        return -1;
    }

    HBI_LNX_DBG("Got dev_addr 0x%x register 0x%x" \
                  "len to read 0x%x dev handle 0x%x\n",
                  dev->devcfg.dev_addr,reg,len,(unsigned int)dev);

    memset(hbi_rw.buf,0,len);

    status = HBI_read((dev->hbi_handle),reg,hbi_rw.buf,len);
    if(status != HBI_STATUS_SUCCESS)
    {
        printk(KERN_CRIT"Err\n");
        return -1;
    }
    hbi_rw.len = len;
    return size;
}

ssize_t hbi_proc_reg_wr(struct file *filp, const char __user *buf, size_t size, loff_t *offset)
{
    struct hbi_dev *dev=NULL;
    uint8_t         val[max_rw_size];
    uint32_t        data_len=0;
    reg_addr_t        reg;
    hbi_status_t      status;
    int              i=0;
    uint8_t          tmp[max_rw_size];

    dev = (struct hbi_dev *)(PDE_DATA(filp->f_inode));

    if(dev==NULL)
    {
        HBI_LNX_DBG("Invalid Device\n");
        return -1;
    }

    /*expected format dev_addr:reg:val */
    if(copy_from_user((void *)val,buf,size))
    {
        HBI_LNX_DBG("Couldn't copy whole user buffer\n");
    }

    while(val[i++] != ' ');

    if(i >= size)
    {
        printk("To write register enter: echo <register addr in hex> <data in hex(should be multiple of 2)>\n");
        return -1;
    }

    reg = atoh(val,i-1);
    data_len=0;

    printk("Writing ... \n");

    while((val[i] != ' ') && (i <(size-1)) && (data_len < sizeof(tmp)))
    {
        printk("input %c %c\n",val[i],val[i+1]);
        tmp[data_len] = atoh(&val[i],2); /* convert two chars into a byte */
        HBI_LNX_DBG("0x%x\n ",tmp[data_len]);
        data_len++;
        i+=2;
    }

    HBI_LNX_DBG("Got addr 0x%x,len %d reg 0x%x\n",dev->devcfg.dev_addr,data_len,reg);

    status = HBI_write((dev->hbi_handle),reg,tmp,data_len);

    if(status != HBI_STATUS_SUCCESS)
    {
        printk("Write Failed\n");
        return -1;
    }

    return size;
}

ssize_t hbi_proc_open_dev_wr(struct file *filp, const char __user *buf, size_t size, loff_t *offset)
{
    hbi_dev_cfg_t     devcfg;
    uint8_t         val[16];
    uint8_t         bus_num[2]={'\0'};
    uint8_t         *tmp;
    struct hbi_dev *device;
    hbi_status_t      status;
    int             i;

    HBI_LNX_DBG("Enter \n");

    memset(val,0,sizeof(val));
    
    if(copy_from_user((void *)val,buf,size))
    {
        HBI_LNX_DBG("Couldn't copy whole user buffer\n");
    }

    tmp = strchr(val,':');
    if(tmp == NULL)
    {
        printk("Enter device addr as bus_num:dev_addr \n");
        return -1;
    }
    for(i=0;i<(tmp-val);i++)
    {
        bus_num[i]=val[i];
    }

    devcfg.bus_num = atoh(bus_num,(tmp-val));
    devcfg.dev_addr = atoh(tmp,size-(tmp-val));
    devcfg.pDevName = NULL;
    
    HBI_LNX_DBG("received dev_addr 0x%x bus number %d \n",
                  (unsigned int)(devcfg.dev_addr),devcfg.bus_num);

    status = internal_hbi_open(&device,&devcfg);

    if(status == HBI_STATUS_SUCCESS)
    {
        filp->private_data = (void *)device;
        HBI_LNX_DBG("Opened device handle 0x%x\n",(unsigned int)device);
    }
    else
        HBI_LNX_DBG("Failed to open device\n");

    return size;
}

ssize_t hbi_proc_open_dev_rd(struct file *filp,  char __user *buf, size_t size, loff_t *offset)
{

    struct hbi_dev *dev;

    HBI_LNX_DBG("Enter \n");

    printk("Opened devices(addr:bus_num)\n");

    if(hbi_lnx_drv_priv.dev != NULL)
    {
        list_for_each_entry(dev,&(hbi_lnx_drv_priv.dev->list),list)
        {
            printk("0x%x:%d\n",dev->devcfg.dev_addr,dev->devcfg.bus_num);
        }
        printk("0x%x:%d\n",dev->devcfg.dev_addr,dev->devcfg.bus_num);
    }

    return 0;
}


ssize_t hbi_proc_close_dev_wr(struct file *filp, 
                              const char __user *buf, 
                              size_t size, loff_t *offset)
{
    uint8_t           val[16];
    uint32_t          dev_addr;
    hbi_status_t      status;
    struct hbi_dev    *device=NULL;

    HBI_LNX_DBG("Enter\n");

    memset(val,0,sizeof(val));

    if(copy_from_user((void *)val,buf,size))
    {
        HBI_LNX_DBG("Couldn't copy whole user buffer\n");
    }

    dev_addr = atoh(val,size);

    HBI_LNX_DBG("Received address 0x%x to close\n",dev_addr);
    if(hbi_lnx_drv_priv.dev == NULL)
    {
        HBI_LNX_DBG("No more devices to close\n");
        return size;
    }

    /* Search if device with requested address is registered */
    list_for_each_entry(device,&(hbi_lnx_drv_priv.dev->list),list)
    {
        if(device && (device->devcfg.dev_addr == dev_addr))
        {
            HBI_LNX_DBG("Close dev_addr 0x%x device handle 0x%x\n",
                        (unsigned int)dev_addr,
                        (unsigned int)device);
            status = internal_hbi_close(device);
            device = NULL;
            return size;
        }
    }

    /* do this for head entry */
    HBI_LNX_DBG("received address 0x%x registered device handle address 0x%x\n",
               (unsigned int)dev_addr,(unsigned int)(device));

    if(device && (device->devcfg.dev_addr == dev_addr))
    {
        status = internal_hbi_close(device);
        HBI_LNX_DBG("internal_hbi_close status %s\n",
                     (status == HBI_STATUS_SUCCESS) ? "OK!" : "Err!");
    }

    return size;
}

ssize_t hbi_proc_wr_cfgrec(struct file *filp, 
                           const char __user *buf, 
                           size_t size, loff_t *offset)
{
#define APP_RST_WAIT 10000

   char                *kbuf;
   struct hbi_dev      *dev = PDE_DATA(filp->f_inode);
   int                 i=0;
   reg_addr_t          reg;
   uint32_t            val;
   user_buffer_t       tmp[2];
   char                c;
   hbi_status_t          status;
   int                  count=0;

   HBI_LNX_DBG("Enter... size of user buffer %d\n",size);

   if(!size || dev == NULL)
   {
     HBI_LNX_DBG("Either size is NULL or no device opened\n");
     return -1;
   }

   kbuf = kmalloc(size,GFP_KERNEL);
   if(!kbuf)
   {
     HBI_LNX_DBG("Memory Allocation Failure\n");
     return -1;
   }

   if(copy_from_user(kbuf,buf,size))
   {
     HBI_LNX_DBG("Couldn't copy whole user buffer\n");
   }

   i=0;
   /* Reset size, it is assumed every new cfgrec loading will invalidate previous settings */
   dev->cfgrec.size=0;

   while(i<size)
   {
     if(kbuf[i] != ';') 
     {
         if (sscanf(&kbuf[i], "%x %c %x", (unsigned int *)&reg, &c, (unsigned int *)&val) == 3)
         {
             /*
                 TODO: this is probably slower approach to write every 2 bytes. 
                 We can change implementation to accumulate the data upto allowed limit and
                 then write. This implementation to be revisit during performance evaluation.
             */
             /* Device bydefault operate in Big Endian so swap MSB first before writing a word */

             tmp[0]=(val>>8)& 0xFF;
             tmp[1]=val&0xFF;
             status = HBI_write((dev->hbi_handle),reg,(user_buffer_t *)tmp,2);
             if(status == HBI_STATUS_SUCCESS)
             {
                 dev->cfgrec.size+=2;
             }
             else
             {
                 HBI_LNX_DBG("HBI write failed\n");
             }
         }
     }

     /* skip to next line */
     while(kbuf[i++] != '\n');
   }

   HBI_LNX_DBG("Received CFGREC of size %d\n",dev->cfgrec.size);

   reg = ZL380xx_HOST_SW_FLAGS_REG;

   tmp[0] = ZL380xx_HOST_SW_FLAGS_APP_REBOOT>>8;
   tmp[1]=ZL380xx_HOST_SW_FLAGS_APP_REBOOT&0xFF;
   status = HBI_write((dev->hbi_handle),reg,tmp,2);

   tmp[0]=0;
   tmp[1]=0;
   
   if(status == HBI_STATUS_SUCCESS)
   {
      do{
      /* wait for reset to complete */
      status = HBI_read(dev->hbi_handle, reg,(user_buffer_t*) tmp, 2);
      }while((status == HBI_STATUS_SUCCESS) && (count++ < APP_RST_WAIT));
   }
   /* value of reg comes out to be 0xFFF8 on successful reset */
   HBI_LNX_DBG("reset result 0x%x\n",*((unsigned int *)tmp));

   kfree(kbuf);
   return size;
}

ssize_t hbi_proc_load_fw(struct file *filp, 
                        const char __user *buf, size_t size, loff_t *offset)
{
   struct hbi_dev    *dev = (struct hbi_dev *)(PDE_DATA(filp->f_inode));
   hbi_status_t status;
   static int first=1;
   static int total_img_len=1;
   static int block_size;
   static int hdr_len;
   hbi_data_t img;
   hbi_img_hdr_t hdr;
   HBI_LNX_DBG("Enter (size %d)... \n",size);
   
   if(dev == NULL)
   {
     HBI_LNX_DBG("NULL device or image size retrieved\n");
     return -1;
   }

   if(!size)
   {
     HBI_LNX_DBG("Either size is NULL or no device opened\n");
     return -1;
   }

   if((dev->hbi_buf.len + size) > sizeof(dev->hbi_buf.buf))
   {
     /* reset firmware buffer */
     memset(&(dev->hbi_buf),0,sizeof(dev->hbi_buf));
     HBI_LNX_DBG("Could not load complete data size of" \
                  " Firmware Buffer is less than size of data\n");
     return -1;
   }
   
   /* this function ideally should be loading image on to device. 
    however sometimes file write operation using cat command/linux shell
    gives data in chunk rather than one buffer with whole image. 
    in such scenario we were getting checksum error after downloading to 
    device was complete. We probably were missing out on information.
    So on safer note, this command accumulates complete data in kernel  
    buffer and then written to device
   */
   if(copy_from_user(&(dev->hbi_buf.buf[dev->hbi_buf.len]),buf,size))
   {
     HBI_LNX_DBG("Couldn't copy whole user buffer\n");
     return -1;
   }

   dev->hbi_buf.len += size;
   if(first)
   {
      /* if this the first chunk received, parse header and 
         get total image len */
      img.pData = dev->hbi_buf.buf;
      img.size = dev->hbi_buf.len;
      status = HBI_get_header(&img,&hdr);
      if(status != HBI_STATUS_SUCCESS)
      {
         printk("Error ! Invalid Image Header Found\n");
         return -1;
      }
      total_img_len = hdr.img_len;
      block_size = hdr.block_size;
      hdr_len = hdr.hdr_len;
      first = 0;
   }
   if(dev->hbi_buf.len < total_img_len)
   {
      HBI_LNX_DBG("size of buffer %d\n",dev->hbi_buf.len);
      return size;
   }

   HBI_LNX_DBG("Passing fwr image of size 0x%x\n",total_img_len);

   /* skip rest of the header jump to payload */
   status = hbi_wr_bin_fw(dev, 
                           &(dev->hbi_buf.buf[hdr_len]),
                           total_img_len,
                           block_size);

   memset(&(dev->hbi_buf),0,sizeof(dev->hbi_buf));
   dev->hbi_buf.len =0;

   if(status != HBI_STATUS_SUCCESS)
   {
      HBI_LNX_DBG("failed to load firmware to device\n");
      return -1;
   }
   return size;
}

ssize_t hbi_proc_start_fw(struct file *filp,  char __user *buf, 
                           size_t size, loff_t *offset)
{
   struct hbi_dev   *pDev=(struct hbi_dev *)(PDE_DATA(filp->f_inode));
   hbi_status_t      status;

   HBI_LNX_DBG("Enter... \n");

   if(pDev == NULL)
   {
     HBI_LNX_DBG("NULL device or image size retrieved\n");
     return -1;
   }

   status = HBI_set_command((pDev->hbi_handle),HBI_CMD_START_FWR,NULL);
   if(status != HBI_STATUS_SUCCESS)
   {
     HBI_LNX_DBG("failed to load firmware to device\n");
     return -1;
   }

   return 0;
}

ssize_t hbi_proc_rd_cfgrec(struct file *filp,  
                           char __user *buf, 
                           size_t size, loff_t *offset)
{
   hbi_status_t    status;
   struct hbi_dev *dev = hbi_lnx_drv_priv.dev;
   int            i=0;
   uint16_t       tmp;

   if(hbi_lnx_drv_priv.dev == NULL)
   {
      HBI_LNX_DBG("No device opened to read cfgrec\n");
      return -1;
   }

   printk("; Addr, Data,\n");
   printk("; ----- -----\n");
   for(i=HBI_CFGREC_BASE;i<HBI_CFGREC_MAX_SIZE;i+=2)
   {
      status=HBI_read(dev->hbi_handle,i,(user_buffer_t *)&tmp,2);
      printk("0x%04X, 0x%04X\n",i,tmp);
   }

   return 0;
}

#ifdef FLASH_PRESENT
ssize_t hbi_proc_save_fwrcfgrec_to_flash(struct file *filp,  char __user *buf, size_t size, loff_t *offset)
{
   struct hbi_dev *dev = (struct hbi_dev*)(PDE_DATA(filp->f_inode));
   hbi_status_t   status;
   int            image_num=-1;

   if(dev==NULL)
   {
      HBI_LNX_DBG("NULL device handle\n");
      return -1;
   }

   status = HBI_set_command((dev->hbi_handle),
                           HBI_CMD_SAVE_FWRCFG_TO_FLASH,
                           &image_num);
   if(status == HBI_STATUS_SUCCESS)
   {
      HBI_LNX_DBG("Assigned image number %d\n",image_num);
   }
   return 0;
}


ssize_t hbi_proc_load_fwrcfgrec_from_flash(struct file *filp, 
                                          const char __user *buf, 
                                          size_t size, loff_t *offset)
{
    struct hbi_dev *dev = (struct hbi_dev *)(PDE_DATA(filp->f_inode));
    hbi_status_t  status;
    char          image_num[2];
    int32_t       tmp;
    
    if(dev == NULL)
    {
        HBI_LNX_DBG("NULL device handle\n");
        return -1;
    }
    
    if(copy_from_user(&tmp,buf,size))
    {
        HBI_LNX_DBG("Couldn't copy whole data\n");
        return -1;
    }
   memset(image_num,0,sizeof(image_num));
    tmp = atoi((char *)&tmp,size);

    image_num[0]=tmp>>8;
    image_num[1]=tmp & 0xff;
    HBI_LNX_DBG("Writing image num %d tmp %d\n",*((unsigned int *)image_num),tmp);
    status = HBI_set_command((dev->hbi_handle),
                              HBI_CMD_LOAD_FWRCFG_FROM_FLASH,
                              (void *)image_num);

    return size;
}


ssize_t hbi_proc_erase_img_from_flash(struct file *filp, 
                                    const char __user *buf, 
                                    size_t size, loff_t *offset)
{
    struct hbi_dev *dev = (struct hbi_dev *)(PDE_DATA(filp->f_inode));

    hbi_status_t  status;
    char          image_num[2];
    int16_t       tmp;
    
    if(dev == NULL)
    {
        HBI_LNX_DBG("NULL device handle\n");
        return -1;
    }
    
    if(copy_from_user(&tmp,buf,size))
    {
      HBI_LNX_DBG("copy_from_user failed. couldn't copy whole user arg\n");
      return -1;
    }
    
    memset(image_num,0,sizeof(image_num));

    tmp = atoi((char *)&tmp,size);

    HBI_LNX_DBG("Erase image number %d\n",tmp);


    image_num[0]=tmp>>8;
    image_num[1]=tmp & 0xFF;

    status = HBI_set_command((dev->hbi_handle),
                              HBI_CMD_ERASE_FWRCFG_FROM_FLASH,
                              (void *)image_num);

    return size;
}

ssize_t hbi_proc_erase_flash(struct file *filp,  
                              char __user *buf, size_t size, loff_t *offset)
{
    struct hbi_dev *dev = (struct hbi_dev*)(PDE_DATA(filp->f_inode));
    hbi_status_t status;

    if(dev==NULL)
    {
        HBI_LNX_DBG("NULL device handle\n");
        return -1;
    }

    status = HBI_set_command((dev->hbi_handle),HBI_CMD_ERASE_WHOLE_FLASH,NULL);
    return 0;
}
#endif

ssize_t hbi_proc_drv_init(struct file *filp,  char __user *buf, size_t size, loff_t *offset)
{
    hbi_status_t status = HBI_init(NULL);
    
    if(status != HBI_STATUS_SUCCESS)
    {
      HBI_LNX_DBG(" HBI_init failed.Err 0x%x\n",status);
    }
    return 0;
}

ssize_t hbi_proc_drv_term(struct file *filp,  char __user *buf, size_t size, loff_t *offset)
{
    hbi_status_t status = HBI_term();
    
    if(status != HBI_STATUS_SUCCESS)
    {
      HBI_LNX_DBG(" HBI_term failed.Err 0x%x\n",status);
    }
    return 0;
}

static int procfs_init(void)
{
    int i;

    hbi_lnx_drv_priv.drv_proc_dir = proc_mkdir("hbi",NULL);

    if(hbi_lnx_drv_priv.drv_proc_dir == NULL )
    {
        HBI_LNX_DBG("Couldn't create HBI Driver Proc Entry\n");
        return -1;
    }

    for(i=HBI_PROC_ENTRY_FIRST;i<HBI_PROC_ENTRY_LAST;i++)
    {
        HBI_LNX_DBG("Creating proc_entry name %s\n",drv_proc_entry[i].name);
        drv_proc_entry[i].proc_entry = proc_create(drv_proc_entry[i].name,
                                                   (S_IFREG|S_IRUGO|S_IWUGO),
                                                   hbi_lnx_drv_priv.drv_proc_dir,
                                                   &(drv_proc_entry[i].ops));
        if(drv_proc_entry[i].proc_entry == NULL)
        {
            HBI_LNX_DBG("Couldn't create HBI Driver %s Proc Entry\n",
                        drv_proc_entry[i].name);
        }
    }
    return 0;
}

static int procfs_term(void)
{
    int i;

    for(i=HBI_PROC_ENTRY_FIRST;i<HBI_PROC_ENTRY_LAST;i++)
    {
        if(drv_proc_entry[i].proc_entry != NULL)
        {
            proc_remove(drv_proc_entry[i].proc_entry);
            drv_proc_entry[i].proc_entry = NULL;
        }
    }
    if(hbi_lnx_drv_priv.drv_proc_dir != NULL)
    {
        /* remove base dir */
        proc_remove(hbi_lnx_drv_priv.drv_proc_dir);
        hbi_lnx_drv_priv.drv_proc_dir = NULL;
    }
    return 0;
}
#endif /* HBI_ENABLE_PROCFS */

static int __init hbi_drv_init(void)
{
    int         ret;

    HBI_LNX_DBG("Enter ... \n");
    memset(&hbi_lnx_drv_priv,0,sizeof(struct hbi_lnx_drv));

    /* register HBI driver as character device driver  */
    ret = alloc_chrdev_region(&hbi_lnx_drv_priv.dev_t,
                              HBI_DRV_MINOR_NUM,HBI_DRV_DEV_CNT,HBI_DEV_NAME);
    if(ret <0)
        return ret;

    hbi_lnx_drv_priv.cdev=cdev_alloc();
    if(hbi_lnx_drv_priv.cdev == NULL)
    {
        unregister_chrdev_region(hbi_lnx_drv_priv.dev_t,HBI_DRV_DEV_CNT);
        return -1;
    }
    
    cdev_init(hbi_lnx_drv_priv.cdev,&fops);

    ret = cdev_add(hbi_lnx_drv_priv.cdev,hbi_lnx_drv_priv.dev_t,HBI_DRV_DEV_CNT);
    if(ret<0)
    {
        unregister_chrdev_region(hbi_lnx_drv_priv.dev_t,HBI_DRV_DEV_CNT);
        cdev_del(hbi_lnx_drv_priv.cdev);
        return ret;
    }
    hbi_lnx_drv_priv.dev_class = class_create(THIS_MODULE,HBI_DEV_NAME);
    if(hbi_lnx_drv_priv.dev_class)
    {
    /* TODO: There's a complexity realized if creating device name with Major 
      and Minor number in it. If we add this, user space app will need to know
      Major and Minor number information which requires user should extract this
      information either through proc or sysfs or our driver export an API to pass
      back this information so that user can open appropriate driver node.
      Currently we foresee having single node is enough as 1 single driver 
      instance handle multiple devices.
      */
        hbi_lnx_drv_priv.device = device_create(hbi_lnx_drv_priv.dev_class,
                                                NULL,
                                                hbi_lnx_drv_priv.dev_t,NULL,
                                                "%s",HBI_DEV_NAME);
        if(hbi_lnx_drv_priv.device == NULL)
        {
            printk("Unable to create a device type\n");
        }
    }
    else
    {
        printk("Unable to create a device class\n");
        unregister_chrdev_region(hbi_lnx_drv_priv.dev_t,HBI_DRV_DEV_CNT);
        cdev_del(hbi_lnx_drv_priv.cdev);
    }

#if (HBI_ENABLE_PROCFS)
    HBI_LNX_DBG("Init PROC FS\n");
    procfs_init();
#endif

    HBI_LNX_DBG("Exit ... \n");
    return 0;
}

static void __exit hbi_drv_exit(void)
{
    struct hbi_dev *dev;
    HBI_LNX_DBG("Check if any device is opened\n");
    /* TODO: driver by itself closing any opened instances 
         before final termination without notifying applications 
         thats instances are getting closed. 
         This may not be desired behavior. However until we receive
         any specific request to turn this off, we are keeping it this
         way and driver unload free up all occupied resources.
    */
    if(hbi_lnx_drv_priv.dev != NULL)
    {
        list_for_each_entry(dev,&(hbi_lnx_drv_priv.dev->list),list)
        {
            HBI_LNX_DBG("Closing Device Addr 0x%x:%d\n",
                           dev->devcfg.dev_addr,
                           dev->devcfg.bus_num);
            internal_hbi_close(dev);
        }
        internal_hbi_close(hbi_lnx_drv_priv.dev);
    }
    
    
    if(HBI_term() != HBI_STATUS_SUCCESS)
    {
        printk("HBI_term failed\n");
        return;
    }
#if (HBI_ENABLE_PROCFS)
    procfs_term();
#endif
    cdev_del(hbi_lnx_drv_priv.cdev);
    device_destroy(hbi_lnx_drv_priv.dev_class,hbi_lnx_drv_priv.dev_t);
    class_destroy(hbi_lnx_drv_priv.dev_class);
    unregister_chrdev_region(hbi_lnx_drv_priv.dev_t,HBI_DRV_DEV_CNT);

    return ;
}

EXPORT_SYMBOL(HBI_init);
EXPORT_SYMBOL(HBI_term);
EXPORT_SYMBOL(HBI_open);
EXPORT_SYMBOL(HBI_close);
EXPORT_SYMBOL(HBI_read);
EXPORT_SYMBOL(HBI_write);
EXPORT_SYMBOL(HBI_reset);
EXPORT_SYMBOL(HBI_set_command);

module_init(hbi_drv_init);
module_exit(hbi_drv_exit);

MODULE_AUTHOR("Shally Verma <shally.verna@microsemi.com>");
MODULE_DESCRIPTION(" Microsemi Timberwolf Voice Processor Driver");
MODULE_LICENSE("GPL");


