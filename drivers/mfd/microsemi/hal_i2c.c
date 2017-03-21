/*
* hal_i2c.c - i2c driver implmentation for HBI
*
* Hardware Abstraction Layer for Voice processor devices
* over linux kernel 3.18.x i2c core driver. Every successful call would return 0 or 
* a linux error code as defined in linux errno.h
*
* Copyright 2016 Microsemi Inc.
*
*/

#include <linux/i2c.h>
#include "typedefs.h"
#include "ssl.h"
#include "hal.h"
#include "vproc_dbg.h"

#define HAL_DEBUG 0



int vproc_i2c_probe(struct i2c_client *, const struct i2c_device_id *);
int vproc_i2c_remove(struct i2c_client *);
int vproc_i2c_detect(struct i2c_client *, struct i2c_board_info *);

/* array containing device names supported by this driver */
static struct i2c_device_id vproc_i2c_device_id[VPROC_MAX_NUM_DEVS];

/* array containing device addresses supported by this driver */
static unsigned short vproc_i2c_addr[VPROC_MAX_NUM_DEVS+1];

/* Structure definining i2c driver */
struct i2c_driver vproc_i2c_driver = {
    .driver = {
        .name = VPROC_DEV_NAME,
        .owner = THIS_MODULE,
    },
    .probe = vproc_i2c_probe,
    .remove = vproc_i2c_remove,
    .id_table = vproc_i2c_device_id,
    .address_list = vproc_i2c_addr
};


int hal_init(void)
{
    return i2c_add_driver(&vproc_i2c_driver);
}

int hal_term()
{
   i2c_del_driver(&vproc_i2c_driver);
   return 0;
}
/*
    This function opens an i2c client to a device 
    identifies via "devcfg" parameter.
    Returns 0 and update client into a reference handle, if successful
    or an error code for an invalid parameter or client instantiation 
    fails.
*/
int hal_open(void **ppHandle,void *pDevCfg)
{
   struct i2c_client      *pClient=NULL;
   struct i2c_board_info   bi;
   struct i2c_adapter      *pAdap=NULL;

   ssl_dev_cfg_t *pDev = (ssl_dev_cfg_t *)pDevCfg;

   if(pDev == NULL)
   {
      VPROC_DBG_PRINT(VPROC_DBG_LVL_ERR,"Invalid Device Cfg Reference\n");
      return -EINVAL;
   }

   pAdap = i2c_get_adapter(pDev->bus_num);
   if(pAdap==NULL)
   {
     VPROC_DBG_PRINT(VPROC_DBG_LVL_ERR,"Invalid Bus Num \n");
     return SSL_STATUS_INVALID_ARG;
   }

   VPROC_DBG_PRINT(VPROC_DBG_LVL_INFO,"i2c adap name %s \n",pAdap->name);

   memset(&bi,0,sizeof(bi));
    if(pDev->pDevName != NULL)
   {
     strcpy(bi.type,pDev->pDevName);
   }
   bi.addr = pDev->dev_addr;

   pClient = i2c_new_device(pAdap,(struct i2c_board_info const *)&bi);
   if(pClient == NULL)
   {
     /* call failed either because address is invalid, valid but occupied
        or there is resource err. Just return code to try again later */
     VPROC_DBG_PRINT(VPROC_DBG_LVL_ERR,"i2c device instantiation failed\n");
     return -EAGAIN;
   }

   *((struct i2c_client **)ppHandle) =  pClient;

   return 0;
}

int hal_close(void *pHandle)
{
    if(pHandle == NULL)
    {
        VPROC_DBG_PRINT(VPROC_DBG_LVL_ERR,"NULL client handle passed\n");
        return -EINVAL;
    }
    i2c_unregister_device((struct i2c_client *) pHandle);
    return 0;
}

int hal_port_rw(void *pHandle,void *pPortAccess)
{
   int ret=0;
   ssl_port_access_t *port = (ssl_port_access_t *)pPortAccess;
   struct i2c_client *pClient = pHandle;
   struct i2c_msg msg[2];
   int msgnum=0;
   ssl_op_t op_type;
   int i;

   if(pHandle == NULL || pPortAccess == NULL)
   {
      VPROC_DBG_PRINT(VPROC_DBG_LVL_ERR,"Invalid Parameters\n");
      return -EINVAL;
   }

   op_type = port->op_type;

   if(op_type & SSL_OP_PORT_WR)
   {
      if(port->pSrc == NULL)
      {
         VPROC_DBG_PRINT(VPROC_DBG_LVL_ERR,"NULL src buffer passed\n");
         return -EINVAL;
      }
      msg[0].addr = pClient->addr;
      msg[0].flags = 0;
      msg[0].buf = port->pSrc;
      msg[0].len = port->nwrite;

      VPROC_DBG_PRINT(VPROC_DBG_LVL_INFO,"writing %d bytes..\n",msg[0].len);
      for(i=0;i<port->nwrite;i++)
      {
         printk("0x%x\t",((uint8_t *)(port->pSrc))[i]);
      }
      printk("\n");

      msgnum++;
   }
   if(op_type & SSL_OP_PORT_RD)
   {
      if(port->pDst == NULL)
      {
         VPROC_DBG_PRINT(VPROC_DBG_LVL_ERR,"NULL destination buffer passed\n");
         return -EINVAL;
      }
      msg[msgnum].addr = pClient->addr;
      msg[msgnum].flags = I2C_M_RD;
      msg[msgnum].buf = port->pDst;
      msg[msgnum].len = port->nread;
      msgnum++;

      VPROC_DBG_PRINT(VPROC_DBG_LVL_INFO,"read %d bytes..\n",msg[1].len);
   }

   ret = i2c_transfer(pClient->adapter,msg,msgnum);
   if(ret < 0)
   {
      VPROC_DBG_PRINT(VPROC_DBG_LVL_ERR,"failed with Error 0x%x\n",ret);
   }
#if HAL_DEBUG
   if(!ret && (msgnum >=1))
   {
      printk("Received...\n");
      for(i=0;i<port->nread;i++)
      {
         printk("0x%x\t",((uint8_t *)(port->pDst))[i]);
      }
      printk("\n");
   }
#endif
   return ret;
}

/*
the probe function is called when an entry in the id_table name field
matches the device's name. It is passed the entry that was matched so
the driver knows which one in the table matched.
Note:
The necessary client fields have already been setup before
the probe function is called.
The call to i2c_attach_client() is no longer needed, if the probe
routine exits successfully, then the driver will be automatically
attached by the core. 

if you wish to add any driver specific information to client
then can call i2c_set_clientdata() here.

*/
int vproc_i2c_probe(struct i2c_client *pClient, const struct i2c_device_id *pDeviceId)
{
    /* Bind device to driver */
    pClient->dev.driver = &(vproc_i2c_driver.driver);
    return 0;
}

int vproc_i2c_remove(struct i2c_client *pClient)
{
    return 0;
}



