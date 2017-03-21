/*
* hal_spi.c - spi driver implmentation for HBI
*
* Hardware Abstraction Layer for Voice processor devices
* over linux kernel 3.18.x spi core driver. Every successful call would return 0 or 
* a linux error code as defined in linux errno.h
*
* Copyright 2016 Microsemi Inc.
*
*/

#include <linux/spi/spi.h>
#include <linux/list.h>
#include "typedefs.h"
#include "ssl.h"
#include "hal.h"
#include "vproc_dbg.h"

#define HAL_DEBUG 0



int vproc_probe(struct spi_device *);
int vproc_remove(struct spi_device *);

/* array containing device names supported by this driver */
static struct spi_device_id vproc_device_id[VPROC_MAX_NUM_DEVS];

/* Structure definining spi driver */
struct spi_driver vproc_driver = {
    .id_table = vproc_device_id,
    .probe = vproc_probe,
    .remove = vproc_remove,
    .driver = {
        .name = VPROC_DEV_NAME, /* name field should be equal  
                                       to module name and without spaces */
        .owner = THIS_MODULE,
    }
};

/*
the probe function is called when an entry in the id_table name field
matches the device's name. It is passed the entry that was matched so
the driver knows which one in the table matched.
Note:
The necessary client fields have already been setup before
the probe function is called.
*/
int vproc_probe(struct spi_device *pClient)
{

    VPROC_DBG_PRINT(VPROC_DBG_LVL_FUNC,"Enter...\n");
    
    /* Bind device to driver */
    pClient->dev.driver = &(vproc_driver.driver);
    
    VPROC_DBG_PRINT(VPROC_DBG_LVL_FUNC,"Exit...\n");
    return 0;
}

int vproc_remove(struct spi_device *pClient)
{
    VPROC_DBG_PRINT(VPROC_DBG_LVL_FUNC,"Enter...\n");
    return 0;
}

int hal_init(void)
{

    VPROC_DBG_PRINT(VPROC_DBG_LVL_FUNC,"Enter...\n");

    return spi_register_driver(&vproc_driver);
}

int hal_term()
{
    VPROC_DBG_PRINT(VPROC_DBG_LVL_FUNC,"Enter\n");

    spi_unregister_driver(&vproc_driver);

    VPROC_DBG_PRINT(VPROC_DBG_LVL_FUNC,"Exit\n");

    return 0;
}
/*
    This function opens an spi client to a device 
    identifies via "devcfg" parameter.
    Returns 0 and update client into a reference handle, if successful
    or an error code for an invalid parameter or client instantiation 
    fails.
*/
int hal_open(void **ppHandle,void *pDevCfg)
{
    ssl_dev_cfg_t *pDev = (ssl_dev_cfg_t *)pDevCfg;
    struct spi_master *adap = NULL;
    struct spi_board_info bi;
    struct spi_device *pClient = NULL;

    VPROC_DBG_PRINT(VPROC_DBG_LVL_FUNC,"Enter\n");
    if(pDev == NULL)
    {
        VPROC_DBG_PRINT(VPROC_DBG_LVL_ERR,"Invalid Device Cfg Reference\n");
        return -EINVAL;
    }
    /* get the controller driver through bus num */
    adap = spi_busnum_to_master(pDev->bus_num);
    if(adap==NULL)
    {
        VPROC_DBG_PRINT(VPROC_DBG_LVL_ERR,"Invalid Bus Num %d \n",pDev->bus_num);
        return SSL_STATUS_INVALID_ARG;
    }

    VPROC_DBG_PRINT(VPROC_DBG_LVL_INFO,"spi adap name %s \n",adap->dev.init_name);

    memset(&bi,0,sizeof(bi));
    if(pDev->pDevName != NULL)
    {
        strcpy(bi.modalias,pDev->pDevName);
    }
    
    bi.bus_num = pDev->bus_num;
    bi.chip_select = pDev->dev_addr;
    bi.max_speed_hz = 25000000;
    bi.mode = SPI_MODE_0;
    
    pClient = spi_new_device(adap,&bi);
    if(pClient == NULL)
    {
        /* call failed either because address is invalid, valid but occupied
           or there is resource err. Just return code to try again later */
        VPROC_DBG_PRINT(VPROC_DBG_LVL_ERR,"spi device instantiation failed\n");
        return -EAGAIN;
    }

    VPROC_DBG_PRINT(VPROC_DBG_LVL_INFO,
                     "Updating handle 0x%x back to user\n",(uint32_t)pClient);

   *((struct spi_device **)ppHandle) =  pClient;

    VPROC_DBG_PRINT(VPROC_DBG_LVL_FUNC,"Exit..\n");

    return 0;
}

int hal_close(void *pHandle)
{
    VPROC_DBG_PRINT(VPROC_DBG_LVL_FUNC,"Enter..\n");

    if(pHandle == NULL)
    {
        VPROC_DBG_PRINT(VPROC_DBG_LVL_ERR,"NULL client handle passed\n");
        return -EINVAL;
    }

    VPROC_DBG_PRINT(VPROC_DBG_LVL_INFO,
                     "Unregistering client 0x%x\n",(u32) pHandle);
    
    spi_unregister_device((struct spi_device *) pHandle);
    
    VPROC_DBG_PRINT(VPROC_DBG_LVL_FUNC,"Exit..\n");
    return 0;
}

int hal_port_rw(void *pHandle,void *pPortAccess)
{
    int                 ret=0;
    ssl_port_access_t   *pPort = (ssl_port_access_t *)pPortAccess;
    struct spi_device  *pClient = (struct spi_device *)pHandle;
    int                 msgnum=0;
    struct spi_transfer xfers[2];
    ssl_op_t          op_type;
#if HAL_DEBUG
    int                  i;
#endif
    VPROC_DBG_PRINT(VPROC_DBG_LVL_FUNC,
                     "Enter (handle 0x%x)...\n",(uint32_t)pHandle);
    if(pHandle == NULL || pPort == NULL)
    {
        VPROC_DBG_PRINT(VPROC_DBG_LVL_ERR,"Invalid Parameters\n");
        return -EINVAL;
    }
    
    op_type = pPort->op_type;
    memset(xfers,0,sizeof(xfers));
    if(op_type & SSL_OP_PORT_WR)
    {
        if(pPort->pSrc == NULL)
        {
            VPROC_DBG_PRINT(VPROC_DBG_LVL_ERR,"NULL src buffer passed\n");
            return -EINVAL;
        }
        
        xfers[0].tx_buf = pPort->pSrc;
        xfers[0].len = pPort->nwrite;
//        xfers[0].speed_hz = 25000000;
        
#if HAL_DEBUG
        VPROC_DBG_PRINT(VPROC_DBG_LVL_INFO,"writing %d bytes..\n",pPort->nwrite);

        for(i=0;i<pPort->nwrite;i++)
        {
            printk("0x%x\t",((uint8_t *)(pPort->pSrc))[i]);
        }
        printk("\n");
#endif
        msgnum++;
    }

    if(op_type & SSL_OP_PORT_RD)
    {
        if(pPort->pDst == NULL)
        {
            VPROC_DBG_PRINT(VPROC_DBG_LVL_ERR,"NULL destination buffer passed\n");
            return -EINVAL;
        }
        xfers[msgnum].rx_buf = pPort->pDst;
        xfers[msgnum].len = pPort->nread;
//        xfers[msgnum].speed_hz = 25000000;

        msgnum++;

        VPROC_DBG_PRINT(VPROC_DBG_LVL_INFO,"read %d bytes..\n",pPort->nread);
    }
    ret  = spi_sync_transfer(pClient,xfers,msgnum);
    if(ret < 0)
    {
        VPROC_DBG_PRINT(VPROC_DBG_LVL_ERR,"failed with Error %d\n",ret);
    }
    #if HAL_DEBUG
    if(msgnum >=1)
    {
        VPROC_DBG_PRINT(VPROC_DBG_LVL_INFO,"Received...\n");
        for(i=0;i<pPort->nread;i++)
        {
            printk("0x%x\t",((uint8_t *)(pPort->pDst))[i]);
        }
        printk("\n");
    }
    #endif
    return ret;
}




