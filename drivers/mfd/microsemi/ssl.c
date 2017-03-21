/*
* ssl.c - System Service Layer implementation for Raspberry Pi
*
*
* Copyright 2016 Microsemi Inc.
*
* This program is free software you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation; either version 2 of the License, or
* (at your option)any later version.
*/

#include <linux/mutex.h>
#include <linux/delay.h>
#include "typedefs.h"
#include "ssl.h"
#include "hal.h"
#include "vproc_dbg.h"

#define SSL_LOCK_NAME_SIZE 32

/* Variable for current debug level set in the system.*/
VPROC_DBG_LVL vproc_dbg_lvl = DEBUG_LEVEL;

/* structure defining lock created by ssl */
struct ssl_lock
{
    uint8_t     name[SSL_LOCK_NAME_SIZE]; /* name of lock */
    struct mutex lock; /* lock of the type mutex */
    bool   inuse;       /* flag indicating current entry is in use */
};

/* strcture defining devices opened by ssl */
struct ssl_dev
{
    void *pClient; /* pointer to device instance */
    bool inuse;   /* flag indicating entry in use */
};

/* structure defining driver */
struct ssl_drv{
    struct ssl_lock     lock[NUM_MAX_LOCKS]; /* locks managed by driver */
    struct ssl_dev      dev[VPROC_MAX_NUM_DEVS]; /* number of devices 
                                                   managed by driver */
    bool                initialised;  /* flag indicating driver is initialised */
};

/* variable keeping driver level information */
static struct ssl_drv ssl_drv_priv; 

/* Macro to check for port handle validity */
#define CHK_PORT_HANDLE_VALIDITY(handle) \
{ \
    for(i=0;i<VPROC_MAX_NUM_DEVS;i++) \
    { \
        if((&(ssl_drv_priv.dev[i]) == handle) && (ssl_drv_priv.dev[i].inuse)) \
            break; \
    } \
    if( i >= VPROC_MAX_NUM_DEVS) \
        return SSL_STATUS_BAD_HANDLE; \
}

/* Macro to check if SSL driver intialised or not */
#define CHK_SSL_INITIALIZED(val) \
   if (!val) { \
      VPROC_DBG_PRINT(VPROC_DBG_LVL_ERR,"Driver not initialised\n"); \
      return SSL_STATUS_NOT_INIT; \
   }

#define CHK_LOCK_HANDLE_VALIDITY(handle) \
   for(i=0;i<NUM_MAX_LOCKS;i++) { \
      if(handle == (ssl_lock_handle_t)&(ssl_drv_priv.lock[i])) break; \
   } \
   if(i>=NUM_MAX_LOCKS) return SSL_STATUS_BAD_HANDLE

ssl_status_t SSL_init(ssl_drv_cfg_t *pCfg)
{
    ssl_status_t status=SSL_STATUS_OK;
    VPROC_DBG_PRINT(VPROC_DBG_LVL_FUNC,"Enter...\n");

    /* if driver is already initialised, do nothing. return okay.
      Please note, we are not maintaining initialization count of the 
      driver here. Once initialized by any layer, will be returned okay 
      to subsequent calls regardless of where it is made from.
    */
    if(ssl_drv_priv.initialised == TRUE)
    {
        VPROC_DBG_PRINT(VPROC_DBG_LVL_INFO,"SSL Driver already initialised\n");
        return SSL_STATUS_OK;
    }
    SSL_memset(&ssl_drv_priv,0,sizeof(struct ssl_drv));
    if(!hal_init())
    {
        ssl_drv_priv.initialised = TRUE;
    }
    else
        status = SSL_STATUS_INTERNAL_ERR;

    VPROC_DBG_PRINT(VPROC_DBG_LVL_FUNC,"Exit...\n");
    return status;
}

ssl_status_t SSL_port_open(ssl_port_handle_t *pHandle,ssl_dev_cfg_t *pDevCfg)
{
   ssl_status_t status=SSL_STATUS_OK;
   int32_t      i;

   VPROC_DBG_PRINT(VPROC_DBG_LVL_FUNC,"Enter...\n");

   /* Check if driver is initialised */
   CHK_SSL_INITIALIZED(ssl_drv_priv.initialised);

   /* Check for bad parameters */
   if(pHandle == NULL || pDevCfg == NULL)
   {
      VPROC_DBG_PRINT(VPROC_DBG_LVL_ERR,"NULL parameter pointer passed\n");
      return SSL_STATUS_INVALID_ARG;
   }

   /* Check if port can be opened */
   for(i=0;i<VPROC_MAX_NUM_DEVS;i++)
   {
     if(!(ssl_drv_priv.dev[i].inuse))
         break;
   }

   if(i >= VPROC_MAX_NUM_DEVS)
   {
     VPROC_DBG_PRINT(VPROC_DBG_LVL_ERR,
                     "Cannot open device. Limit exceed Maximum Allowed." \
                    " Please refer to platform file for maximum allowed port.\n");
     return SSL_STATUS_RESOURCE_ERR;
   }

   /* open device port */
   if(hal_open(&(ssl_drv_priv.dev[i].pClient),pDevCfg)<0)
   {
     VPROC_DBG_PRINT(VPROC_DBG_LVL_ERR,"HAL Port open failed\n");
     return  SSL_STATUS_INTERNAL_ERR;
   }

   ssl_drv_priv.dev[i].inuse = TRUE;
   *pHandle = (ssl_port_handle_t)&(ssl_drv_priv.dev[i]);

   VPROC_DBG_PRINT(VPROC_DBG_LVL_FUNC,"Exit...\n");
   return status;
}

ssl_status_t SSL_port_close(ssl_port_handle_t handle)
{
   ssl_status_t   status=SSL_STATUS_OK;
   int8_t         i;
   struct ssl_dev *pDev = (struct ssl_dev *)handle;

   VPROC_DBG_PRINT(VPROC_DBG_LVL_FUNC,"Enter...\n");

   CHK_SSL_INITIALIZED(ssl_drv_priv.initialised);
   CHK_PORT_HANDLE_VALIDITY(pDev);

   if(hal_close(pDev->pClient)<0)
   {
     VPROC_DBG_PRINT(VPROC_DBG_LVL_ERR,"HAL Port close failed\n");
     return SSL_STATUS_INTERNAL_ERR;
   }
   pDev->inuse = FALSE;

   VPROC_DBG_PRINT(VPROC_DBG_LVL_FUNC,"Exit...\n");

   return status;
}

ssl_status_t SSL_term(void)
{
   ssl_status_t  status=SSL_STATUS_OK;
   int32_t        i;

   VPROC_DBG_PRINT(VPROC_DBG_LVL_FUNC,"Enter...\n");

   if(ssl_drv_priv.initialised != TRUE)
   {
      VPROC_DBG_PRINT(VPROC_DBG_LVL_INFO,"SSL Driver not initialised \n");
      return SSL_STATUS_OK;
   }

   /* Check if any of the resource is in use. if so, return error with message.
      This also means that if there are driver has been initialised multiple 
      times as long as layer hold any resource it cannot be terminated */
   for(i=0;i<VPROC_MAX_NUM_DEVS;i++)
   {
     if(ssl_drv_priv.dev[i].inuse)
     {
         VPROC_DBG_PRINT(VPROC_DBG_LVL_ERR,
                        "Can't terminate driver. please close all ports\n");
         return SSL_STATUS_FAILED;
     }
   }

   if(hal_term()<0)
   {
      VPROC_DBG_PRINT(VPROC_DBG_LVL_ERR,"hal_term() failed\n");
      status = SSL_STATUS_INTERNAL_ERR;
   }

   ssl_drv_priv.initialised = FALSE;

   VPROC_DBG_PRINT(VPROC_DBG_LVL_FUNC,"Exit...\n");

   return status;
}

ssl_status_t SSL_port_rw(ssl_port_handle_t handle,ssl_port_access_t *pPortAccess)
{
   ssl_status_t   status=SSL_STATUS_OK;
   struct ssl_dev *pDev = (struct ssl_dev *)handle;
   int32_t        i;
   size_t         num_bytes_read=0;
   size_t         num_bytes_write=0;

   VPROC_DBG_PRINT(VPROC_DBG_LVL_FUNC,"Enter...\n");

   CHK_SSL_INITIALIZED(ssl_drv_priv.initialised);
   CHK_PORT_HANDLE_VALIDITY(pDev);
   
   if(pPortAccess == NULL)
   {
      VPROC_DBG_PRINT(VPROC_DBG_LVL_ERR,"NULL port access info passed\n");
      return SSL_STATUS_INVALID_ARG;
   }

   num_bytes_read  = pPortAccess->nread;
   num_bytes_write = pPortAccess->nwrite;

   if(hal_port_rw(pDev->pClient,pPortAccess) <0)
   {
     VPROC_DBG_PRINT(VPROC_DBG_LVL_ERR,"HAL Port read/write failed.\n");
     return SSL_STATUS_INTERNAL_ERR;
   }

   if(((pPortAccess->op_type & SSL_OP_PORT_RD) && 
       (pPortAccess->nread < num_bytes_read))  ||
       ((pPortAccess->op_type & SSL_OP_PORT_WR) && 
       (pPortAccess->nwrite < num_bytes_write)))
   {
     status = SSL_STATUS_OP_INCOMPLETE;
   }

   VPROC_DBG_PRINT(VPROC_DBG_LVL_FUNC,"Exit...\n");

   return status;
}

ssl_status_t SSL_port_write(ssl_port_handle_t handle,void *pSrc, size_t *pSize)
{
   ssl_status_t      status=SSL_STATUS_OK;
   struct ssl_dev    *pDev = (struct ssl_dev *)handle;
   ssl_port_access_t port_access;
   int               i;

   VPROC_DBG_PRINT(VPROC_DBG_LVL_FUNC,"Enter...\n");

   CHK_SSL_INITIALIZED(ssl_drv_priv.initialised);
   CHK_PORT_HANDLE_VALIDITY(pDev);
   if(pSrc == NULL)
   {
      VPROC_DBG_PRINT(VPROC_DBG_LVL_ERR,"NULL param passed.\n");
      return SSL_STATUS_INVALID_ARG;
   }

   port_access.pDst = NULL;
   port_access.pSrc = pSrc;
   port_access.nread = 0;
   port_access.nwrite = *pSize;
   port_access.op_type = SSL_OP_PORT_WR;

   VPROC_DBG_PRINT(VPROC_DBG_LVL_INFO,
                  "Writing to client 0x%x\n",
                  (unsigned int)(pDev->pClient));

   if(hal_port_rw(pDev->pClient,&port_access) < 0)
   {
     VPROC_DBG_PRINT(VPROC_DBG_LVL_ERR,"HAL port write failed.n");
     return SSL_STATUS_INTERNAL_ERR;
   }

   if((port_access.op_type & SSL_OP_PORT_WR) && (port_access.nwrite < *pSize))
   {
      status = SSL_STATUS_OP_INCOMPLETE;
      *pSize = port_access.nwrite;
   }

   VPROC_DBG_PRINT(VPROC_DBG_LVL_FUNC,"Exit...\n");

   return status;
}

ssl_status_t SSL_memset(void *pDst, int32_t val,size_t size)
{
    VPROC_DBG_PRINT(VPROC_DBG_LVL_FUNC,"Enter...\n");

    if(pDst == NULL)
    {
        VPROC_DBG_PRINT(VPROC_DBG_LVL_ERR,"NULL destination pointer\n");
        return SSL_STATUS_INVALID_ARG;
    }
    
    memset(pDst,val,size);

    VPROC_DBG_PRINT(VPROC_DBG_LVL_FUNC,"Exit...\n");

    return SSL_STATUS_OK;
}

ssl_status_t SSL_memcpy(void *pDst,const void *pSrc, size_t size)
{
    VPROC_DBG_PRINT(VPROC_DBG_LVL_FUNC,"Enter...\n");

    if(pSrc == NULL || pDst == NULL)
    {
        VPROC_DBG_PRINT(VPROC_DBG_LVL_ERR,"NULL src or dst pointer\n");
        return SSL_STATUS_INVALID_ARG;
    }
    
    memcpy(pDst,pSrc,size);

    VPROC_DBG_PRINT(VPROC_DBG_LVL_FUNC,"Exit...\n");

    return SSL_STATUS_OK;
}

ssl_status_t SSL_delay(uint32_t tms)
{
    mdelay(tms);
    return SSL_STATUS_OK;
}


ssl_status_t SSL_lock_create(ssl_lock_handle_t *pLock, 
                              const char *pName, void *pOption)
{
   int32_t i;

   VPROC_DBG_PRINT(VPROC_DBG_LVL_FUNC,"Enter...\n");

   if(pLock == NULL)
   {
     VPROC_DBG_PRINT(VPROC_DBG_LVL_ERR,"Invalid Parameter list\n");
     return SSL_STATUS_INVALID_ARG;
   }

   for(i=0;i<NUM_MAX_LOCKS;i++)
   {
     if(!(ssl_drv_priv.lock[i].inuse))
         break;
   }

   if(i>=NUM_MAX_LOCKS)
   {
      VPROC_DBG_PRINT(VPROC_DBG_LVL_ERR,
                     "Unable to create a lock.resources exhausted\n");
      return SSL_STATUS_RESOURCE_ERR;
   }

   mutex_init(&(ssl_drv_priv.lock[i].lock));

   if(pName)
      strncpy(ssl_drv_priv.lock[i].name,pName,sizeof(ssl_drv_priv.lock[i].name));

   ssl_drv_priv.lock[i].inuse = TRUE;

   *pLock = (ssl_lock_handle_t )&(ssl_drv_priv.lock[i]);

   VPROC_DBG_PRINT(VPROC_DBG_LVL_FUNC,"Exit...\n");     
   return SSL_STATUS_OK;
}

/* wait type parameter if doesnt match to any value as in enum 
   SSL_WAIT_TYPE then will be assumed as timeout in millisecond*/
ssl_status_t SSL_lock(ssl_lock_handle_t lock_id,ssl_wait_t wait_type)
{
   int32_t i;

   CHK_LOCK_HANDLE_VALIDITY(lock_id);

   switch(wait_type)
   {
      case SSL_WAIT_NONE:
         mutex_trylock(&(((struct ssl_lock*)lock_id)->lock));
      break;
      case SSL_WAIT_FOREVER:
         mutex_lock(&(((struct ssl_lock*)lock_id)->lock));
      break;
      default:
         VPROC_DBG_PRINT(VPROC_DBG_LVL_ERR,
                       "timed wait not supported by this SSL implementation\n");
         return SSL_STATUS_INVALID_ARG;
   }

   return SSL_STATUS_OK;
}

ssl_status_t SSL_unlock(ssl_lock_handle_t lock_id)
{
    int32_t i;

    CHK_LOCK_HANDLE_VALIDITY(lock_id);
    mutex_unlock(&(((struct ssl_lock*)lock_id)->lock));

    return SSL_STATUS_OK;
}

ssl_status_t SSL_lock_delete(ssl_lock_handle_t lock_id)
{
    struct ssl_lock *pLock;
    int32_t i;
    
    CHK_LOCK_HANDLE_VALIDITY(lock_id);
    
    pLock = (struct ssl_lock *)lock_id;
    mutex_destroy(&(pLock->lock));
    pLock->inuse = FALSE;

    VPROC_DBG_PRINT(VPROC_DBG_LVL_FUNC,"Exit...\n");

    return SSL_STATUS_OK;
}


