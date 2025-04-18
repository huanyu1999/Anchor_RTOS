/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : app_sdCard.c
  * @brief          : 日志文件写入，具体实现：写入时间，写入内容，文件创建，文件管理
  ******************************************************************************
  * @attention
  ******************************************************************************
  */
#include "main.h"
#include "dev.h"

#include "ff.h"
#include "ffconf.h"
// #include "app_sdCard.h"


/**************************************************************FatFs support**************************************************************/
FATFS SDFatFs;  /* File system object for SD card logical drive */
FIL MyFile;     /* File object */
char SDPath[4]; /* SD card logical drive path */
static uint8_t workBuffer[FF_MAX_SS]; /* a work buffer for the f_mkfs() */

  void task7_sdCard(void *arg)
  {
      FRESULT res;                                          /* FatFs function common result code */
      uint32_t byteswritten, bytesread;                     /* File write/read counts */
      uint8_t wtext[] = "This is STM32 working with FatFs"; /* File write buffer */
      uint8_t rtext[100];                                   /* File read buffer */
  
      /*##-1- Link the micro SD disk I/O driver ##################################*/
      if (dev_FATFS_LinkDriver(&SDCard_driver, SDPath) == 0)
      {
          /*##-2- Register the file system object to the FatFs module ##############*/
          if (f_mount(&SDFatFs, (TCHAR const*)SDPath, 0) != FR_OK)
          {
              /* FatFs Initialization Error */
              log_e("f_mount failed.");
          }
          else
          {
              /*##-3- Create a FAT file system (format) on the logical drive #########*/
              /* WARNING: Formatting the uSD card will delete all content on the device */
              if(f_mkfs((TCHAR const*)SDPath, FM_FAT32, 0, workBuffer, sizeof(workBuffer)) != FR_OK)
              {
                  /* FatFs Format Error */
                  // Error_Handler();
                  log_e("f_mkfs failed.");
              }
              else
              {
                  /*##-4- Create and Open a new text file object with write access #####*/
                  if(f_open(&MyFile, "STM32.TXT", FA_CREATE_ALWAYS | FA_WRITE) != FR_OK)
                  {       
                      /* 'STM32.TXT' file Open for write Error */
                      // Error_Handler();
                      log_e("f_open failed.");
                  }
                  else
                  {
                      /*##-5- Write data to the text file ################################*/
                      res = f_write(&MyFile, wtext, sizeof(wtext), (void *)&byteswritten);
  
                      if((byteswritten == 0) || (res != FR_OK))
                      {
                          /* 'STM32.TXT' file Write or EOF Error */
                          // Error_Handler();
                          log_e("'STM32.TXT' file Write or EOF Error.");
                      }
                      else
                      {
                          /*##-6- Close the open text file #################################*/
                          f_close(&MyFile);
              
                          /*##-7- Open the text file object with read access ###############*/
                          if(f_open(&MyFile, "STM32.TXT", FA_READ) != FR_OK)
                          {
                              /* 'STM32.TXT' file Open for read Error */
                              log_e("'STM32.TXT' file Open for read Error.");
                          }
                          else
                          {
                              /*##-8- Read data from the text file ###########################*/
                              res = f_read(&MyFile, rtext, sizeof(rtext), (UINT*)&bytesread);
                              
                              if((bytesread == 0) || (res != FR_OK))
                              {
                                  /* 'STM32.TXT' file Read or EOF Error */
                                  log_e("'STM32.TXT' file Read or EOF Error.");Error_Handler();
                              }
                              else
                              {
                                  /*##-9- Close the open text file #############################*/
                                  f_close(&MyFile);
                                  
                                  /*##-10- Compare read data with the expected data ############*/
                                  if((bytesread != byteswritten))
                                  {                
                                      /* Read data is different from the expected data */
                                      log_e("Read data is different from the expected data.");
                                  }
                                  else
                                  {
                                      /* Success of the demo: no error occurrence */
                                      // BSP_LED_On(LED1);
                                      log_d("Success of the sd card demo.");
                                  }
                              }       
                          }
                      }
                  }
              }
          }
      }
  
      /*##-11- Unlink the RAM disk I/O driver ####################################*/
      dev_FATFS_UnLinkDriver(SDPath);
  
      for (;;)
      {
      }
  }