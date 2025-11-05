#include "wdt_drv.h"
#include "wdt_reg.h"

/*******************************************************************************
* Function Name  : Wdt_FeedDog
* Description    : Feed (refresh) the watchdog timer to prevent system reset.
* Input          : - WDT : Pointer to the base address of the Watchdog module.
*
* Output         : None
* Return         : None
*******************************************************************************/
void Wdt_FeedDog(WDT_TypeDef* WDT) {
	WDT->WDT_WSR = 0x5555;
	WDT->WDT_WSR = 0xAAAA;
}

/*******************************************************************************
* Function Name  : Wdt_EnableFunc
* Description    : Enable the watchdog timer functionality.
* Input          : - WDT : Pointer to the base address of the Watchdog module.
*
* Output         : None
* Return         : None
*******************************************************************************/
void Wdt_EnableFunc(WDT_TypeDef* WDT) {
	WDT->WDT_WCR |= WDT_EN;
}

/*******************************************************************************
* Function Name  : Wdt_DisableFunc
* Description    : Disable the watchdog timer functionality.
* Input          : - WDT : Pointer to the base address of the Watchdog module.
*
* Output         : None
* Return         : None
*******************************************************************************/
void Wdt_DisableFunc(WDT_TypeDef* WDT) {
	WDT->WDT_WCR &= ~WDT_EN;
}

/*******************************************************************************
* Function Name  : Wdt_SetMode
* Description    : Configure the watchdog timer operating mode.
* Input          : - WDT     : Pointer to the base address of the Watchdog module.
*                  - WdtMode : Watchdog operating mode to be set.
*
* Output         : None
* Return         : None
*******************************************************************************/
void Wdt_SetMode(WDT_TypeDef* WDT, uint8_t WdtMode) {
	WDT->WDT_WCR |= WdtMode;
}

/*******************************************************************************
* Function Name  : Wdt_IsEnabled
* Description    : Check whether the watchdog timer is enabled.
* Input          : - WDT : Pointer to the base address of the Watchdog module.
*
* Output         : None
* Return         : true  - Watchdog is enabled  
*                  false - Watchdog is disabled
*******************************************************************************/
bool Wdt_IsEnabled(WDT_TypeDef* WDT) {
	uint16_t ret = WDT->WDT_WCR;
	return (ret & WDT_EN) ? true : false;
}

/*******************************************************************************
* Function Name  : Wdt_IsInDbg
* Description    : Check whether the watchdog timer is in debug mode.
* Input          : - WDT : Pointer to the base address of the Watchdog module.
*
* Output         : None
* Return         : true  - Watchdog is in debug mode
*                  false - Watchdog is not in debug mode
*******************************************************************************/
bool Wdt_IsInDbg(WDT_TypeDef* WDT) {
	uint16_t ret = WDT->WDT_WCR;
	return (ret & WDT_DBG) ? true : false;
}

/*******************************************************************************
* Function Name  : Wdt_SetCnt
* Description    : Set the watchdog timer's reload (counter) value.
* Input          : - WDT      : Pointer to the base address of the Watchdog module.
*                  - WdtCount : Reload value to be set for the watchdog counter.
*
* Output         : None
* Return         : None
*******************************************************************************/
void Wdt_SetCnt(WDT_TypeDef* WDT, uint16_t WdtCount) {
	WDT->WDT_WMR = WdtCount;
}

