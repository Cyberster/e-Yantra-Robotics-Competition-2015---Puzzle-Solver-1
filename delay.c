
 
 
 /***************************************************************************************************
                             Revision History
****************************************************************************************************				   
15.0: Initial version 
***************************************************************************************************/
#include<util\delay.h>
#include"delay.h"



/***************************************************************************************************
                        void DELAY_sec(uint16_t var_delaySecCount_u16)
****************************************************************************************************
  * I/P Arguments: uint16_t.
  * Return value	: none

  * description:
      This function is used generate delay in sec .
      It generates a delay of 1sec for each count,
      if 10 is passed as the argument then it generates delay of 10sec
***************************************************************************************************/
void DELAY_sec(uint16_t var_delaySecCount_u16)
 {
	 while(var_delaySecCount_u16!=0)
	  {
	     DELAY_ms(1000);	      /* DELAY_ms is called to generate 1sec delay */
		 var_delaySecCount_u16--;
		}
  }
