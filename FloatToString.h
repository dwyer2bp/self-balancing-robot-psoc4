/* ========================================
 * UartPrintString cannot print floating point values
 * So this was created to take care of float-to-string
 * This truncates to nearest 1-thousandth
 * Author: Brian Dwyer
*/
#include <project.h>
#include <stdio.h>
void ftostring(float f, char *res);


void ftostring(float f, char *res){
    int whole = (int)f;
    int thou = (f-(float)whole)*1000;
    
    if( whole < 0 || thou < 0 )
    {
       snprintf(res,32, "-%d.%d", -1*whole, -1*thou); 
    }
    else
    {
       snprintf(res,32, "%d.%d", whole, thou); 
    }
    
}
/* [] END OF FILE */
