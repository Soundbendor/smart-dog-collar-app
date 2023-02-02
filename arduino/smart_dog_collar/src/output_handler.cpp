#include "../inc/output_handler.h"

// TODO 4: handle output in some way
void Output_Handler::handleOutput(int activity)
{ 
// Handle seizure
if(activity == SEIZURE)
{
    // Send push notification?

}

// Everything that must be done
// Send status to phone

#ifdef SMART_DOG_COLLAR_DEBUG
// Check what was the result of the model
Serial.print(labels[activity]);
#endif
}