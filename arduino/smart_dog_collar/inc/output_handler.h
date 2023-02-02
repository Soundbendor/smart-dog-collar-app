#ifndef OUTPUT_HANDLER_H
#define OUTPUT_HANDLER_H

#include <ArduinoBLE.h>

#define LABEL_COUNT 7
#define SEIZURE 4

class Output_Handler{
/*************/
/* Variables */
/*************/
private:
    const char *labels[LABEL_COUNT] = { "car", "leisure", "play", "run_jog",
                                        "seizure", "sleep", "walk" };
/***********/
/* Methods */
/***********/
public:
    void handleOutput(int);
};

#endif