#include "progress.h"

///////////////////////////////////////////////////////////////////////////////
void printfProgress(float current_value, float min_value, float max_value)
{
    if (current_value < min_value)
        current_value = min_value;

    if (current_value > max_value)
        current_value = max_value;

    float percent = (current_value - min_value) / (max_value - min_value) * 100;

    printf("\r [ "); // go back to start of line

    int i = 0;
    for (; i < percent; i++) {
        putchar('#');
    }

    for (; i < 100; i++) {
        putchar(' ');
    }

    printf(" ] %3.0f %%", percent);

    if (current_value == max_value)
        putchar('\n');

    // force drawing by flushing buffer
    fflush(stdout);
}
///////////////////////////////////////////////////////////////////////////////
