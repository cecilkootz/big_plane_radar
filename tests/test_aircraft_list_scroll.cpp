#include "aircraft_list_scroll.h"

#include <stdio.h>
#include <stdlib.h>

#define CHECK(condition) do { \
    if (!(condition)) { \
        fprintf(stderr, "CHECK failed at %s:%d: %s\n", __FILE__, __LINE__, #condition); \
        exit(1); \
    } \
} while (0)

int main() {
    using AircraftListScroll::clampOffset;
    using AircraftListScroll::maxOffset;
    using AircraftListScroll::offsetForDrag;

    CHECK(maxOffset(8, 10) == 0);
    CHECK(maxOffset(18, 10) == 8);
    CHECK(clampOffset(20, 18, 10) == 8);

    CHECK(offsetForDrag(0, 19, 18, 10, 20, 36) == 0);
    CHECK(offsetForDrag(0, 20, 18, 10, 20, 36) == 1);
    CHECK(offsetForDrag(0, 56, 18, 10, 20, 36) == 2);
    CHECK(offsetForDrag(7, 200, 18, 10, 20, 36) == 8);
    CHECK(offsetForDrag(5, -20, 18, 10, 20, 36) == 4);
    CHECK(offsetForDrag(2, -200, 18, 10, 20, 36) == 0);

    puts("aircraft_list_scroll tests passed");
    return 0;
}
