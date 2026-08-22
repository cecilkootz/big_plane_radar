#include "airport_lookup.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

using RadarAirports::degreesFromE5;
using RadarAirports::findByIata;
using RadarAirports::findByIcao;

#define CHECK(condition) do { \
    if (!(condition)) { \
        fprintf(stderr, "CHECK failed at %s:%d: %s\n", __FILE__, __LINE__, #condition); \
        exit(1); \
    } \
} while (0)

// The binary search relies on the generator emitting the catalog sorted.
static void testCatalogIsSortedByIcao() {
    for (size_t i = 1; i < kAirportCatalogCount; i++) {
        CHECK(strcmp(kAirportCatalog[i - 1].icao, kAirportCatalog[i].icao) < 0);
    }
}

static void testFindByIcao() {
    const AirportCatalogEntry *jfk = findByIcao("KJFK");
    CHECK(jfk != nullptr);
    CHECK(strcmp(jfk->iata, "JFK") == 0);
    CHECK(fabsf(degreesFromE5(jfk->latE5) - 40.63945f) < 0.0001f);
    CHECK(fabsf(degreesFromE5(jfk->lonE5) - (-73.77932f)) < 0.0001f);

    CHECK(findByIcao(kAirportCatalog[0].icao) == &kAirportCatalog[0]);
    CHECK(findByIcao(kAirportCatalog[kAirportCatalogCount - 1].icao) ==
          &kAirportCatalog[kAirportCatalogCount - 1]);

    CHECK(findByIcao("AAAA") == nullptr);
    CHECK(findByIcao("") == nullptr);
    CHECK(findByIcao(nullptr) == nullptr);
}

static void testFindByIata() {
    const AirportCatalogEntry *san = findByIata("SAN");
    CHECK(san != nullptr);
    CHECK(strcmp(san->icao, "KSAN") == 0);
    CHECK(fabsf(degreesFromE5(san->latE5) - 32.7336f) < 0.0001f);

    CHECK(findByIata("ZZZ") == nullptr);
    CHECK(findByIata("") == nullptr);
    CHECK(findByIata(nullptr) == nullptr);
}

int main() {
    testCatalogIsSortedByIcao();
    testFindByIcao();
    testFindByIata();
    printf("airport lookup tests passed\n");
    return 0;
}
