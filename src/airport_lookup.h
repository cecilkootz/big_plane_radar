#pragma once

#include <string.h>

#include "airport_catalog.h"

namespace RadarAirports {

inline float degreesFromE5(int32_t valueE5) {
    return static_cast<float>(valueE5) * 1e-5f;
}

// The catalog is generated sorted by ICAO code.
inline const AirportCatalogEntry *findByIcao(const char *icao) {
    if (icao == nullptr || icao[0] == '\0') return nullptr;
    size_t low = 0;
    size_t high = kAirportCatalogCount;
    while (low < high) {
        size_t middle = low + (high - low) / 2;
        int comparison = strcmp(kAirportCatalog[middle].icao, icao);
        if (comparison < 0) {
            low = middle + 1;
        } else if (comparison > 0) {
            high = middle;
        } else {
            return &kAirportCatalog[middle];
        }
    }
    return nullptr;
}

inline const AirportCatalogEntry *findByIata(const char *iata) {
    if (iata == nullptr || iata[0] == '\0') return nullptr;
    for (size_t i = 0; i < kAirportCatalogCount; i++) {
        if (kAirportCatalog[i].iata[0] != '\0' &&
            strcmp(kAirportCatalog[i].iata, iata) == 0) {
            return &kAirportCatalog[i];
        }
    }
    return nullptr;
}

}  // namespace RadarAirports
