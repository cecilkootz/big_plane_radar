# Airport and Route Data

Big Plane Radar handles map runways and route city names independently:

- airports and runways are generated from OurAirports and compiled into flash;
- route cities are received from ADSBdb at runtime and cached only in RAM.

Users do not need to download or generate either dataset.

## Airport and Runway Settings

The device setup page provides:

- `Show airports and runways` to enable or disable the overlay;
- `Nearest to radar center` to select nearby airports automatically;
- `Airports shown` to retain one to three airports, with one as the default;
- `Search radius` from 10 to 500 km;
- `Manual ICAO code` to select one airport, for example `LEVC`.

Selection is recalculated after saving and rebooting. The global catalog is
scanned once; normal frame rendering visits only the selected airports and all
their physical runways. Each airport ICAO label is drawn once.

## Dynamic Route Cities

The ADSBdb callsign response already includes origin and destination IATA codes,
airport names, and municipalities. The firmware stores the returned municipality
directly in the active route entry and remembers up to 128 `IATA -> city` pairs
in an in-memory LRU cache.

The city cache is deliberately not written to flash. It fills automatically as
aircraft are seen and starts empty after a restart. Missing city names fall back
to their three-letter IATA codes.

Route text is normalized to the display's ASCII character set. Common Latin
accents are transliterated. Before drawing, both city names are measured with the
actual screen font. If the complete route is too wide, the visually longer city
is shortened first and receives `...`; shortening continues until the line fits.
IATA-to-IATA remains the final fallback.

## Runway Data Sources

The runway catalog is generated from the public OurAirports CSV export:

- [OurAirports data downloads](https://ourairports.com/data/)
- [OurAirports data repository](https://github.com/davidmegginson/ourairports-data)
- [airports.csv](https://davidmegginson.github.io/ourairports-data/airports.csv)
- [runways.csv](https://davidmegginson.github.io/ourairports-data/runways.csv)

The generated header records both source URLs and SHA-256 checksums. It omits a
generation timestamp, so identical inputs produce identical output.

## Update the Runway Catalog

Requirements: Python 3.10 or newer and internet access. No third-party Python
packages are required. The script falls back to system `curl` when a macOS
Python installation does not have a working CA certificate bundle.

From the repository root, run:

```sh
python3 scripts/update_airport_data.py
```

The command downloads current CSV files into `.cache/ourairports/`, validates
their columns and row counts, and atomically replaces:

```text
src/airport_catalog.h
```

It prints the generated airport and physical-runway counts. A download or
validation failure leaves the existing header untouched.

Regenerate without network access from the local cache:

```sh
python3 scripts/update_airport_data.py --offline
```

Verify the header without modifying it:

```sh
python3 scripts/update_airport_data.py --offline --check
```

Use archived or explicitly downloaded CSV files:

```sh
python3 scripts/update_airport_data.py \
  --airports /path/to/airports.csv \
  --runways /path/to/runways.csv
```

`--airports` and `--runways` must be supplied together.

## Generation Rules

The catalog includes open medium and large airports with at least one usable,
non-closed runway. Coordinates are stored at five decimal places, headings in
tenths of a degree, and runway lengths in meters.

After updating data, run:

```sh
python3 scripts/update_airport_data.py --offline --check
git diff --stat
CLEAN=1 bash build_arduino_cli.sh
```

Do not edit `src/airport_catalog.h` manually. Change the generator or source-data
handling, then regenerate it.

## Troubleshooting

If no airport is shown in automatic mode, increase the search radius and verify
the radar coordinates. Small airports and heliports are intentionally excluded.

If manual mode reports `ICAO code not found`, verify the ICAO code and regenerate
from current OurAirports data.

If a route shows IATA codes rather than cities, ADSBdb did not return usable
municipality names and the cities were not yet present in the RAM cache.
