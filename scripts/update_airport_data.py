#!/usr/bin/env python3
"""Download OurAirports data and build the deterministic runway catalog."""

from __future__ import annotations

import argparse
import csv
import hashlib
import math
import os
import shutil
import subprocess
import sys
import tempfile
import urllib.error
import urllib.request
from dataclasses import dataclass
from pathlib import Path


AIRPORTS_URL = "https://davidmegginson.github.io/ourairports-data/airports.csv"
RUNWAYS_URL = "https://davidmegginson.github.io/ourairports-data/runways.csv"
AIRPORT_TYPES = {"large_airport", "medium_airport"}
MIN_AIRPORT_ROWS = 50_000
MIN_RUNWAY_ROWS = 30_000


@dataclass(frozen=True)
class Airport:
    source_ident: str
    icao: str
    iata: str
    latitude: float
    longitude: float


@dataclass(frozen=True)
class Runway:
    latitude: float
    longitude: float
    heading: float
    length_m: int


def parse_float(value: str | None) -> float | None:
    if value is None or not value.strip():
        return None
    try:
        result = float(value)
    except ValueError:
        return None
    return result if math.isfinite(result) else None


def c_escape(value: str) -> str:
    return value.replace("\\", "\\\\").replace('"', '\\"')


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def download(url: str, destination: Path) -> None:
    destination.parent.mkdir(parents=True, exist_ok=True)
    request = urllib.request.Request(url, headers={"User-Agent": "big-plane-radar-data-builder/1"})
    try:
        with urllib.request.urlopen(request, timeout=60) as response:
            with tempfile.NamedTemporaryFile(dir=destination.parent, delete=False) as temporary:
                temporary_path = Path(temporary.name)
                while chunk := response.read(1024 * 1024):
                    temporary.write(chunk)
    except urllib.error.URLError as error:
        curl = shutil.which("curl")
        if curl is None:
            raise RuntimeError(
                f"Python HTTPS failed ({error}) and curl is unavailable"
            ) from error
        file_descriptor, temporary_name = tempfile.mkstemp(dir=destination.parent)
        os.close(file_descriptor)
        temporary_path = Path(temporary_name)
        try:
            subprocess.run(
                [
                    curl,
                    "--fail",
                    "--location",
                    "--silent",
                    "--show-error",
                    "--user-agent",
                    "big-plane-radar-data-builder/1",
                    url,
                    "--output",
                    str(temporary_path),
                ],
                check=True,
            )
        except (OSError, subprocess.CalledProcessError) as curl_error:
            temporary_path.unlink(missing_ok=True)
            raise RuntimeError(f"Unable to download {url}: {curl_error}") from curl_error
    os.replace(temporary_path, destination)


def validate_csv(path: Path, required_columns: set[str], minimum_rows: int) -> None:
    with path.open(newline="", encoding="utf-8") as source:
        reader = csv.DictReader(source)
        missing = required_columns.difference(reader.fieldnames or [])
        if missing:
            raise ValueError(f"{path}: missing columns: {', '.join(sorted(missing))}")
        row_count = sum(1 for _ in reader)
    if row_count < minimum_rows:
        raise ValueError(f"{path}: expected at least {minimum_rows} rows, found {row_count}")


def load_airports(path: Path) -> dict[str, Airport]:
    catalog: dict[str, Airport] = {}
    with path.open(newline="", encoding="utf-8") as source:
        for row in csv.DictReader(source):
            airport_type = row["type"].strip()
            if airport_type == "closed":
                continue

            iata = row["iata_code"].strip().upper()
            if len(iata) != 3:
                iata = ""

            if airport_type not in AIRPORT_TYPES:
                continue
            source_ident = row["ident"].strip().upper()
            icao = (row["icao_code"].strip() or source_ident).upper()
            latitude = parse_float(row["latitude_deg"])
            longitude = parse_float(row["longitude_deg"])
            if not source_ident or not icao or len(icao) > 4 or latitude is None or longitude is None:
                continue
            catalog[source_ident] = Airport(source_ident, icao, iata, latitude, longitude)

    return catalog


def bearing(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    delta_lon = math.radians(lon2 - lon1)
    y = math.sin(delta_lon) * math.cos(phi2)
    x = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(delta_lon)
    return (math.degrees(math.atan2(y, x)) + 360.0) % 360.0


def load_runways(path: Path, airports: dict[str, Airport]) -> dict[str, list[Runway]]:
    result: dict[str, list[Runway]] = {ident: [] for ident in airports}
    with path.open(newline="", encoding="utf-8") as source:
        for row in csv.DictReader(source):
            ident = row["airport_ident"].strip().upper()
            airport = airports.get(ident)
            if airport is None or row["closed"].strip() == "1":
                continue

            length_ft = parse_float(row["length_ft"])
            if length_ft is None or length_ft <= 0:
                continue
            length_m = max(1, min(65_535, round(length_ft * 0.3048)))

            le_lat = parse_float(row["le_latitude_deg"])
            le_lon = parse_float(row["le_longitude_deg"])
            he_lat = parse_float(row["he_latitude_deg"])
            he_lon = parse_float(row["he_longitude_deg"])
            if None not in (le_lat, le_lon, he_lat, he_lon):
                latitude = (le_lat + he_lat) * 0.5
                longitude = (le_lon + he_lon) * 0.5
                heading = bearing(le_lat, le_lon, he_lat, he_lon)
            else:
                latitude = airport.latitude
                longitude = airport.longitude
                heading = parse_float(row["le_heading_degT"])
                if heading is None:
                    heading = parse_float(row["he_heading_degT"])
                if heading is None:
                    continue

            result[ident].append(Runway(latitude, longitude, heading % 180.0, length_m))

    return {ident: runways for ident, runways in result.items() if runways}


def generated_comment(airports_hash: str, runways_hash: str) -> list[str]:
    lines = [
        "// Generated by scripts/update_airport_data.py. Do not edit manually.",
        f"// Airports source: {AIRPORTS_URL}",
        f"// airports.csv SHA-256: {airports_hash}",
    ]
    lines.extend([
        f"// Runways source: {RUNWAYS_URL}",
        f"// runways.csv SHA-256: {runways_hash}",
    ])
    return lines


def build_catalog_header(
    airports: dict[str, Airport],
    runways_by_airport: dict[str, list[Runway]],
    airports_hash: str,
    runways_hash: str,
) -> tuple[str, int, int]:
    airport_rows: list[tuple[Airport, int, int]] = []
    runway_rows: list[Runway] = []
    seen_icao: set[str] = set()
    source_idents = sorted(
        runways_by_airport,
        key=lambda source_ident: (airports[source_ident].icao, source_ident),
    )
    for source_ident in source_idents:
        airport = airports[source_ident]
        if airport.icao in seen_icao:
            continue
        seen_icao.add(airport.icao)
        first_runway = len(runway_rows)
        runways = sorted(
            runways_by_airport[source_ident],
            key=lambda item: (round(item.heading, 1), -item.length_m),
        )
        if len(runways) > 255:
            raise ValueError(f"{airport.icao}: runway count exceeds uint8_t storage")
        if first_runway > 65_535:
            raise ValueError("runway catalog exceeds uint16_t index storage")
        runway_rows.extend(runways)
        airport_rows.append((airport, first_runway, len(runways)))

    lines = generated_comment(airports_hash, runways_hash)
    lines.extend([
        "#pragma once",
        "",
        "#include <stddef.h>",
        "#include <stdint.h>",
        "",
        "struct AirportCatalogEntry {",
        "    int32_t latE5;",
        "    int32_t lonE5;",
        "    uint16_t firstRunway;",
        "    char icao[5];",
        "    char iata[4];",
        "    uint8_t runwayCount;",
        "};",
        "static_assert(sizeof(AirportCatalogEntry) == 20, \"Unexpected airport entry layout\");",
        "",
        "struct AirportRunwayEntry {",
        "    int32_t latE5;",
        "    int32_t lonE5;",
        "    uint16_t headingDeciDeg;",
        "    uint16_t lengthMeters;",
        "};",
        "static_assert(sizeof(AirportRunwayEntry) == 12, \"Unexpected runway entry layout\");",
        "",
        "static const AirportCatalogEntry kAirportCatalog[] = {",
    ])
    for airport, first_runway, runway_count in airport_rows:
        lines.append(
            f"    {{{round(airport.latitude * 100_000)}, {round(airport.longitude * 100_000)}, "
            f'{first_runway}, "{c_escape(airport.icao)}", "{c_escape(airport.iata)}", '
            f"{runway_count}}},"
        )
    lines.extend([
        "};",
        "",
        "static const AirportRunwayEntry kAirportRunways[] = {",
    ])
    for runway in runway_rows:
        lines.append(
            f"    {{{round(runway.latitude * 100_000)}, {round(runway.longitude * 100_000)}, "
            f"{round(runway.heading * 10)}, {runway.length_m}}},"
        )
    lines.extend([
        "};",
        "",
        "static constexpr size_t kAirportCatalogCount =",
        "    sizeof(kAirportCatalog) / sizeof(kAirportCatalog[0]);",
        "static constexpr size_t kAirportRunwayCount =",
        "    sizeof(kAirportRunways) / sizeof(kAirportRunways[0]);",
        "",
    ])
    return "\n".join(lines), len(airport_rows), len(runway_rows)


def write_or_check(path: Path, content: str, check: bool) -> bool:
    current = path.read_text(encoding="utf-8") if path.exists() else None
    if current == content:
        print(f"Up to date: {path}")
        return True
    if check:
        print(f"Out of date: {path}", file=sys.stderr)
        return False
    path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(
        mode="w", encoding="utf-8", dir=path.parent, delete=False
    ) as temporary:
        temporary.write(content)
        temporary_path = Path(temporary.name)
    os.replace(temporary_path, path)
    os.chmod(path, 0o644)
    print(f"Wrote: {path}")
    return True


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Download OurAirports CSV files and rebuild the firmware runway catalog."
    )
    parser.add_argument("--airports", type=Path, help="Use a local airports.csv file")
    parser.add_argument("--runways", type=Path, help="Use a local runways.csv file")
    parser.add_argument("--cache-dir", type=Path, default=Path(".cache/ourairports"))
    parser.add_argument("--offline", action="store_true", help="Use cached CSV files without downloading")
    parser.add_argument("--check", action="store_true", help="Exit non-zero when generated headers differ")
    parser.add_argument("--catalog-output", type=Path, default=Path("src/airport_catalog.h"))
    args = parser.parse_args()

    if (args.airports is None) != (args.runways is None):
        parser.error("--airports and --runways must be provided together")

    if args.airports is not None:
        airports_path = args.airports
        runways_path = args.runways
    else:
        airports_path = args.cache_dir / "airports.csv"
        runways_path = args.cache_dir / "runways.csv"
        if not args.offline:
            print(f"Downloading {AIRPORTS_URL}")
            download(AIRPORTS_URL, airports_path)
            print(f"Downloading {RUNWAYS_URL}")
            download(RUNWAYS_URL, runways_path)

    if not airports_path.exists() or not runways_path.exists():
        parser.error("CSV input is missing; run without --offline first or provide both local files")

    validate_csv(
        airports_path,
        {"ident", "type", "latitude_deg", "longitude_deg", "icao_code", "iata_code"},
        MIN_AIRPORT_ROWS,
    )
    validate_csv(
        runways_path,
        {"airport_ident", "length_ft", "closed", "le_latitude_deg", "le_longitude_deg", "le_heading_degT", "he_latitude_deg", "he_longitude_deg", "he_heading_degT"},
        MIN_RUNWAY_ROWS,
    )

    airports_hash = sha256(airports_path)
    runways_hash = sha256(runways_path)
    airports = load_airports(airports_path)
    runways_by_airport = load_runways(runways_path, airports)
    catalog_header, airport_count, runway_count = build_catalog_header(
        airports, runways_by_airport, airports_hash, runways_hash
    )

    ok = write_or_check(args.catalog_output, catalog_header, args.check)
    print(
        f"Generated {airport_count} airports and {runway_count} physical runways."
    )
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
