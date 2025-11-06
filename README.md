java -Dosm.file="C:\Users\Shrijay\Downloads\map.osm" -jar target\osm-router-1.0.0.jar

# OSM Shortest Path Router (Java 17)

Compute and visualize shortest driving paths on OpenStreetMap data using a manual Dijkstra implementation. No external routing APIs.

## Features
- Parses `.osm` XML via SAX
- Builds a graph from `highway` ways; supports `oneway` yes/-1
- Edge weights by Haversine meters
- Dijkstra shortest path
- HTTP API with SparkJava
  - `GET /nearest?lat=&lon=` → nearest node id
  - `GET /route?from=&to=` → list of `[lat, lon]`
- Static frontend (Leaflet) to pick points and visualize route

## Requirements
- Java 17+
- Maven 3.8+
- An `.osm` extract for your area (e.g., from https://download.geofabrik.de/)

## Build
```bash
mvn -q -DskipTests package
```

## Run
Provide the OSM file via `OSM_FILE` env or `-Dosm.file` system property.
```bash
# Example (Windows)
set OSM_FILE=C:\\data\\your-area.osm
java -jar target\\osm-router-1.0.0-shaded.jar
# Or
java -Dosm.file=C:\\data\\your-area.osm -jar target\\osm-router-1.0.0-shaded.jar
```
Server listens on port 4567 by default (`PORT` env overrides).

Open the UI at `http://localhost:4567/`.

## API Examples
```bash
curl "http://localhost:4567/nearest?lat=52.52&lon=13.405"
curl "http://localhost:4567/route?from=123456&to=789012"
```

## Notes
- Parsing large OSM files can take time and memory. Prefer smaller regional extracts.
- Bidirectionality is assumed unless a way has `oneway=yes/true/1` (forward only) or `oneway=-1` (reverse only).
