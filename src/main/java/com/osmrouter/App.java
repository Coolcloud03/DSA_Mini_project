package com.osmrouter;

import static spark.Spark.*;

import com.google.gson.Gson;
import java.io.File;
import java.util.*;

public class App {
    private static final Gson gson = new Gson();

    public static void main(String[] args) {

        // asking for the OSM file path via env var or system property
        String osmPath = "C:/Users/shrijay/Downloads/map.osm";

        File f = new File(osmPath);
        if (!f.exists()) {
            System.err.println("ERROR: OSM file not found: " + f.getAbsolutePath());
            System.exit(1);
        }
        // build the graph osm
        System.out.println("Parsing OSM file: " + f.getAbsolutePath());
        // calling the function in OSMGraphBuilder to build the graph
        OSMGraphBuilder builder = new OSMGraphBuilder();

        // builder.buildGraph returns a Graph object containing nodes and adjacency list
        OSMGraphBuilder.Graph graph = builder.buildGraph(f);
        System.out.println("Loaded nodes: " + graph.nodeIdToNode.size() + ", edges: " + graph.adjacency.values().stream().mapToInt(List::size).sum());

        // calling DijkstraRouter to create a router instance
        DijkstraRouter router = new DijkstraRouter(graph.adjacency, graph.nodeIdToNode);

        // setting up SparkJava HTTP server
        port(Integer.parseInt(Optional.ofNullable(System.getenv("PORT")).orElse("4569")));
        staticFiles.location("/public");

        get("/health", (req, res) -> {
            res.type("application/json");
            return gson.toJson(Map.of("status", "ok"));
        });

        // serve index for root path
        get("/", (req, res) -> {
            res.redirect("/index.html");
            return null;
        });

        get("/nearest", (req, res) -> {
            res.type("application/json");
            String slat = req.queryParams("lat");
            String slon = req.queryParams("lon");
            if (slat == null || slon == null) {
                res.status(400);
                return gson.toJson(Map.of("error", "lat and lon are required"));
            }
            double lat, lon;
            try {
                lat = Double.parseDouble(slat);
                lon = Double.parseDouble(slon);
            } catch (Exception e) {
                res.status(400);
                return gson.toJson(Map.of("error", "invalid lat/lon"));
            }

            long nearestId = -1L;
            double best = Double.POSITIVE_INFINITY;
            for (OSMGraphBuilder.Node n : graph.nodeIdToNode.values()) {
                double d = GeoUtils.haversineMeters(lat, lon, n.lat, n.lon);
                if (d < best) {
                    best = d;
                    nearestId = n.id;
                }
            }
            if (nearestId == -1L) {
                res.status(404);
                return gson.toJson(Map.of("error", "no nodes"));
            }
            OSMGraphBuilder.Node nn = graph.nodeIdToNode.get(nearestId);
            return gson.toJson(Map.of(
                "id", nearestId,
                "lat", nn.lat,
                "lon", nn.lon
            ));
        });

        get("/route", (req, res) -> {
            res.type("application/json");
            String sFrom = req.queryParams("from");
            String sTo = req.queryParams("to");
            if (sFrom == null || sTo == null) {
                res.status(400);
                return gson.toJson(Map.of("error", "from and to are required (node ids)"));
            }
            long fromId, toId;
            try {
                fromId = Long.parseLong(sFrom);
                toId = Long.parseLong(sTo);
            } catch (Exception e) {
                res.status(400);
                return gson.toJson(Map.of("error", "invalid from/to"));
            }

            if (!graph.nodeIdToNode.containsKey(fromId) || !graph.nodeIdToNode.containsKey(toId)) {
                res.status(404);
                return gson.toJson(Map.of("error", "from/to node not found in graph"));
            }

            List<Long> path = router.shortestPath(fromId, toId);
            if (path.isEmpty()) {
                res.status(404);
                return gson.toJson(Map.of("error", "no route found"));
            }
            List<double[]> coords = new ArrayList<>();
            for (Long nid : path) {
                OSMGraphBuilder.Node n = graph.nodeIdToNode.get(nid);
                coords.add(new double[] { n.lat, n.lon });
            }
            Map<String, Object> out = new HashMap<>();
            out.put("count", coords.size());
            out.put("coordinates", coords);
            return gson.toJson(out);
        });
    }
}


