package com.osmrouter;

import java.io.File;
import java.io.FileInputStream;
import java.util.*;
import javax.xml.parsers.SAXParser;
import javax.xml.parsers.SAXParserFactory;
import org.xml.sax.Attributes;
import org.xml.sax.helpers.DefaultHandler;

public class OSMGraphBuilder {
    public static class Node {
        public final long id;
        public final double lat;
        public final double lon;
        public Node(long id, double lat, double lon) {
            this.id = id;
            this.lat = lat;
            this.lon = lon;
        }
    }

    public static class Edge {
        public final long from;
        public final long to;
        public final double weightMeters;
        public Edge(long from, long to, double weightMeters) {
            this.from = from;
            this.to = to;
            this.weightMeters = weightMeters;
        }
    }

    public static class Graph {
        public final Map<Long, Node> nodeIdToNode;
        public final Map<Long, List<Edge>> adjacency;
        public Graph(Map<Long, Node> nodeIdToNode, Map<Long, List<Edge>> adjacency) {
            this.nodeIdToNode = nodeIdToNode;
            this.adjacency = adjacency;
        }
    }

    public Graph buildGraph(File osmFile) {
        Map<Long, Node> nodes = new HashMap<>();
        Map<Long, List<Edge>> graph = new HashMap<>();

        try {
            SAXParserFactory factory = SAXParserFactory.newInstance();
            factory.setNamespaceAware(false);
            factory.setValidating(false);
            SAXParser saxParser = factory.newSAXParser();

            DefaultHandler handler = new DefaultHandler() {
                boolean inWay = false;
                boolean wayIsHighway = false;
                boolean wayOnewayForward = false; // true if k=oneway yes/true/1
                boolean wayOnewayReverse = false; // true if k=oneway -1
                List<Long> wayNodeRefs = new ArrayList<>();

                @Override
                public void startElement(String uri, String localName, String qName, Attributes attributes) {
                    switch (qName) {
                        case "node": {
                            long id = Long.parseLong(attributes.getValue("id"));
                            double lat = Double.parseDouble(attributes.getValue("lat"));
                            double lon = Double.parseDouble(attributes.getValue("lon"));
                            nodes.put(id, new Node(id, lat, lon));
                            break;
                        }
                        case "way": {
                            inWay = true;
                            wayIsHighway = false;
                            wayOnewayForward = false;
                            wayOnewayReverse = false;
                            wayNodeRefs.clear();
                            break;
                        }
                        case "nd": {
                            if (inWay) {
                                String ref = attributes.getValue("ref");
                                if (ref != null) wayNodeRefs.add(Long.parseLong(ref));
                            }
                            break;
                        }
                        case "tag": {
                            if (inWay) {
                                String k = attributes.getValue("k");
                                String v = attributes.getValue("v");
                                if ("highway".equals(k)) {
                                    wayIsHighway = true;
                                }
                                if ("oneway".equals(k)) {
                                    if ("-1".equals(v)) wayOnewayReverse = true;
                                    if ("yes".equalsIgnoreCase(v) || "true".equalsIgnoreCase(v) || "1".equals(v)) wayOnewayForward = true;
                                }
                            }
                            break;
                        }
                    }
                }

                @Override
                public void endElement(String uri, String localName, String qName) {
                    if ("way".equals(qName)) {
                        if (wayIsHighway && wayNodeRefs.size() >= 2) {
                            for (int i = 0; i < wayNodeRefs.size() - 1; i++) {
                                long a = wayNodeRefs.get(i);
                                long b = wayNodeRefs.get(i + 1);
                                Node na = nodes.get(a);
                                Node nb = nodes.get(b);
                                if (na == null || nb == null) continue;
                                double w = GeoUtils.haversineMeters(na.lat, na.lon, nb.lat, nb.lon);
                                if (!wayOnewayReverse) {
                                    graph.computeIfAbsent(a, k -> new ArrayList<>()).add(new Edge(a, b, w));
                                }
                                if (!wayOnewayForward) {
                                    graph.computeIfAbsent(b, k -> new ArrayList<>()).add(new Edge(b, a, w));
                                }
                            }
                        }
                        inWay = false;
                    }
                }
            };

            try (FileInputStream fis = new FileInputStream(osmFile)) {
                saxParser.parse(fis, handler);
            }
        } catch (Exception e) {
            throw new RuntimeException("Failed to parse OSM: " + e.getMessage(), e);
        }

        // Ensure all vertices exist in adjacency map
        for (Long id : nodes.keySet()) {
            graph.computeIfAbsent(id, k -> new ArrayList<>());
        }

        return new Graph(nodes, graph);
    }
}


