package com.osmrouter;

import java.util.*;

public class DijkstraRouter {
    private final Map<Long, List<OSMGraphBuilder.Edge>> adjacency;
    private final Map<Long, OSMGraphBuilder.Node> nodes;

    public DijkstraRouter(Map<Long, List<OSMGraphBuilder.Edge>> adjacency, Map<Long, OSMGraphBuilder.Node> nodes) {
        this.adjacency = adjacency;
        this.nodes = nodes;
    }

    public List<Long> shortestPath(long sourceId, long targetId) {
        if (sourceId == targetId) return List.of(sourceId);
        if (!adjacency.containsKey(sourceId) || !adjacency.containsKey(targetId)) return List.of();

        Map<Long, Double> dist = new HashMap<>();
        Map<Long, Long> prev = new HashMap<>();
        for (Long id : adjacency.keySet()) dist.put(id, Double.POSITIVE_INFINITY);
        dist.put(sourceId, 0.0);

        PriorityQueue<long[]> pq = new PriorityQueue<>(Comparator.comparingDouble(a -> Double.longBitsToDouble(a[1])));
        pq.add(new long[] { sourceId, Double.doubleToLongBits(0.0) });

        Set<Long> visited = new HashSet<>();

        while (!pq.isEmpty()) {
            long[] cur = pq.poll();
            long u = cur[0];
            double du = Double.longBitsToDouble(cur[1]);
            if (visited.contains(u)) continue;
            visited.add(u);
            if (u == targetId) break;
            List<OSMGraphBuilder.Edge> edges = adjacency.get(u);
            if (edges == null) continue;
            for (OSMGraphBuilder.Edge e : edges) {
                double alt = du + e.weightMeters;
                if (alt < dist.getOrDefault(e.to, Double.POSITIVE_INFINITY)) {
                    dist.put(e.to, alt);
                    prev.put(e.to, u);
                    pq.add(new long[] { e.to, Double.doubleToLongBits(alt) });
                }
            }
        }

        if (!prev.containsKey(targetId) && sourceId != targetId) return List.of();

        LinkedList<Long> path = new LinkedList<>();
        Long cur = targetId;
        path.addFirst(cur);
        while (!Objects.equals(cur, sourceId)) {
            cur = prev.get(cur);
            if (cur == null) return List.of();
            path.addFirst(cur);
        }
        return path;
    }
}


