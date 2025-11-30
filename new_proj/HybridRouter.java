package com.osmrouter;

import java.util.*;

/**
 * HybridRouter - single-file hybrid routing implementation:
 *  - ALT (A* + Landmarks) as primary fast algorithm (preprocessing step)
 *  - Plain A* for medium-range queries
 *  - Dijkstra for short/local queries
 *
 * How to use:
 *   HybridRouter router = new HybridRouter(graph.adjacency, graph.nodeIdToNode);
 *   List<Long> path = router.shortestPath(srcId, dstId);
 *
 * Configurable behavior:
 *   - numberOfLandmarks (default 6)
 *   - altUseDistanceMetersThreshold: if straight-line distance > this, use ALT
 *   - aStarUseDistanceMetersThreshold: if straight-line distance > this and <= alt threshold use A*
 *   - else use Dijkstra
 *
 * Notes:
 *  - Preprocessing (landmark selection and one-to-all Dijkstra from each landmark) happens
 *    the first time you call ensurePreprocessed() (or during construction if you call preprocess()).
 *  - Landmark distances are stored as arrays (index = landmark index). Memory ~ (numLandmarks * numNodes).
 *  - Heuristic for ALT = max over landmarks of | dist(landmark->target) - dist(landmark->node) | (admissible).
 */
public class HybridRouter {
    private final Map<Long, List<OSMGraphBuilder.Edge>> adjacency;
    private final Map<Long, OSMGraphBuilder.Node> nodes;
    // landmark data
    private final int numberOfLandmarks;
    private List<Long> landmarks = new ArrayList<>();
    // distancesFromLandmark.get(i) -> Map nodeId -> dist (meters) from landmark i to node
    private List<Map<Long, Double>> distancesFromLandmark = new ArrayList<>();
    private boolean preprocessed = false;

    // thresholds (meters) - tune these to your dataset / needs
    private double altUseDistanceMetersThreshold = 5000.0;    // > this => use ALT
    private double aStarUseDistanceMetersThreshold = 1000.0; // between this and ALT threshold => A*
                                                            // <= this => Dijkstra

    public HybridRouter(Map<Long, List<OSMGraphBuilder.Edge>> adjacency,
                        Map<Long, OSMGraphBuilder.Node> nodes) {
        this(adjacency, nodes, 6);
    }

    /**
     * @param numberOfLandmarks number of landmarks to build for ALT. 4-12 is typical.
     */
    public HybridRouter(Map<Long, List<OSMGraphBuilder.Edge>> adjacency,
                        Map<Long, OSMGraphBuilder.Node> nodes,
                        int numberOfLandmarks) {
        this.adjacency = adjacency;
        this.nodes = nodes;
        this.numberOfLandmarks = Math.max(1, numberOfLandmarks);
    }

    /**
     * Shortcut: call this to force preprocessing immediately (landmarks + distances).
     * This runs one-to-all Dijkstra from each landmark (can be slow on very large graphs but
     * is required for ALT heuristics).
     */
    public synchronized void preprocess() {
        if (preprocessed) return;
        chooseLandmarks();
        buildDistancesFromLandmarks();
        preprocessed = true;
        System.out.println("HybridRouter: Preprocessed " + landmarks.size() + " landmarks.");
    }

    /**
     * Main public routing API.
     */
    public List<Long> shortestPath(long sourceId, long targetId) {
        if (sourceId == targetId) return List.of(sourceId);
        if (!adjacency.containsKey(sourceId) || !adjacency.containsKey(targetId)) return List.of();

        // Ensure preprocessing is available lazily
        if (!preprocessed) preprocess();

        // Straight-line distance (meters)
        OSMGraphBuilder.Node s = nodes.get(sourceId);
        OSMGraphBuilder.Node t = nodes.get(targetId);
        if (s == null || t == null) return List.of();
        double straight = GeoUtils.haversineMeters(s.lat, s.lon, t.lat, t.lon);

        if (straight > altUseDistanceMetersThreshold) {
            // use ALT (A* with landmark heuristic)
            return altAStar(sourceId, targetId);
        } else if (straight > aStarUseDistanceMetersThreshold) {
            // use plain A*
            return aStar(sourceId, targetId);
        } else {
            // short/local: Dijkstra
            return dijkstra(sourceId, targetId);
        }
    }

    // -------------------------
    // Dijkstra (single-source) - adapted to project conventions
    // -------------------------
    private List<Long> dijkstra(long sourceId, long targetId) {
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

    // -------------------------
    // Plain A* (uses haversine as heuristic)
    // -------------------------
    private List<Long> aStar(long sourceId, long targetId) {
        if (sourceId == targetId) return List.of(sourceId);

        Map<Long, Double> gScore = new HashMap<>();
        Map<Long, Double> fScore = new HashMap<>();
        Map<Long, Long> cameFrom = new HashMap<>();

        for (Long id : adjacency.keySet()) {
            gScore.put(id, Double.POSITIVE_INFINITY);
            fScore.put(id, Double.POSITIVE_INFINITY);
        }
        gScore.put(sourceId, 0.0);
        fScore.put(sourceId, heuristicHaversine(sourceId, targetId));

        class E { long id; double f; E(long id, double f){this.id=id;this.f=f;} }
        PriorityQueue<E> open = new PriorityQueue<>(Comparator.comparingDouble(e -> e.f));
        open.add(new E(sourceId, fScore.get(sourceId)));
        Set<Long> closed = new HashSet<>();

        while (!open.isEmpty()) {
            E cur = open.poll();
            long u = cur.id;
            if (closed.contains(u)) continue;
            if (u == targetId) break;
            closed.add(u);

            List<OSMGraphBuilder.Edge> edges = adjacency.get(u);
            if (edges == null) continue;
            double gu = gScore.getOrDefault(u, Double.POSITIVE_INFINITY);
            for (OSMGraphBuilder.Edge e : edges) {
                long v = e.to;
                double tentativeG = gu + e.weightMeters;
                if (tentativeG < gScore.getOrDefault(v, Double.POSITIVE_INFINITY)) {
                    cameFrom.put(v, u);
                    gScore.put(v, tentativeG);
                    double fv = tentativeG + heuristicHaversine(v, targetId);
                    fScore.put(v, fv);
                    open.add(new E(v, fv));
                }
            }
        }

        if (!cameFrom.containsKey(targetId) && sourceId != targetId) return List.of();

        LinkedList<Long> path = new LinkedList<>();
        Long cur = targetId;
        path.addFirst(cur);
        while (!Objects.equals(cur, sourceId)) {
            cur = cameFrom.get(cur);
            if (cur == null) return List.of();
            path.addFirst(cur);
        }
        return path;
    }

    private double heuristicHaversine(long aId, long bId) {
        OSMGraphBuilder.Node a = nodes.get(aId);
        OSMGraphBuilder.Node b = nodes.get(bId);
        if (a == null || b == null) return 0.0;
        return GeoUtils.haversineMeters(a.lat, a.lon, b.lat, b.lon);
    }

    // -------------------------
    // ALT (A* with landmark heuristic)
    // -------------------------
    private List<Long> altAStar(long sourceId, long targetId) {
        if (!preprocessed) preprocess(); // ensure landmarks/distances present

        // f = g + h_alt, where h_alt(node) = max_i | dist(landmark_i -> target) - dist(landmark_i -> node) |
        Map<Long, Double> gScore = new HashMap<>();
        Map<Long, Double> fScore = new HashMap<>();
        Map<Long, Long> cameFrom = new HashMap<>();

        for (Long id : adjacency.keySet()) {
            gScore.put(id, Double.POSITIVE_INFINITY);
            fScore.put(id, Double.POSITIVE_INFINITY);
        }
        gScore.put(sourceId, 0.0);
        fScore.put(sourceId, altHeuristic(sourceId, targetId));

        class E { long id; double f; E(long id, double f){this.id=id;this.f=f;} }
        PriorityQueue<E> open = new PriorityQueue<>(Comparator.comparingDouble(e -> e.f));
        open.add(new E(sourceId, fScore.get(sourceId)));
        Set<Long> closed = new HashSet<>();

        while (!open.isEmpty()) {
            E cur = open.poll();
            long u = cur.id;
            if (closed.contains(u)) continue;
            if (u == targetId) break;
            closed.add(u);

            List<OSMGraphBuilder.Edge> edges = adjacency.get(u);
            if (edges == null) continue;
            double gu = gScore.getOrDefault(u, Double.POSITIVE_INFINITY);
            for (OSMGraphBuilder.Edge e : edges) {
                long v = e.to;
                double tentativeG = gu + e.weightMeters;
                if (tentativeG < gScore.getOrDefault(v, Double.POSITIVE_INFINITY)) {
                    cameFrom.put(v, u);
                    gScore.put(v, tentativeG);
                    double fv = tentativeG + altHeuristic(v, targetId);
                    fScore.put(v, fv);
                    open.add(new E(v, fv));
                }
            }
        }

        if (!cameFrom.containsKey(targetId) && sourceId != targetId) return List.of();

        LinkedList<Long> path = new LinkedList<>();
        Long cur = targetId;
        path.addFirst(cur);
        while (!Objects.equals(cur, sourceId)) {
            cur = cameFrom.get(cur);
            if (cur == null) return List.of();
            path.addFirst(cur);
        }
        return path;
    }

    /**
     * ALT heuristic: max over landmarks of |dist(l -> target) - dist(l -> node)|
     * We stored distances as "landmark -> node" (one-to-all).
     */
    private double altHeuristic(long nodeId, long targetId) {
        double best = 0.0;
        for (int i = 0; i < landmarks.size(); i++) {
            Map<Long, Double> distMap = distancesFromLandmark.get(i);
            Double dToNode = distMap.get(nodeId);
            Double dToTarget = distMap.get(targetId);
            if (dToNode == null || dToTarget == null) continue;
            double estimate = Math.abs(dToTarget - dToNode);
            if (estimate > best) best = estimate;
        }
        // fall back to straight-line if all else fails
        if (best == 0.0) {
            return heuristicHaversine(nodeId, targetId);
        }
        return best;
    }

    // -------------------------
    // Landmark selection & distances
    // -------------------------
    private void chooseLandmarks() {
        // Simple farthest-point heuristic:
        // 1) pick an arbitrary node as first
        // 2) repeatedly pick node farthest (in graph distance sense approximated by haversine) from previous picked set
        if (nodes.isEmpty()) return;
        List<Long> allNodes = new ArrayList<>(nodes.keySet());
        Random rnd = new Random(12345);
        long first = allNodes.get(rnd.nextInt(allNodes.size()));
        landmarks.add(first);

        for (int i = 1; i < numberOfLandmarks; i++) {
            long best = -1L;
            double bestMin = -1.0;
            for (Long cand : allNodes) {
                double minDist = Double.POSITIVE_INFINITY;
                OSMGraphBuilder.Node cn = nodes.get(cand);
                if (cn == null) continue;
                for (Long lm : landmarks) {
                    OSMGraphBuilder.Node ln = nodes.get(lm);
                    if (ln == null) continue;
                    double d = GeoUtils.haversineMeters(cn.lat, cn.lon, ln.lat, ln.lon);
                    if (d < minDist) minDist = d;
                }
                if (minDist > bestMin) {
                    bestMin = minDist;
                    best = cand;
                }
            }
            if (best == -1L) break;
            landmarks.add(best);
        }
    }

    private void buildDistancesFromLandmarks() {
        distancesFromLandmark.clear();
        for (int i = 0; i < landmarks.size(); i++) {
            long lm = landmarks.get(i);
            Map<Long, Double> dist = singleSourceDijkstraOneToAll(lm);
            distancesFromLandmark.add(dist);
        }
    }

    /**
     * Run Dijkstra from a source to compute distances to all nodes.
     * This is used once per-landmark during preprocessing.
     */
    private Map<Long, Double> singleSourceDijkstraOneToAll(long sourceId) {
        Map<Long, Double> dist = new HashMap<>();
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
            List<OSMGraphBuilder.Edge> edges = adjacency.get(u);
            if (edges == null) continue;
            for (OSMGraphBuilder.Edge e : edges) {
                double alt = du + e.weightMeters;
                if (alt < dist.getOrDefault(e.to, Double.POSITIVE_INFINITY)) {
                    dist.put(e.to, alt);
                    pq.add(new long[] { e.to, Double.doubleToLongBits(alt) });
                }
            }
        }
        return dist;
    }

    // -------------------------
    // Configuration setters
    // -------------------------
    public void setAltUseDistanceMetersThreshold(double meters) {
        this.altUseDistanceMetersThreshold = meters;
    }

    public void setAStarUseDistanceMetersThreshold(double meters) {
        this.aStarUseDistanceMetersThreshold = meters;
    }

    /**
     * If you want to rebuild landmarks (e.g. after loading different graph), call this and then preprocess().
     */
    public synchronized void resetLandmarks() {
        this.landmarks = new ArrayList<>();
        this.distancesFromLandmark = new ArrayList<>();
        this.preprocessed = false;
    }
}

