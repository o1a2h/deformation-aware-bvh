// BVHBenchmark.cs — Place in Assets/Scripts/
using UnityEngine;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Globalization;
using System.IO;
using System.Text;
using Debug = UnityEngine.Debug;

[RequireComponent(typeof(MeshFilter), typeof(MeshRenderer))]
public class BVHBenchmark : MonoBehaviour
{
    public enum DeformationType { None, SineWave, Twist, Localized, Pulse, SquashStretch }
    public enum UpdateMethod { AwareBVH, KineticBVH, RefitRotate, DynamicBVH }

    [Header("Mesh")]
    public string meshName = "noname";

    [Header("Deformation")]
    public DeformationType deformation = DeformationType.Localized;

    [Header("Localized Deformation Settings")]
    public Vector3 localizedCenter = new Vector3(0.5f, 0.75f, 0.5f);
    [Range(0.05f, 0.5f)]
    public float localizedRadius = 0.15f;

    [Header("Movement")]
    public float moveSpeed = 2f;
    public float moveDistance = 3f;

    [Header("BVH")]
    public UpdateMethod method = UpdateMethod.AwareBVH;
    public int maxTrisPerLeaf = 10;
    public bool useMultithreading = false;

    [Header("Benchmark")]
    public int benchmarkRunCount = 10;

    [Header("Debug — Hierarchy")]
    [Range(0, 15)] public int bvhLevel = 4;
    public bool cumulativeLevels = false;
    [Range(0f, 0.1f)] public float boxThickness = 0.01f;

    [Header("Debug — Vertices")]
    public bool showExtremeVertices = false;
    public bool showCollidingVertices = true;
    [Range(0.001f, 0.1f)] public float vertexSize = 0.01f;

    [Header("Obstacle")]
    public Collider obstacle;

    // ---- Per-frame timing ----
    [NonSerialized] public BVHTree tree;
    [NonSerialized] public UpdateStats lastUpdateStats;
    [NonSerialized] public double lastDeformMs;
    [NonSerialized] public double lastQueryMs;
    [NonSerialized] public double lastTotalMs;
    [NonSerialized] public int queryNodesVisited;
    [NonSerialized] public HashSet<int> collidedVertices = new HashSet<int>();

    // ---- Internal mesh state ----
    private MeshFilter meshFilter;
    private Mesh workingMesh;
    private Vector3[] originalVerts;
    private Vector3[] currentVerts;
    private Vector3[] previousVerts;
    private Vector3[] originalNormals;
    private int[] tris;
    private Vector3 startPos;

    private Vector3 meshBoundsMin, meshBoundsMax, meshDiag;
    private float meshDiagLen;

    // ---- Mode flags ----
    [NonSerialized] public bool isPreviewing;
    [NonSerialized] public bool isMoving;
    [NonSerialized] public bool isBenchmarking;
    [NonSerialized] public bool isBenchmarkingAll;
    private float timer;

    // ---- Logging ----
    private bool isLogging;
    private StringBuilder csvBuilder;
    private int frameIndex;
    private string pendingSavePath;

    // ---- Benchmark state ----
    [NonSerialized] public int benchCurrentRun;
    private int benchTotalRuns;
    private Queue<UpdateMethod> benchAllQueue;
    [NonSerialized] public string benchAllStatus = "";

    private readonly Stopwatch sw = new Stopwatch();

    // ================================================================
    void Awake()
    {
        meshFilter = GetComponent<MeshFilter>();
        var src = meshFilter.sharedMesh;
        workingMesh = Instantiate(src);
        meshFilter.mesh = workingMesh;

        originalVerts = src.vertices;
        originalNormals = src.normals;
        if (originalNormals == null || originalNormals.Length != originalVerts.Length)
            originalNormals = new Vector3[originalVerts.Length];
        currentVerts = (Vector3[])originalVerts.Clone();
        previousVerts = (Vector3[])originalVerts.Clone();
        tris = workingMesh.triangles;
        startPos = transform.position;

        meshBoundsMin = Vector3.positiveInfinity;
        meshBoundsMax = Vector3.negativeInfinity;
        for (int i = 0; i < originalVerts.Length; i++)
        {
            meshBoundsMin = Vector3.Min(meshBoundsMin, originalVerts[i]);
            meshBoundsMax = Vector3.Max(meshBoundsMax, originalVerts[i]);
        }
        meshDiag = meshBoundsMax - meshBoundsMin;
        meshDiagLen = meshDiag.magnitude;
    }

    void Start() => RebuildBVH();

    void Update()
    {
        if (isPreviewing)
        {
            timer += Time.deltaTime;
            RunFrame(timer);
        }
        else if (isMoving || isBenchmarking || isBenchmarkingAll)
        {
            timer += Time.deltaTime;
            bool stillMoving = ApplyMovement();
            if (stillMoving)
                RunFrame(timer);
            else if (isBenchmarkingAll || isBenchmarking)
                OnBenchmarkRunFinished();
            else
                OnMoveFinished();
        }
        else
        {
            ResetToIdle();
        }
    }

    // ================================================================
    //  CORE FRAME — different path for Aware vs others
    // ================================================================
    private void RunFrame(float t)
    {
        Stopwatch swTotal = Stopwatch.StartNew();

        if (method == UpdateMethod.AwareBVH && tree != null)
        {
            if (useMultithreading)
            {
                // ---- AWARE PARALLEL: deform first, then parallel inline check ----
                sw.Restart();
                Array.Copy(currentVerts, previousVerts, currentVerts.Length);
                ApplyDeformation(t); // deform without inline check
                workingMesh.vertices = currentVerts;
                workingMesh.RecalculateNormals();
                sw.Stop();
                lastDeformMs = sw.Elapsed.TotalMilliseconds;

                // Phase 2: Parallel inline check + sequential recompute
                sw.Restart();
                lastUpdateStats = ParallelAwareUpdater.Update(
                    tree, currentVerts, previousVerts, tris);
                sw.Stop();
                lastUpdateStats.updateTimeMs = sw.Elapsed.TotalMilliseconds;
            }
            else
            {
                // ---- AWARE SINGLE-THREADED: integrated deformation + BVH check ----
                sw.Restart();
                Array.Copy(currentVerts, previousVerts, currentVerts.Length);
                lastUpdateStats = default;
                AwareUpdater.BeginFrame(tree);
                ApplyDeformationWithAwareCheck(t);
                workingMesh.vertices = currentVerts;
                workingMesh.RecalculateNormals();
                sw.Stop();
                lastDeformMs = sw.Elapsed.TotalMilliseconds;

                // Phase 2: Bottom-up dirty recompute only
                sw.Restart();
                AwareUpdater.RecomputeDirty(tree, currentVerts, tris, ref lastUpdateStats);
                sw.Stop();
                lastUpdateStats.updateTimeMs = sw.Elapsed.TotalMilliseconds;
            }
        }
        else
        {
            // ---- OTHER METHODS: separate deformation then BVH update ----
            // Phase 1: Pure deformation
            sw.Restart();
            Array.Copy(currentVerts, previousVerts, currentVerts.Length);
            ApplyDeformation(t);
            workingMesh.vertices = currentVerts;
            workingMesh.RecalculateNormals();
            sw.Stop();
            lastDeformMs = sw.Elapsed.TotalMilliseconds;

            // Phase 2: BVH update (single-threaded or parallel)
            if (tree != null)
            {
                if (useMultithreading)
                {
                    switch (method)
                    {
                        case UpdateMethod.KineticBVH:
                            lastUpdateStats = ParallelKineticUpdater.Update(
                                tree, previousVerts, currentVerts, tris, Time.deltaTime);
                            break;
                        case UpdateMethod.RefitRotate:
                            lastUpdateStats = ParallelRefitRotateUpdater.Update(
                                tree, currentVerts, tris);
                            break;
                        case UpdateMethod.DynamicBVH:
                            lastUpdateStats = ParallelDynamicBVHUpdater.Update(
                                tree, currentVerts, tris);
                            break;
                    }
                }
                else
                {
                    switch (method)
                    {
                        case UpdateMethod.KineticBVH:
                            lastUpdateStats = KineticUpdater.Update(
                                tree, previousVerts, currentVerts, tris, Time.deltaTime);
                            break;
                        case UpdateMethod.RefitRotate:
                            lastUpdateStats = RefitRotateUpdater.Update(tree, currentVerts, tris);
                            break;
                        case UpdateMethod.DynamicBVH:
                            lastUpdateStats = DynamicBVHUpdater.Update(tree, currentVerts, tris);
                            break;
                    }
                }
            }
        }

        // Phase 3: Collision Query (same for all)
        sw.Restart();
        collidedVertices.Clear();
        queryNodesVisited = 0;
        if (obstacle != null && tree != null)
            QueryNode(0, obstacle.bounds);
        sw.Stop();
        lastQueryMs = sw.Elapsed.TotalMilliseconds;

        swTotal.Stop();
        lastTotalMs = swTotal.Elapsed.TotalMilliseconds;

        if (isLogging) LogFrame();
    }

    // ================================================================
    //  INTEGRATED DEFORMATION + AWARE BVH CHECK
    //  This is the paper's core idea: the BVH update piggybacks on
    //  the deformation loop. Each vertex is checked against its leaf
    //  bounds immediately after its new position is computed.
    // ================================================================
    private void ApplyDeformationWithAwareCheck(float t)
    {
        for (int i = 0; i < originalVerts.Length; i++)
        {
            Vector3 v = originalVerts[i];
            Vector3 newPos;

            switch (deformation)
            {
                case DeformationType.None:
                    newPos = v;
                    break;

                case DeformationType.SineWave:
                    newPos = v + Vector3.one * (Mathf.Sin(t * 5f + v.y) * 0.1f);
                    break;

                case DeformationType.Twist:
                    {
                        float angle = v.y * Mathf.Sin(t * 3f) * 2f;
                        float cos = Mathf.Cos(angle), sin = Mathf.Sin(angle);
                        newPos = new Vector3(v.x * cos - v.z * sin, v.y, v.x * sin + v.z * cos);
                        break;
                    }

                case DeformationType.Localized:
                    {
                        Vector3 center = meshBoundsMin + Vector3.Scale(localizedCenter, meshDiag);
                        float radius = localizedRadius * meshDiagLen;
                        float dist = Vector3.Distance(v, center);
                        if (dist < radius)
                        {
                            float falloff = 1f - (dist / radius);
                            falloff *= falloff;
                            float disp = Mathf.Sin(t * 5f + v.y * 3f) * 0.15f * falloff;
                            newPos = v + Vector3.one * disp;
                        }
                        else
                        {
                            newPos = v;
                        }
                        break;
                    }

                case DeformationType.Pulse:
                    {
                        Vector3 n = (i < originalNormals.Length) ? originalNormals[i] : Vector3.up;
                        newPos = v + n * (Mathf.Sin(t * 8f) * 0.15f);
                        break;
                    }

                case DeformationType.SquashStretch:
                    {
                        float s = Mathf.Sin(t * 4f) * 0.5f;
                        newPos = new Vector3(v.x * (1f - s * 0.5f), v.y * (1f + s), v.z * (1f - s * 0.5f));
                        break;
                    }

                default:
                    newPos = v;
                    break;
            }

            currentVerts[i] = newPos;

            // Inline BVH check — O(1) for non-escaping vertices
            AwareUpdater.CheckVertex(tree, i, newPos, previousVerts, ref lastUpdateStats);
        }
    }

    // ================================================================
    //  PURE DEFORMATION (used by Kinetic, RefitRotate, Dynamic)
    // ================================================================
    private void ApplyDeformation(float t)
    {
        for (int i = 0; i < originalVerts.Length; i++)
        {
            Vector3 v = originalVerts[i];
            switch (deformation)
            {
                case DeformationType.None:
                    currentVerts[i] = v;
                    break;

                case DeformationType.SineWave:
                    currentVerts[i] = v + Vector3.one * (Mathf.Sin(t * 5f + v.y) * 0.1f);
                    break;

                case DeformationType.Twist:
                    {
                        float angle = v.y * Mathf.Sin(t * 3f) * 2f;
                        float cos = Mathf.Cos(angle), sin = Mathf.Sin(angle);
                        currentVerts[i] = new Vector3(v.x * cos - v.z * sin, v.y, v.x * sin + v.z * cos);
                        break;
                    }

                case DeformationType.Localized:
                    {
                        Vector3 center = meshBoundsMin + Vector3.Scale(localizedCenter, meshDiag);
                        float radius = localizedRadius * meshDiagLen;
                        float dist = Vector3.Distance(v, center);
                        if (dist < radius)
                        {
                            float falloff = 1f - (dist / radius);
                            falloff *= falloff;
                            float disp = Mathf.Sin(t * 5f + v.y * 3f) * 0.15f * falloff;
                            currentVerts[i] = v + Vector3.one * disp;
                        }
                        else
                        {
                            currentVerts[i] = v;
                        }
                        break;
                    }

                case DeformationType.Pulse:
                    {
                        Vector3 n = (i < originalNormals.Length) ? originalNormals[i] : Vector3.up;
                        currentVerts[i] = v + n * (Mathf.Sin(t * 8f) * 0.15f);
                        break;
                    }

                case DeformationType.SquashStretch:
                    {
                        float s = Mathf.Sin(t * 4f) * 0.5f;
                        currentVerts[i] = new Vector3(v.x * (1f - s * 0.5f), v.y * (1f + s), v.z * (1f - s * 0.5f));
                        break;
                    }
            }
        }
    }

    // ================================================================
    //  MOVEMENT
    // ================================================================
    private bool ApplyMovement()
    {
        float totalDuration = (moveDistance * 4f) / Mathf.Max(moveSpeed, 0.01f);
        float p = Mathf.Clamp01(timer / totalDuration);
        if (p >= 1f) { transform.position = startPos; return false; }

        float offset;
        if (p < 0.25f) offset = Mathf.Lerp(0, moveDistance, p / 0.25f);
        else if (p < 0.75f) offset = Mathf.Lerp(moveDistance, -moveDistance, (p - 0.25f) / 0.5f);
        else offset = Mathf.Lerp(-moveDistance, 0, (p - 0.75f) / 0.25f);

        transform.position = startPos + new Vector3(offset, 0f, 0f);
        return true;
    }

    // ================================================================
    //  COLLISION QUERY
    // ================================================================
    private void QueryNode(int idx, Bounds obsWorld)
    {
        if (idx < 0 || idx >= tree.nodeCount) return;
        queryNodesVisited++;

        ref var nd = ref tree.nodes[idx];
        Vector3 wCenter = transform.TransformPoint(nd.bounds.center);
        Vector3 wSize = Vector3.Scale(nd.bounds.size, transform.lossyScale);
        Bounds nodeWorld = new Bounds(wCenter, wSize);

        if (!obsWorld.Intersects(nodeWorld)) return;

        if (tree.IsLeaf(idx))
        {
            int start = nd.triStart, count = nd.triCount;
            for (int t = start; t < start + count; t++)
            {
                int triBase = tree.triIndices[t];
                for (int i = 0; i < 3; i++)
                {
                    int v = tris[triBase + i];
                    if (obsWorld.Contains(transform.TransformPoint(currentVerts[v])))
                        collidedVertices.Add(v);
                }
            }
        }
        else
        {
            QueryNode(nd.left, obsWorld);
            QueryNode(nd.right, obsWorld);
        }
    }

    // ================================================================
    //  CSV LOGGING
    // ================================================================
    private string CsvHeader()
    {
        int triCount = tris.Length / 3;
        return
            $"# method={method},deformation={deformation}," +
            $"triangles={triCount},vertices={originalVerts.Length}," +
            $"treeNodes={tree?.nodeCount ?? 0},maxTrisPerLeaf={maxTrisPerLeaf}," +
            $"threading={(useMultithreading ? "parallel" : "single")}," +
            $"moveSpeed={F(moveSpeed)},moveDistance={F(moveDistance)}\n" +
            "run,frame,cycleProgress," +
            "deformMs,updateMs,queryMs,totalMs," +
            "verticesChecked,nodesVisitedUpdate,dirtyNodes," +
            "nodesVisitedQuery,collidedVertices";
    }

    private void LogFrame()
    {
        float totalDuration = (moveDistance * 4f) / Mathf.Max(moveSpeed, 0.01f);
        float progress = Mathf.Clamp01(timer / totalDuration);
        int run = (isBenchmarking || isBenchmarkingAll) ? benchCurrentRun : 1;
        var s = lastUpdateStats;

        csvBuilder.AppendLine(
            $"{run},{frameIndex},{F(progress)}," +
            $"{F(lastDeformMs)},{F(s.updateTimeMs)},{F(lastQueryMs)},{F(lastTotalMs)}," +
            $"{s.verticesChecked},{s.nodesVisited},{s.dirtyNodes}," +
            $"{queryNodesVisited},{collidedVertices.Count}");
        frameIndex++;
    }

    private void BeginLogging(string filePath)
    {
        isLogging = true; frameIndex = 0;
        csvBuilder = new StringBuilder();
        csvBuilder.AppendLine(CsvHeader());
        pendingSavePath = filePath;
    }

    private void FlushLog()
    {
        if (!isLogging || csvBuilder == null) return;
        string dir = Path.GetDirectoryName(pendingSavePath);
        if (!string.IsNullOrEmpty(dir) && !Directory.Exists(dir))
            Directory.CreateDirectory(dir);
        File.WriteAllText(pendingSavePath, csvBuilder.ToString());
        Debug.Log($"<color=green>[BVH] Saved → {pendingSavePath}</color>");
        isLogging = false; csvBuilder = null;
    }

    private string MakePath(string prefix, UpdateMethod m)
    {
        string ts = DateTime.Now.ToString("yyyy-MM-dd");
        int triCount = tris.Length / 3;
        string threadTag = useMultithreading ? "-MT" : "-ST";
        return Path.Combine(Application.dataPath, "Benchmark",
        $"{prefix}-{m}-{deformation}-{meshName}-{triCount}{threadTag}-{ts}.csv");
    }

    // ================================================================
    //  PUBLIC API
    // ================================================================
    public void RebuildBVH()
    {
        tree = new BVHTree();
        tree.Build(currentVerts, tris, maxTrisPerLeaf);
        DynamicBVHUpdater.Reset();
        ParallelDynamicBVHUpdater.Reset();
    }

    public void StartPreview()
    {
        isPreviewing = true;
        isMoving = isBenchmarking = isBenchmarkingAll = false;
        timer = 0f; ResetVerts();
    }

    public void StopPreview()
    {
        isPreviewing = false; timer = 0f;
        ResetToIdle(); RebuildBVH();
    }

    public void StartMove()
    {
        isMoving = true;
        isPreviewing = isBenchmarking = isBenchmarkingAll = false;
        timer = 0f; ResetVerts(); RebuildBVH();
        BeginLogging(MakePath("_singletest", method));
    }

    private void OnMoveFinished()
    {
        isMoving = false; timer = 0f;
        FlushLog(); ResetToIdle(); RebuildBVH();
    }

    public void StartBenchmark()
    {
        isBenchmarking = true;
        isPreviewing = isMoving = isBenchmarkingAll = false;
        benchCurrentRun = 1; benchTotalRuns = benchmarkRunCount;
        timer = 0f; ResetVerts(); RebuildBVH();
        BeginLogging(MakePath("benchmark", method));
        Debug.Log($"<color=cyan>[BVH] Benchmark: {benchTotalRuns} runs, {method}</color>");
    }

    public void StartBenchmarkAll()
    {
        isBenchmarkingAll = true;
        isPreviewing = isMoving = isBenchmarking = false;
        benchAllQueue = new Queue<UpdateMethod>();
        benchAllQueue.Enqueue(UpdateMethod.AwareBVH);
        benchAllQueue.Enqueue(UpdateMethod.KineticBVH);
        benchAllQueue.Enqueue(UpdateMethod.RefitRotate);
        benchAllQueue.Enqueue(UpdateMethod.DynamicBVH);
        StartNextBenchmarkAll();
    }

    private void StartNextBenchmarkAll()
    {
        if (benchAllQueue.Count == 0)
        {
            isBenchmarkingAll = false;
            benchAllStatus = "All done!";
            ResetToIdle(); RebuildBVH();
            Debug.Log("<color=green>[BVH] Benchmark All complete!</color>");
            return;
        }
        method = benchAllQueue.Dequeue();
        benchCurrentRun = 1; benchTotalRuns = benchmarkRunCount;
        timer = 0f;
        benchAllStatus = $"{method} ({4 - benchAllQueue.Count}/4)";
        ResetVerts(); RebuildBVH();
        BeginLogging(MakePath("benchall", method));
        Debug.Log($"<color=cyan>[BVH] Bench All: starting {method}</color>");
    }

    private void OnBenchmarkRunFinished()
    {
        Debug.Log($"<color=cyan>[BVH] Run {benchCurrentRun}/{benchTotalRuns} done ({method})</color>");
        if (benchCurrentRun < benchTotalRuns)
        {
            benchCurrentRun++;
            timer = 0f; ResetVerts(); RebuildBVH();
        }
        else
        {
            FlushLog();
            if (isBenchmarkingAll) StartNextBenchmarkAll();
            else
            {
                isBenchmarking = false; timer = 0f;
                ResetToIdle(); RebuildBVH();
                Debug.Log($"<color=green>[BVH] Benchmark complete ({benchTotalRuns} runs)</color>");
            }
        }
    }

    private void ResetVerts()
    {
        Array.Copy(originalVerts, previousVerts, originalVerts.Length);
        Array.Copy(originalVerts, currentVerts, originalVerts.Length);
    }

    private void ResetToIdle()
    {
        Array.Copy(originalVerts, currentVerts, originalVerts.Length);
        workingMesh.vertices = currentVerts;
        workingMesh.RecalculateNormals();
        transform.position = startPos;
    }

    private static string F(double v) => v.ToString("F4", CultureInfo.InvariantCulture);
    private static string F(float v) => v.ToString("F4", CultureInfo.InvariantCulture);

    // ================================================================
    //  GIZMOS
    // ================================================================
    void OnDrawGizmos()
    {
        if (tree == null || tree.nodeCount == 0 || currentVerts == null) return;

        Gizmos.matrix = transform.localToWorldMatrix;
        DrawBVHBoxes(0);
        if (showExtremeVertices) DrawExtremes(0);

        if (showCollidingVertices && collidedVertices.Count > 0)
        {
            Gizmos.matrix = Matrix4x4.identity;
            Gizmos.color = Color.magenta;
            foreach (int v in collidedVertices)
                Gizmos.DrawSphere(transform.TransformPoint(currentVerts[v]), vertexSize * 3f);
        }
    }

    private void DrawBVHBoxes(int idx)
    {
        if (idx < 0 || idx >= tree.nodeCount) return;
        int d = tree.nodes[idx].depth;
        bool draw = cumulativeLevels ? d <= bvhLevel : d == bvhLevel;
        if (draw)
        {
            Gizmos.color = Color.HSVToRGB(1f - (d * 0.14f) % 1f, 0.75f, 1f);
            Bounds b = tree.nodes[idx].bounds;
            if (boxThickness < 0.001f) Gizmos.DrawWireCube(b.center, b.size);
            else DrawThickBox(b.center, b.size);
        }
        if (!tree.IsLeaf(idx))
        {
            DrawBVHBoxes(tree.nodes[idx].left);
            DrawBVHBoxes(tree.nodes[idx].right);
        }
    }

    private void DrawExtremes(int idx)
    {
        if (idx < 0 || idx >= tree.nodeCount) return;
        int d = tree.nodes[idx].depth;
        bool draw = cumulativeLevels ? d <= bvhLevel : d == bvhLevel;
        if (draw)
        {
            Gizmos.color = new Color(1.0f, 0.0f, 0.0f, 1.0f);
            int b = idx * 6;
            for (int a = 0; a < 6; a++)
            {
                int v = tree.extremes[b + a];
                if (v >= 0 && v < currentVerts.Length)
                    Gizmos.DrawSphere(currentVerts[v], vertexSize);
            }
        }
        if (!tree.IsLeaf(idx))
        {
            DrawExtremes(tree.nodes[idx].left);
            DrawExtremes(tree.nodes[idx].right);
        }
    }

    private void DrawThickBox(Vector3 c, Vector3 s)
    {
        Vector3 e = s * 0.5f;
        Span<Vector3> p = stackalloc Vector3[8];
        for (int i = 0; i < 8; i++)
            p[i] = c + new Vector3(
                (i & 1) != 0 ? e.x : -e.x,
                (i & 2) != 0 ? e.y : -e.y,
                (i & 4) != 0 ? e.z : -e.z);
        DrawEdge(p[0], p[1]); DrawEdge(p[2], p[3]); DrawEdge(p[4], p[5]); DrawEdge(p[6], p[7]);
        DrawEdge(p[0], p[2]); DrawEdge(p[1], p[3]); DrawEdge(p[4], p[6]); DrawEdge(p[5], p[7]);
        DrawEdge(p[0], p[4]); DrawEdge(p[1], p[5]); DrawEdge(p[2], p[6]); DrawEdge(p[3], p[7]);
    }

    private void DrawEdge(Vector3 a, Vector3 b)
    {
        Vector3 mid = (a + b) * 0.5f;
        Vector3 dir = b - a;
        if (dir.sqrMagnitude < 0.00001f) return;
        Matrix4x4 old = Gizmos.matrix;
        Gizmos.matrix = old * Matrix4x4.TRS(mid, Quaternion.LookRotation(dir),
                            new Vector3(boxThickness, boxThickness, dir.magnitude));
        Gizmos.DrawCube(Vector3.zero, Vector3.one);
        Gizmos.matrix = old;
    }
}