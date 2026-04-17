// BVHBenchmarkEditor.cs — Place in Assets/Scripts/Editor/
using UnityEngine;
using UnityEditor;

[CustomEditor(typeof(BVHBenchmark))]
public class BVHBenchmarkEditor : Editor
{
    public override void OnInspectorGUI()
    {
        DrawDefaultInspector();

        BVHBenchmark bm = (BVHBenchmark)target;
        bool busy = bm.isPreviewing || bm.isMoving || bm.isBenchmarking || bm.isBenchmarkingAll;

        EditorGUILayout.Space(10);
        EditorGUILayout.LabelField("Controls", EditorStyles.boldLabel);

        // ---- Row 1: Preview + Move ----
        EditorGUILayout.BeginHorizontal();

        GUI.backgroundColor = bm.isPreviewing ? Color.green : Color.white;
        if (GUILayout.Button(bm.isPreviewing ? "■ Stop Preview" : "▶ Preview"))
        {
            if (bm.isPreviewing) bm.StopPreview();
            else bm.StartPreview();
        }
        GUI.backgroundColor = Color.white;

        GUI.enabled = !busy;
        if (GUILayout.Button("▶ Move (log CSV)"))
            bm.StartMove();
        GUI.enabled = true;

        EditorGUILayout.EndHorizontal();

        // ---- Row 2: Benchmark single method ----
        EditorGUILayout.BeginHorizontal();

        GUI.enabled = !busy;
        GUI.backgroundColor = Color.yellow;
        if (GUILayout.Button($"▶ Benchmark ({bm.benchmarkRunCount}x)"))
            bm.StartBenchmark();
        GUI.backgroundColor = Color.white;
        GUI.enabled = true;

        if (bm.isBenchmarking)
        {
            EditorGUILayout.LabelField(
                $"Run {bm.benchCurrentRun}/{bm.benchmarkRunCount}",
                EditorStyles.boldLabel, GUILayout.Width(100));
        }

        EditorGUILayout.EndHorizontal();

        // ---- Row 3: Benchmark All ----
        EditorGUILayout.BeginHorizontal();

        GUI.enabled = !busy;
        GUI.backgroundColor = new Color(1f, 0.5f, 0f); // orange
        if (GUILayout.Button($"▶ Benchmark ALL (4×{bm.benchmarkRunCount} runs)"))
            bm.StartBenchmarkAll();
        GUI.backgroundColor = Color.white;
        GUI.enabled = true;

        if (bm.isBenchmarkingAll)
        {
            EditorGUILayout.LabelField(
                $"{bm.benchAllStatus} run {bm.benchCurrentRun}/{bm.benchmarkRunCount}",
                EditorStyles.boldLabel);
        }

        EditorGUILayout.EndHorizontal();

        // ---- Rebuild ----
        GUI.enabled = !busy;
        if (GUILayout.Button("↻ Rebuild BVH"))
            bm.RebuildBVH();
        GUI.enabled = true;

        // ---- Live Stats ----
        if (bm.tree != null && Application.isPlaying)
        {
            EditorGUILayout.Space(8);
            EditorGUILayout.LabelField("Live Stats", EditorStyles.boldLabel);
            EditorGUILayout.LabelField("Tree nodes", bm.tree.nodeCount.ToString("N0"));
            EditorGUILayout.LabelField("Method", bm.method.ToString());
            EditorGUILayout.LabelField("Deformation", bm.deformation.ToString());
            EditorGUILayout.LabelField("Threading",
                bm.useMultithreading ? "Parallel (Job System)" : "Single-threaded");

            EditorGUILayout.Space(4);
            EditorGUILayout.LabelField("— Timing —", EditorStyles.miniLabel);
            EditorGUILayout.LabelField("  Deform + mesh", $"{bm.lastDeformMs:F2} ms");
            EditorGUILayout.LabelField("  BVH update", $"{bm.lastUpdateStats.updateTimeMs:F2} ms");
            EditorGUILayout.LabelField("  Collision query", $"{bm.lastQueryMs:F2} ms");
            EditorGUILayout.LabelField("  Total", $"{bm.lastTotalMs:F2} ms");

            EditorGUILayout.Space(4);
            EditorGUILayout.LabelField("— Update Phase —", EditorStyles.miniLabel);
            var s = bm.lastUpdateStats;
            EditorGUILayout.LabelField("  Vertices checked", s.verticesChecked.ToString("N0"));
            EditorGUILayout.LabelField("  Nodes visited (update)", s.nodesVisited.ToString("N0"));
            EditorGUILayout.LabelField("  Dirty nodes", s.dirtyNodes.ToString("N0"));

            EditorGUILayout.Space(4);
            EditorGUILayout.LabelField("— Query Phase —", EditorStyles.miniLabel);
            EditorGUILayout.LabelField("  Nodes visited (query)", bm.queryNodesVisited.ToString("N0"));
            EditorGUILayout.LabelField("  Collided vertices", bm.collidedVertices.Count.ToString("N0"));
        }

        if (Application.isPlaying) Repaint();
    }
}