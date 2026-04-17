// GlobalBenchmark.cs , Place in Assets/Scripts/
using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Reflection;
using UnityEngine;
#if UNITY_EDITOR
using UnityEditor;
#endif

public class GlobalBenchmark : MonoBehaviour
{
    [Flags]
    public enum DeformationMask
    {
        None = 1 << 0,
        SineWave = 1 << 1,
        Twist = 1 << 2,
        Localized = 1 << 3,
        Pulse = 1 << 4,
        SquashStretch = 1 << 5
    }

    [Header("Benchmark Matrix Settings")]
    public DeformationMask testDeformations = DeformationMask.None | DeformationMask.Localized | DeformationMask.SineWave | DeformationMask.Twist;
    public string maxTrianglesPerLeaf = "2, 32, 512, 8192";
    public int benchmarkRuns = 10;

    [Header("Threading Modes")]
    public bool testSingleThread = true;
    public bool testMultithread = true;

    [Header("Output")]
    public string saveDirectory = "D:/o1a2h/Programming/Results/";

    [Header("State (Read Only)")]
    public bool isRunning = false;
    public string currentStatus = "Idle";

    private readonly string[] targetMeshes = {
        "Dragon-2-13614.fbx", "Dragon-3-54456.fbx", "Dragon-4-217826.fbx",
        "Plane-2-16896.fbx", "Plane-3-66560.fbx", "Plane-4-264192.fbx",
        "Suzanne-2-15744.fbx", "Suzanne-3-62976.fbx", "Suzanne-4-251904.fbx"
        //"Dragon-4-217826.fbx"
    };

    public void StartGlobalBenchmark()
    {
        if (isRunning)
        {
            Debug.LogWarning("[GlobalBenchmark] Benchmark is already running!");
            return;
        }

        if (!Application.isPlaying)
        {
            Debug.LogError("[GlobalBenchmark] You must be in Play Mode to run the benchmark.");
            return;
        }

        if (!testSingleThread && !testMultithread)
        {
            Debug.LogError("[GlobalBenchmark] You must select at least one threading mode.");
            return;
        }

        StartCoroutine(RunMatrixRoutine());
    }

    private IEnumerator RunMatrixRoutine()
    {
        isRunning = true;
        int totalCombinations = 0;
        int currentCombination = 0;

        // Parse configurations
        List<BVHBenchmark.DeformationType> deformations = GetSelectedDeformations();
        List<int> maxTris = maxTrianglesPerLeaf.Split(',').Select(s => int.Parse(s.Trim())).ToList();
        BVHBenchmark.UpdateMethod[] methods = (BVHBenchmark.UpdateMethod[])Enum.GetValues(typeof(BVHBenchmark.UpdateMethod));

        List<bool> threadingModesList = new List<bool>();
        if (testSingleThread) threadingModesList.Add(false);
        if (testMultithread) threadingModesList.Add(true);
        bool[] threadingModes = threadingModesList.ToArray();

        totalCombinations = targetMeshes.Length * deformations.Count * methods.Length * threadingModes.Length * maxTris.Count;

        if (!Directory.Exists(saveDirectory))
            Directory.CreateDirectory(saveDirectory);

        Collider wallCollider = GameObject.Find("Wall")?.GetComponent<Collider>();
        if (wallCollider == null)
            Debug.LogWarning("[GlobalBenchmark] Could not find a GameObject named 'Wall' with a Collider.");

        // Reflection to inject custom path into BVHBenchmark
        FieldInfo pathField = typeof(BVHBenchmark).GetField("pendingSavePath", BindingFlags.NonPublic | BindingFlags.Instance);

        foreach (string fbxName in targetMeshes)
        {
            currentStatus = $"Loading {fbxName} ...";

#if UNITY_EDITOR
            string assetPath = $"Assets/Mesh/{fbxName}";
            GameObject prefab = AssetDatabase.LoadAssetAtPath<GameObject>(assetPath);

            if (prefab == null)
            {
                Debug.LogError($"[GlobalBenchmark] Could not find mesh at {assetPath}. Skipping.");
                continue;
            }

            // Apply the requested rotation here
            GameObject instance = Instantiate(prefab, Vector3.zero, Quaternion.Euler(0, -90, 0));

            // FBX imports usually have meshes on child objects
            MeshFilter mf = instance.GetComponentInChildren<MeshFilter>();
            if (mf == null)
            {
                Debug.LogError($"[GlobalBenchmark] No MeshFilter found in {fbxName}. Skipping.");
                Destroy(instance);
                continue;
            }

            BVHBenchmark bm = mf.gameObject.AddComponent<BVHBenchmark>();

            // Setup static properties requested
            bm.localizedCenter = new Vector3(0.5f, 0.75f, 0.75f);
            bm.localizedRadius = 0.2f;
            bm.obstacle = wallCollider;
            bm.benchmarkRunCount = benchmarkRuns;

            // Setup new movement parameters
            bm.moveSpeed = 4f;
            bm.moveDistance = 2.5f;

            string meshNameClean = Path.GetFileNameWithoutExtension(fbxName);
            bm.meshName = meshNameClean;

            // Iterate Matrix
            foreach (var def in deformations)
            {
                foreach (var method in methods)
                {
                    foreach (var thread in threadingModes)
                    {
                        foreach (var tri in maxTris)
                        {
                            currentCombination++;
                            currentStatus = $"({currentCombination}/{totalCombinations}) {meshNameClean} | {def} | {method} | {(thread ? "Multi" : "Single")} | {tri} Tris";

                            // Configure benchmark
                            bm.deformation = def;
                            bm.method = method;
                            bm.useMultithreading = thread;
                            bm.maxTrisPerLeaf = tri;

                            // Start
                            bm.StartBenchmark();

                            // Inject custom save path formatted exactly as requested
                            string threadStr = thread ? "multithreading" : "single";
                            string customPath = Path.Combine(saveDirectory, $"{benchmarkRuns} {meshNameClean} {def} {method} {threadStr} {tri}.csv");
                            pathField.SetValue(bm, customPath.Replace("\\", "/"));

                            // Wait for the specific benchmark configuration to finish
                            while (bm.isBenchmarking)
                            {
                                yield return null;
                            }
                        }
                    }
                }
            }

            Destroy(instance);
#else
            Debug.LogError("Global Benchmark must be run inside the Unity Editor.");
            yield break;
#endif
        }

        currentStatus = "Complete!";
        isRunning = false;
        Debug.Log($"<color=green>[GlobalBenchmark] Finished! Saved {totalCombinations} CSV files to {saveDirectory}</color>");
    }

    private List<BVHBenchmark.DeformationType> GetSelectedDeformations()
    {
        var list = new List<BVHBenchmark.DeformationType>();
        if ((testDeformations & DeformationMask.None) != 0) list.Add(BVHBenchmark.DeformationType.None);
        if ((testDeformations & DeformationMask.SineWave) != 0) list.Add(BVHBenchmark.DeformationType.SineWave);
        if ((testDeformations & DeformationMask.Twist) != 0) list.Add(BVHBenchmark.DeformationType.Twist);
        if ((testDeformations & DeformationMask.Localized) != 0) list.Add(BVHBenchmark.DeformationType.Localized);
        if ((testDeformations & DeformationMask.Pulse) != 0) list.Add(BVHBenchmark.DeformationType.Pulse);
        if ((testDeformations & DeformationMask.SquashStretch) != 0) list.Add(BVHBenchmark.DeformationType.SquashStretch);
        return list;
    }
}