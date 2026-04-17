// GlobalBenchmarkEditor.cs , Place in Assets/Scripts/Editor/
using UnityEditor;
using UnityEngine;

[CustomEditor(typeof(GlobalBenchmark))]
public class GlobalBenchmarkEditor : Editor
{
    public override void OnInspectorGUI()
    {
        GlobalBenchmark gb = (GlobalBenchmark)target;

        EditorGUILayout.Space(10);
        EditorGUILayout.LabelField("Global Benchmark Orchestrator", EditorStyles.boldLabel);

        EditorGUI.BeginDisabledGroup(gb.isRunning);

        // Deformations Checkbox List
        gb.testDeformations = (GlobalBenchmark.DeformationMask)EditorGUILayout.EnumFlagsField("Deformations", gb.testDeformations);

        // Triangle Settings
        gb.maxTrianglesPerLeaf = EditorGUILayout.TextField("Max Triangles Per Leaf", gb.maxTrianglesPerLeaf);

        // Number of Runs
        gb.benchmarkRuns = EditorGUILayout.IntField("Runs per Benchmark", gb.benchmarkRuns);

        EditorGUILayout.Space(5);
        EditorGUILayout.LabelField("Threading Modes", EditorStyles.boldLabel);
        gb.testSingleThread = EditorGUILayout.Toggle("Test Single-threaded", gb.testSingleThread);
        gb.testMultithread = EditorGUILayout.Toggle("Test Multithreaded", gb.testMultithread);

        EditorGUILayout.Space(5);
        gb.saveDirectory = EditorGUILayout.TextField("Save Directory", gb.saveDirectory);

        EditorGUI.EndDisabledGroup();

        EditorGUILayout.Space(15);

        GUI.backgroundColor = gb.isRunning ? Color.gray : Color.green;
        if (GUILayout.Button(gb.isRunning ? "Benchmark Running..." : "▶ Start Global Benchmark", GUILayout.Height(30)))
        {
            if (!Application.isPlaying)
            {
                Debug.LogWarning("[GlobalBenchmark] Please press the Unity Play button first, then click Start Global Benchmark.");
            }
            else if (!gb.testSingleThread && !gb.testMultithread)
            {
                Debug.LogWarning("[GlobalBenchmark] You must select at least one threading mode.");
            }
            else
            {
                gb.StartGlobalBenchmark();
            }
        }
        GUI.backgroundColor = Color.white;

        if (gb.isRunning || gb.currentStatus != "Idle")
        {
            EditorGUILayout.Space(10);
            EditorGUILayout.HelpBox(gb.currentStatus, MessageType.Info);
            Repaint();
        }
    }
}