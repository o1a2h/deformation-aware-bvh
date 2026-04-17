// DynamicBVHUpdater.cs — Place in Assets/Scripts/
// Larsson & Akenine-Möller (2006) dynamic BVH with fat/enlarged bounds.
//
// Key idea: each node stores a "fat" bounding box enlarged by a margin.
// During update, only check if leaf geometry has escaped its fat bounds.
// If yes: refit that leaf and propagate upward until an ancestor's fat
// bounds still contain the child. This avoids full-tree traversal.
//
// The margin is recomputed on each refit: fatBounds = bounds enlarged by
// a fraction of the diagonal (typically 10-20%).
//
// Complexity:
//   Best case:  O(K · log N)  where K = leaves whose geometry escaped
//   Worst case: O(N)          when deformation exceeds all margins
//
// This method exploits temporal coherence through spatial margin,
// unlike the Aware method which uses vertex-level kinetic tracking.

using UnityEngine;
using System;

public static class DynamicBVHUpdater
{
    // Extra arrays stored alongside the tree
    // fatBoundsMin/Max: 3 floats per node, flattened
    private static float[] fatMin;
    private static float[] fatMax;
    private static int lastNodeCount;

    /// <summary>Margin as fraction of node diagonal.</summary>
    private const float MARGIN_FRACTION = 0.15f;

    public static UpdateStats Update(BVHTree tree, Vector3[] curr, int[] meshTris)
    {
        UpdateStats stats = default;
        var sw = System.Diagnostics.Stopwatch.StartNew();

        // Allocate or reallocate fat bounds storage
        EnsureStorage(tree.nodeCount);

        // If fat bounds were never initialized, do a full refit first
        if (!HasFatBounds(tree))
        {
            FullRefitWithFat(tree, curr, meshTris, ref stats);
            sw.Stop();
            stats.updateTimeMs = sw.Elapsed.TotalMilliseconds;
            return stats;
        }

        // ----------------------------------------------------------------
        // STEP 1 — Check each leaf: has geometry escaped fat bounds?
        // ----------------------------------------------------------------
        for (int n = 0; n < tree.nodeCount; n++)
            tree.nodes[n].dirty = false;

        for (int n = 0; n < tree.nodeCount; n++)
        {
            if (!tree.IsLeaf(n)) continue;
            stats.nodesVisited++;

            // Compute tight bounds for this leaf
            int start = tree.nodes[n].triStart;
            int count = tree.nodes[n].triCount;

            Vector3 tightMin = Vector3.positiveInfinity;
            Vector3 tightMax = Vector3.negativeInfinity;

            for (int t = start; t < start + count; t++)
            {
                int triBase = tree.triIndices[t];
                for (int i = 0; i < 3; i++)
                {
                    int v = meshTris[triBase + i];
                    Vector3 p = curr[v];
                    stats.verticesChecked++;

                    if (p.x < tightMin.x) tightMin.x = p.x;
                    if (p.y < tightMin.y) tightMin.y = p.y;
                    if (p.z < tightMin.z) tightMin.z = p.z;
                    if (p.x > tightMax.x) tightMax.x = p.x;
                    if (p.y > tightMax.y) tightMax.y = p.y;
                    if (p.z > tightMax.z) tightMax.z = p.z;
                }
            }

            // Check if tight bounds exceed fat bounds
            int fb = n * 3;
            bool escaped =
                tightMin.x < fatMin[fb] || tightMin.y < fatMin[fb + 1] || tightMin.z < fatMin[fb + 2] ||
                tightMax.x > fatMax[fb] || tightMax.y > fatMax[fb + 1] || tightMax.z > fatMax[fb + 2];

            if (!escaped) continue;

            // ---- Leaf escaped: refit this leaf and propagate up ----
            tree.RecomputeLeafExtremes(n, curr, meshTris);
            tree.RecomputeBounds(n, curr);
            SetFatBounds(n, tree.nodes[n].bounds);
            tree.nodes[n].dirty = true;
            stats.dirtyNodes++;

            // Propagate upward
            int parent = tree.nodes[n].parent;
            while (parent >= 0)
            {
                stats.nodesVisited++;

                // Recompute parent from children
                tree.MergeChildExtremes(parent, curr);
                tree.RecomputeBounds(parent, curr);

                // Check if parent's new bounds still within its fat bounds
                Bounds pb = tree.nodes[parent].bounds;
                int pfb = parent * 3;
                bool parentContained =
                    pb.min.x >= fatMin[pfb] && pb.min.y >= fatMin[pfb + 1] && pb.min.z >= fatMin[pfb + 2] &&
                    pb.max.x <= fatMax[pfb] && pb.max.y <= fatMax[pfb + 1] && pb.max.z <= fatMax[pfb + 2];

                tree.nodes[parent].dirty = true;
                stats.dirtyNodes++;
                SetFatBounds(parent, pb);

                if (parentContained)
                    break; // fat bounds still hold — stop climbing

                parent = tree.nodes[parent].parent;
            }
        }

        sw.Stop();
        stats.updateTimeMs = sw.Elapsed.TotalMilliseconds;
        return stats;
    }

    // ---- Fat bounds helpers ----

    private static void EnsureStorage(int nodeCount)
    {
        if (fatMin == null || fatMin.Length < nodeCount * 3)
        {
            fatMin = new float[nodeCount * 3];
            fatMax = new float[nodeCount * 3];
            lastNodeCount = 0; // force init
        }
    }

    private static bool HasFatBounds(BVHTree tree)
    {
        return lastNodeCount == tree.nodeCount;
    }

    private static void SetFatBounds(int n, Bounds tight)
    {
        Vector3 diag = tight.size;
        float margin = Mathf.Max(diag.x, Mathf.Max(diag.y, diag.z)) * MARGIN_FRACTION;
        Vector3 m = new Vector3(margin, margin, margin);

        int fb = n * 3;
        Vector3 lo = tight.min - m;
        Vector3 hi = tight.max + m;
        fatMin[fb] = lo.x; fatMin[fb + 1] = lo.y; fatMin[fb + 2] = lo.z;
        fatMax[fb] = hi.x; fatMax[fb + 1] = hi.y; fatMax[fb + 2] = hi.z;
    }

    private static void FullRefitWithFat(BVHTree tree, Vector3[] curr, int[] meshTris,
                                         ref UpdateStats stats)
    {
        for (int n = tree.nodeCount - 1; n >= 0; n--)
        {
            stats.nodesVisited++;
            stats.dirtyNodes++;

            if (tree.IsLeaf(n))
            {
                tree.RecomputeLeafExtremes(n, curr, meshTris);
                stats.verticesChecked += tree.nodes[n].triCount * 3;
            }
            else
            {
                tree.MergeChildExtremes(n, curr);
            }

            tree.RecomputeBounds(n, curr);
            SetFatBounds(n, tree.nodes[n].bounds);
        }
        lastNodeCount = tree.nodeCount;
    }

    /// <summary>Call this when the BVH is rebuilt to force fat bounds re-init.</summary>
    public static void Reset()
    {
        lastNodeCount = 0;
    }
}