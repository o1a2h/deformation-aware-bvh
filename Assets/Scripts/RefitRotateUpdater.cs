// RefitRotateUpdater.cs — Place in Assets/Scripts/
// Refitting with Tree Rotations (Kopta et al. 2012).
//
// Standard bottom-up O(N) refit, plus at each internal node we consider
// swapping a child with a grandchild if it reduces the SAH cost.
// This maintains better tree quality than plain refitting under deformation.
//
// Complexity: O(N) per frame — visits every node unconditionally.
// The rotations add constant overhead per internal node (check 4 swaps).

using UnityEngine;

public static class RefitRotateUpdater
{
    public static UpdateStats Update(BVHTree tree, Vector3[] curr, int[] meshTris)
    {
        UpdateStats stats = default;
        var sw = System.Diagnostics.Stopwatch.StartNew();

        // ----------------------------------------------------------------
        // STEP 1 — Bottom-up refit (reverse order guarantees children first)
        // ----------------------------------------------------------------
        for (int n = tree.nodeCount - 1; n >= 0; n--)
        {
            stats.nodesVisited++;

            if (tree.IsLeaf(n))
            {
                tree.RecomputeLeafExtremes(n, curr, meshTris);
                tree.RecomputeBounds(n, curr);

                // Count vertices scanned
                stats.verticesChecked += tree.nodes[n].triCount * 3;
            }
            else
            {
                tree.MergeChildExtremes(n, curr);
                tree.RecomputeBounds(n, curr);
            }

            stats.dirtyNodes++; // every node is recomputed
        }

        // ----------------------------------------------------------------
        // STEP 2 — Tree rotations pass (top-down)
        //          At each internal node with grandchildren, consider swaps.
        //          A rotation is performed if it reduces the parent's SA cost.
        // ----------------------------------------------------------------
        for (int n = 0; n < tree.nodeCount; n++)
        {
            if (tree.IsLeaf(n)) continue;

            int L = tree.nodes[n].left;
            int R = tree.nodes[n].right;

            // Try rotating grandchildren of L with R, and vice versa
            TryRotate(tree, n, L, R, curr);
            TryRotate(tree, n, R, L, curr);
        }

        sw.Stop();
        stats.updateTimeMs = sw.Elapsed.TotalMilliseconds;
        return stats;
    }

    /// <summary>
    /// Consider swapping 'other' with each grandchild of 'child'.
    /// If a swap reduces the combined surface area, perform it.
    /// </summary>
    private static void TryRotate(BVHTree tree, int parent, int child, int other,
                                  Vector3[] curr)
    {
        if (tree.IsLeaf(child)) return; // no grandchildren to swap

        int grandL = tree.nodes[child].left;
        int grandR = tree.nodes[child].right;

        float currentCost = SA(tree, child);

        // Option A: swap other ↔ grandL  →  child would contain {other, grandR}
        float costA = CombinedSA(tree, other, grandR, curr);

        // Option B: swap other ↔ grandR  →  child would contain {grandL, other}
        float costB = CombinedSA(tree, grandL, other, curr);

        if (costA < currentCost && costA <= costB)
        {
            // Perform rotation: grandL goes up to parent level, other goes into child
            PerformSwap(tree, parent, child, other, grandL, curr);
        }
        else if (costB < currentCost)
        {
            PerformSwap(tree, parent, child, other, grandR, curr);
        }
    }

    /// <summary>
    /// Swap 'outsider' (sibling of child) with 'grandchild' (child of child).
    /// After swap: child's children become {outsider, remaining_grandchild}.
    /// </summary>
    private static void PerformSwap(BVHTree tree, int parent, int child,
                                    int outsider, int grandchild, Vector3[] curr)
    {
        int otherGrand;
        if (tree.nodes[child].left == grandchild)
        {
            otherGrand = tree.nodes[child].right;
            tree.nodes[child].left = outsider;
        }
        else
        {
            otherGrand = tree.nodes[child].left;
            tree.nodes[child].right = outsider;
        }

        // grandchild replaces outsider as child of parent
        if (tree.nodes[parent].left == outsider)
            tree.nodes[parent].left = grandchild;
        else
            tree.nodes[parent].right = grandchild;

        // Fix parent pointers
        tree.nodes[outsider].parent = child;
        tree.nodes[grandchild].parent = parent;

        // Recompute bounds for child (its children changed)
        tree.MergeChildExtremes(child, curr);
        tree.RecomputeBounds(child, curr);

        // Recompute bounds for parent
        tree.MergeChildExtremes(parent, curr);
        tree.RecomputeBounds(parent, curr);
    }

    private static float SA(BVHTree tree, int n)
    {
        Vector3 d = tree.nodes[n].bounds.size;
        return 2f * (d.x * d.y + d.y * d.z + d.z * d.x);
    }

    private static float CombinedSA(BVHTree tree, int a, int b, Vector3[] curr)
    {
        Bounds combined = tree.nodes[a].bounds;
        combined.Encapsulate(tree.nodes[b].bounds);
        Vector3 d = combined.size;
        return 2f * (d.x * d.y + d.y * d.z + d.z * d.x);
    }
}