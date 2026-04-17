// AwareUpdater.cs — Place in Assets/Scripts/
// Deformation-Aware BVH: inline vertex checking during the deformation loop.
//
// Architecture (as described in the paper):
//   The physics/deformation solver updates each vertex. DURING that update,
//   we check if the vertex escaped its leaf's bounds or is an extreme vertex.
//   If so, we propagate dirty flags upward. After the loop, a single bottom-up
//   pass recomputes bounds for dirty nodes only.
//
// Cost per non-deforming vertex: O(1) — 6 int comparisons + 6 float comparisons
// Cost per deforming vertex that escapes: O(log N) — ancestor walk
// No separate N-loops for centroids, residuals, or filtering.

using UnityEngine;
using System.Runtime.CompilerServices;

public static class AwareUpdater
{
    /// <summary>
    /// Call ONCE before the deformation loop to reset dirty flags.
    /// </summary>
    public static void BeginFrame(BVHTree tree)
    {
        for (int n = 0; n < tree.nodeCount; n++)
            tree.nodes[n].dirty = false;
    }

    /// <summary>
    /// Call INSIDE the deformation loop, immediately after computing each vertex's
    /// new position. This is the core kinetic overtaking check — O(1) for vertices
    /// that stay within bounds, O(log N) for vertices that escape.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static void CheckVertex(BVHTree tree, int vertIdx, Vector3 newPos,
                                   Vector3[] verts, ref UpdateStats stats)
    {
        float dx = newPos.x - verts[vertIdx].x;
        float dy = newPos.y - verts[vertIdx].y;
        float dz = newPos.z - verts[vertIdx].z;
        if (dx * dx + dy * dy + dz * dz < 1e-12f) return;


        int leaf = tree.vertexToLeaf[vertIdx];
        if (leaf < 0) return;

        int node = leaf;
        int[] ext = tree.extremes;

        while (node >= 0)
        {
            int b = node * 6;

            // If already dirty and we're not an extreme here, stop
            if (tree.nodes[node].dirty)
            {
                if (ext[b] != vertIdx && ext[b + 1] != vertIdx &&
                    ext[b + 2] != vertIdx && ext[b + 3] != vertIdx &&
                    ext[b + 4] != vertIdx && ext[b + 5] != vertIdx)
                    return;
            }

            // Check 1: is this vertex one of the 6 extremes?
            bool isExtreme =
                ext[b] == vertIdx || ext[b + 1] == vertIdx ||
                ext[b + 2] == vertIdx || ext[b + 3] == vertIdx ||
                ext[b + 4] == vertIdx || ext[b + 5] == vertIdx;

            // Check 2: does the vertex lie outside current bounds?
            bool escaped =
                newPos.x < verts[ext[b]].x || newPos.x > verts[ext[b + 1]].x ||
                newPos.y < verts[ext[b + 2]].y || newPos.y > verts[ext[b + 3]].y ||
                newPos.z < verts[ext[b + 4]].z || newPos.z > verts[ext[b + 5]].z;

            if (escaped)
            {
                // Vertex overtook bounds — definitely dirty
                if (node == leaf) stats.verticesChecked++;
                stats.nodesVisited++;
                tree.nodes[node].dirty = true;
                node = tree.nodes[node].parent;
            }
            else if (isExtreme)
            {
                // Extreme vertex — only dirty if it actually moved
                if (dx * dx + dy * dy + dz * dz < 1e-10f)
                    return; // stationary extreme — bounds unchanged

                if (node == leaf) stats.verticesChecked++;
                stats.nodesVisited++;
                tree.nodes[node].dirty = true;
                node = tree.nodes[node].parent;
            }
            else
            {
                return; // inside bounds, not extreme — zero cost
            }
        }
    }

    /// <summary>
    /// Call ONCE after the deformation loop. Recomputes extremes and bounds
    /// for dirty nodes only, bottom-up.
    /// </summary>
    public static void RecomputeDirty(BVHTree tree, Vector3[] verts, int[] meshTris,
                                      ref UpdateStats stats)
    {
        for (int n = tree.nodeCount - 1; n >= 0; n--)
        {
            if (!tree.nodes[n].dirty) continue;
            stats.dirtyNodes++;

            if (tree.IsLeaf(n))
                tree.RecomputeLeafExtremes(n, verts, meshTris);
            else
                tree.MergeChildExtremes(n, verts);

            tree.RecomputeBounds(n, verts);
        }
    }
}