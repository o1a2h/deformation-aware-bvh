// KineticUpdater.cs - Place in Assets/Scripts/
// Faithful discrete-timestep adaptation of Zachmann & Weller (2006).
//
// The original kinetic BVH relies on continuous-time event queues:
//   - Store certificates ("vertex v is the extreme for axis a of node n")
//   - Predict the exact time each certificate will fail via velocity extrapolation
//   - Process only expired certificates
//
// Under discrete PBD / procedural deformation, velocities change every frame,
// so ALL predictions become stale and must be recomputed.  This implementation
// honestly reflects that cost:
//   1. Compute velocities for ALL vertices                   → O(N)
//   2. For every leaf, scan ALL contained vertices to verify
//      certificates and predict failures via extrapolation   → O(N)
//   3. Recompute bounds for invalidated nodes                → O(dirty)
//
// The benchmark should show that steps 1-2 are the bottleneck compared to the
// Aware method, which skips inactive vertices entirely.

using UnityEngine;
using System;

public static class KineticUpdater
{
    /// <param name="tree">BVH with extremes from the previous frame.</param>
    /// <param name="prev">Vertex positions last frame.</param>
    /// <param name="curr">Vertex positions this frame.</param>
    /// <param name="meshTris">Mesh triangle index buffer.</param>
    /// <param name="dt">Time since last frame (for velocity computation).</param>
    public static UpdateStats Update(BVHTree tree, Vector3[] prev, Vector3[] curr,
                                     int[] meshTris, float dt)
    {
        UpdateStats stats = default;
        var sw = System.Diagnostics.Stopwatch.StartNew();

        int N = curr.Length;
        float safeDt = Mathf.Max(dt, 0.00001f);

        // ----------------------------------------------------------------
        // STEP 1 - Compute per-vertex velocities  (O(N))
        //          In Zachmann's framework these drive the failure predictions.
        // ----------------------------------------------------------------
        Vector3[] vel = new Vector3[N];
        for (int i = 0; i < N; i++)
            vel[i] = (curr[i] - prev[i]) / safeDt;

        // ----------------------------------------------------------------
        // STEP 2 - Clear dirty flags
        // ----------------------------------------------------------------
        for (int n = 0; n < tree.nodeCount; n++)
            tree.nodes[n].dirty = false;

        // ----------------------------------------------------------------
        // STEP 3 - Certificate verification for every leaf
        //          For each of the 6 extreme axes, check whether:
        //            a) any vertex already overtook the stored extreme, OR
        //            b) velocity extrapolation predicts failure within dt
        //          Also mark dirty if any extreme vertex itself moved.
        // ----------------------------------------------------------------
        for (int n = 0; n < tree.nodeCount; n++)
        {
            if (!tree.IsLeaf(n)) continue;
            stats.nodesVisited++;

            int b = n * 6;
            int start = tree.nodes[n].triStart;
            int count = tree.nodes[n].triCount;
            bool leafDirty = false;

            // Check each of the 6 certificates
            for (int a = 0; a < 6 && !leafDirty; a++)
            {
                int extremeV = tree.extremes[b + a];
                int axisIdx = a / 2;           // 0=x 1=y 2=z
                bool isMin = (a % 2 == 0);

                float ePos = curr[extremeV][axisIdx];
                float eVel = vel[extremeV][axisIdx];

                // Scan every vertex in the leaf
                for (int t = start; t < start + count; t++)
                {
                    int triBase = tree.triIndices[t];
                    for (int i = 0; i < 3; i++)
                    {
                        int v = meshTris[triBase + i];
                        if (v == extremeV) continue;
                        stats.verticesChecked++;

                        float vPos = curr[v][axisIdx];

                        // (a) Direct overtaking - already happened
                        if (isMin && vPos < ePos) { leafDirty = true; break; }
                        if (!isMin && vPos > ePos) { leafDirty = true; break; }

                        // (b) Velocity-based prediction - will happen within dt?
                        float vVel = vel[v][axisIdx];
                        float relVel = isMin ? (eVel - vVel) : (vVel - eVel);
                        if (relVel > 0.0001f)
                        {
                            float dist = isMin ? (vPos - ePos) : (ePos - vPos);
                            float tFail = dist / relVel;
                            if (tFail >= 0f && tFail <= safeDt)
                            { leafDirty = true; break; }
                        }
                    }
                    if (leafDirty) break;
                }
            }

            // Even if no challenger overtook, the extreme vertex itself may have moved
            if (!leafDirty)
            {
                for (int a = 0; a < 6; a++)
                {
                    int ev = tree.extremes[b + a];
                    if (vel[ev].sqrMagnitude > 0.0001f)
                    { leafDirty = true; break; }
                }
            }

            if (!leafDirty) continue;

            // Mark this leaf + all ancestors dirty
            tree.nodes[n].dirty = true;
            int parent = tree.nodes[n].parent;
            while (parent >= 0 && !tree.nodes[parent].dirty)
            {
                tree.nodes[parent].dirty = true;
                parent = tree.nodes[parent].parent;
            }
        }

        // ----------------------------------------------------------------
        // STEP 4 - Bottom-up recompute of dirty nodes (same as Aware)
        // ----------------------------------------------------------------
        for (int n = tree.nodeCount - 1; n >= 0; n--)
        {
            if (!tree.nodes[n].dirty) continue;
            stats.dirtyNodes++;

            if (tree.IsLeaf(n))
                tree.RecomputeLeafExtremes(n, curr, meshTris);
            else
                tree.MergeChildExtremes(n, curr);

            tree.RecomputeBounds(n, curr);
        }

        sw.Stop();
        stats.updateTimeMs = sw.Elapsed.TotalMilliseconds;
        return stats;
    }
}