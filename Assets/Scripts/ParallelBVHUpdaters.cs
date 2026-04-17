// ParallelBVHUpdaters.cs — Place in Assets/Scripts/
// Multithreaded versions of all four BVH update methods using Unity's C# Job System.
//
// ARCHITECTURE:
//   - All methods use IJobParallelFor for their main O(N) loops
//   - NativeArrays are allocated once per frame and disposed after use
//   - AwareBVH: deformation + inline check run in parallel; dirty flags use
//     a NativeArray<int> with atomic-like writes (last-write-wins for bool flags)
//   - Kinetic/Refit/Dynamic: their main scan loops are parallelized
//   - The bottom-up recomputation pass remains single-threaded (sequential dependency)
//
// USAGE:
//   Toggle BVHBenchmark.useMultithreading = true to switch all methods to parallel.
//   The benchmark CSV logs the same metrics regardless of threading mode.

using Unity.Collections;
using Unity.Jobs;
using Unity.Burst;
using UnityEngine;
using System;
using System.Diagnostics;

// ============================================================
//  SHARED DATA — flat arrays copied into NativeArrays per frame
// ============================================================
public static class ParallelBVHData
{
    /// <summary>
    /// Copy BVH structure into NativeArrays for job access.
    /// Caller must dispose all returned arrays.
    /// </summary>
    public static void CopyToNative(
        BVHTree tree, Vector3[] verts, Vector3[] prevVerts, int[] meshTris,
        out NativeArray<Vector3> nVerts, out NativeArray<Vector3> nPrev,
        out NativeArray<int> nTris, out NativeArray<int> nExtremes,
        out NativeArray<int> nVertToLeaf, out NativeArray<int> nParents,
        out NativeArray<int> nLeftRight, out NativeArray<int> nDirty,
        out NativeArray<int> nTriStart, out NativeArray<int> nTriCount,
        out NativeArray<int> nTriIndices, out NativeArray<int> nIsLeaf)
    {
        int N = verts.Length;
        int nodeCount = tree.nodeCount;

        nVerts = new NativeArray<Vector3>(N, Allocator.TempJob);
        nVerts.CopyFrom(verts);

        nPrev = new NativeArray<Vector3>(N, Allocator.TempJob);
        if (prevVerts != null) nPrev.CopyFrom(prevVerts);

        nTris = new NativeArray<int>(meshTris.Length, Allocator.TempJob);
        nTris.CopyFrom(meshTris);

        nExtremes = new NativeArray<int>(nodeCount * 6, Allocator.TempJob);
        NativeArray<int>.Copy(tree.extremes, nExtremes, nodeCount * 6);

        nVertToLeaf = new NativeArray<int>(N, Allocator.TempJob);
        nVertToLeaf.CopyFrom(tree.vertexToLeaf);

        nParents = new NativeArray<int>(nodeCount, Allocator.TempJob);
        nLeftRight = new NativeArray<int>(nodeCount * 2, Allocator.TempJob);
        nDirty = new NativeArray<int>(nodeCount, Allocator.TempJob);
        nTriStart = new NativeArray<int>(nodeCount, Allocator.TempJob);
        nTriCount = new NativeArray<int>(nodeCount, Allocator.TempJob);
        nIsLeaf = new NativeArray<int>(nodeCount, Allocator.TempJob);

        for (int i = 0; i < nodeCount; i++)
        {
            nParents[i] = tree.nodes[i].parent;
            nLeftRight[i * 2] = tree.nodes[i].left;
            nLeftRight[i * 2 + 1] = tree.nodes[i].right;
            nDirty[i] = 0;
            nTriStart[i] = tree.nodes[i].triStart;
            nTriCount[i] = tree.nodes[i].triCount;
            nIsLeaf[i] = tree.IsLeaf(i) ? 1 : 0;
        }

        nTriIndices = new NativeArray<int>(tree.triIndices.Length, Allocator.TempJob);
        nTriIndices.CopyFrom(tree.triIndices);
    }

    public static void DisposeAll(
        NativeArray<Vector3> nVerts, NativeArray<Vector3> nPrev,
        NativeArray<int> nTris, NativeArray<int> nExtremes,
        NativeArray<int> nVertToLeaf, NativeArray<int> nParents,
        NativeArray<int> nLeftRight, NativeArray<int> nDirty,
        NativeArray<int> nTriStart, NativeArray<int> nTriCount,
        NativeArray<int> nTriIndices, NativeArray<int> nIsLeaf)
    {
        if (nVerts.IsCreated) nVerts.Dispose();
        if (nPrev.IsCreated) nPrev.Dispose();
        if (nTris.IsCreated) nTris.Dispose();
        if (nExtremes.IsCreated) nExtremes.Dispose();
        if (nVertToLeaf.IsCreated) nVertToLeaf.Dispose();
        if (nParents.IsCreated) nParents.Dispose();
        if (nLeftRight.IsCreated) nLeftRight.Dispose();
        if (nDirty.IsCreated) nDirty.Dispose();
        if (nTriStart.IsCreated) nTriStart.Dispose();
        if (nTriCount.IsCreated) nTriCount.Dispose();
        if (nTriIndices.IsCreated) nTriIndices.Dispose();
        if (nIsLeaf.IsCreated) nIsLeaf.Dispose();
    }
}

// ============================================================
//  1. PARALLEL AWARE BVH — Inline check in parallel
// ============================================================
// Each vertex is checked independently. Dirty flag writes are
// last-write-wins on NativeArray<int> (0/1). This is safe because
// dirty=1 is idempotent — multiple threads writing 1 is correct.
// The only risk is a false negative if thread A reads dirty[n]=0
// while thread B is writing 1, but since we walk upward and
// re-check, the worst case is redundant propagation, not missed flags.

[BurstCompile]
public struct AwareInlineCheckJob : IJobParallelFor
{
    [ReadOnly] public NativeArray<Vector3> newPositions;
    [ReadOnly] public NativeArray<Vector3> prevPositions;
    [ReadOnly] public NativeArray<int> extremes;
    [ReadOnly] public NativeArray<int> vertToLeaf;
    [ReadOnly] public NativeArray<int> parents;

    // Written by multiple threads — last-write-wins (idempotent: only write 1)
    [NativeDisableParallelForRestriction]
    public NativeArray<int> dirty;

    public void Execute(int vertIdx)
    {
        Vector3 newPos = newPositions[vertIdx];
        Vector3 oldPos = prevPositions[vertIdx];
        float dx = newPos.x - oldPos.x;
        float dy = newPos.y - oldPos.y;
        float dz = newPos.z - oldPos.z;
        float sqDist = dx * dx + dy * dy + dz * dz;
        if (sqDist < 1e-12f) return;

        int leaf = vertToLeaf[vertIdx];
        if (leaf < 0) return;

        int node = leaf;
        while (node >= 0)
        {
            int b = node * 6;

            // If already dirty and not an extreme, stop
            if (dirty[node] == 1)
            {
                if (extremes[b] != vertIdx && extremes[b + 1] != vertIdx &&
                    extremes[b + 2] != vertIdx && extremes[b + 3] != vertIdx &&
                    extremes[b + 4] != vertIdx && extremes[b + 5] != vertIdx)
                    return;
            }

            bool isExtreme =
                extremes[b] == vertIdx || extremes[b + 1] == vertIdx ||
                extremes[b + 2] == vertIdx || extremes[b + 3] == vertIdx ||
                extremes[b + 4] == vertIdx || extremes[b + 5] == vertIdx;

            bool escaped =
                newPos.x < prevPositions[extremes[b]].x ||
                newPos.x > prevPositions[extremes[b + 1]].x ||
                newPos.y < prevPositions[extremes[b + 2]].y ||
                newPos.y > prevPositions[extremes[b + 3]].y ||
                newPos.z < prevPositions[extremes[b + 4]].z ||
                newPos.z > prevPositions[extremes[b + 5]].z;

            if (escaped)
            {
                dirty[node] = 1;
                node = parents[node];
            }
            else if (isExtreme && sqDist >= 1e-10f)
            {
                dirty[node] = 1;
                node = parents[node];
            }
            else
            {
                return;
            }
        }
    }
}

public static class ParallelAwareUpdater
{
    public static UpdateStats Update(BVHTree tree, Vector3[] newVerts,
                                     Vector3[] prevVerts, int[] meshTris)
    {
        UpdateStats stats = default;
        var sw = Stopwatch.StartNew();

        int N = newVerts.Length;
        int nodeCount = tree.nodeCount;

        // Allocate native arrays
        var nNew = new NativeArray<Vector3>(N, Allocator.TempJob);
        nNew.CopyFrom(newVerts);
        var nPrev = new NativeArray<Vector3>(N, Allocator.TempJob);
        nPrev.CopyFrom(prevVerts);
        var nExtremes = new NativeArray<int>(nodeCount * 6, Allocator.TempJob);
        NativeArray<int>.Copy(tree.extremes, nExtremes, nodeCount * 6);
        var nVertToLeaf = new NativeArray<int>(N, Allocator.TempJob);
        nVertToLeaf.CopyFrom(tree.vertexToLeaf);
        var nParents = new NativeArray<int>(nodeCount, Allocator.TempJob);
        var nDirty = new NativeArray<int>(nodeCount, Allocator.TempJob);
        for (int i = 0; i < nodeCount; i++)
        {
            nParents[i] = tree.nodes[i].parent;
            nDirty[i] = 0;
        }

        // Schedule parallel inline checks
        var job = new AwareInlineCheckJob
        {
            newPositions = nNew,
            prevPositions = nPrev,
            extremes = nExtremes,
            vertToLeaf = nVertToLeaf,
            parents = nParents,
            dirty = nDirty,
        };
        job.Schedule(N, 256).Complete();

        // Copy dirty flags back
        for (int i = 0; i < nodeCount; i++)
        {
            tree.nodes[i].dirty = nDirty[i] == 1;
            if (nDirty[i] == 1) stats.nodesVisited++;
        }

        nNew.Dispose();
        nPrev.Dispose();
        nExtremes.Dispose();
        nVertToLeaf.Dispose();
        nParents.Dispose();
        nDirty.Dispose();

        // Bottom-up recompute (sequential — parent depends on children)
        for (int n = nodeCount - 1; n >= 0; n--)
        {
            if (!tree.nodes[n].dirty) continue;
            stats.dirtyNodes++;
            if (tree.IsLeaf(n))
                tree.RecomputeLeafExtremes(n, newVerts, meshTris);
            else
                tree.MergeChildExtremes(n, newVerts);
            tree.RecomputeBounds(n, newVerts);
        }

        // Count vertices checked (approximate — count vertices in dirty leaves)
        for (int n = 0; n < nodeCount; n++)
        {
            if (tree.nodes[n].dirty && tree.IsLeaf(n))
                stats.verticesChecked += tree.nodes[n].triCount * 3;
        }

        sw.Stop();
        stats.updateTimeMs = sw.Elapsed.TotalMilliseconds;
        return stats;
    }
}

// ============================================================
//  2. PARALLEL REFIT+ROTATION
// ============================================================
// Leaf refit is embarrassingly parallel. Internal nodes must be
// processed bottom-up (sequential). Rotations remain sequential.

[BurstCompile]
public struct RefitLeafJob : IJobParallelFor
{
    [ReadOnly] public NativeArray<Vector3> verts;
    [ReadOnly] public NativeArray<int> meshTris;
    [ReadOnly] public NativeArray<int> triIndices;
    [ReadOnly] public NativeArray<int> triStart;
    [ReadOnly] public NativeArray<int> triCount;
    [ReadOnly] public NativeArray<int> leafNodeIndices;

    [NativeDisableParallelForRestriction]
    public NativeArray<int> extremes;

    public void Execute(int jobIdx)
    {
        int n = leafNodeIndices[jobIdx];
        int start = triStart[n];
        int count = triCount[n];
        int b = n * 6;

        int firstV = meshTris[triIndices[start]];
        for (int a = 0; a < 6; a++) extremes[b + a] = firstV;

        for (int t = start; t < start + count; t++)
        {
            int triBase = triIndices[t];
            for (int i = 0; i < 3; i++)
            {
                int v = meshTris[triBase + i];
                Vector3 p = verts[v];
                if (p.x < verts[extremes[b]].x) extremes[b] = v;
                if (p.x > verts[extremes[b + 1]].x) extremes[b + 1] = v;
                if (p.y < verts[extremes[b + 2]].y) extremes[b + 2] = v;
                if (p.y > verts[extremes[b + 3]].y) extremes[b + 3] = v;
                if (p.z < verts[extremes[b + 4]].z) extremes[b + 4] = v;
                if (p.z > verts[extremes[b + 5]].z) extremes[b + 5] = v;
            }
        }
    }
}

public static class ParallelRefitRotateUpdater
{
    private static int[] leafIndicesCache;
    private static int leafCount;

    public static UpdateStats Update(BVHTree tree, Vector3[] curr, int[] meshTris)
    {
        UpdateStats stats = default;
        var sw = Stopwatch.StartNew();
        int nodeCount = tree.nodeCount;

        // Build leaf index list (cache across frames)
        if (leafIndicesCache == null || leafIndicesCache.Length < nodeCount)
        {
            leafIndicesCache = new int[nodeCount];
        }
        leafCount = 0;
        for (int i = 0; i < nodeCount; i++)
        {
            if (tree.IsLeaf(i))
                leafIndicesCache[leafCount++] = i;
        }

        // Parallel leaf refit
        var nVerts = new NativeArray<Vector3>(curr.Length, Allocator.TempJob);
        nVerts.CopyFrom(curr);
        var nTris = new NativeArray<int>(meshTris.Length, Allocator.TempJob);
        nTris.CopyFrom(meshTris);
        var nTriIndices = new NativeArray<int>(tree.triIndices.Length, Allocator.TempJob);
        nTriIndices.CopyFrom(tree.triIndices);
        var nTriStart = new NativeArray<int>(nodeCount, Allocator.TempJob);
        var nTriCount = new NativeArray<int>(nodeCount, Allocator.TempJob);
        for (int i = 0; i < nodeCount; i++)
        {
            nTriStart[i] = tree.nodes[i].triStart;
            nTriCount[i] = tree.nodes[i].triCount;
        }
        var nExtremes = new NativeArray<int>(nodeCount * 6, Allocator.TempJob);
        NativeArray<int>.Copy(tree.extremes, nExtremes, nodeCount * 6);
        var nLeafIndices = new NativeArray<int>(leafCount, Allocator.TempJob);
        NativeArray<int>.Copy(leafIndicesCache, 0, nLeafIndices, 0, leafCount);

        var leafJob = new RefitLeafJob
        {
            verts = nVerts,
            meshTris = nTris,
            triIndices = nTriIndices,
            triStart = nTriStart,
            triCount = nTriCount,
            leafNodeIndices = nLeafIndices,
            extremes = nExtremes,
        };
        leafJob.Schedule(leafCount, 64).Complete();

        // Copy extremes back
        NativeArray<int>.Copy(nExtremes, 0, tree.extremes, 0, nodeCount * 6);

        nVerts.Dispose();
        nTris.Dispose();
        nTriIndices.Dispose();
        nTriStart.Dispose();
        nTriCount.Dispose();
        nExtremes.Dispose();
        nLeafIndices.Dispose();

        // Recompute leaf bounds
        for (int i = 0; i < leafCount; i++)
        {
            int n = leafIndicesCache[i];
            tree.RecomputeBounds(n, curr);
            stats.verticesChecked += tree.nodes[n].triCount * 3;
            stats.nodesVisited++;
            stats.dirtyNodes++;
        }

        // Sequential bottom-up for internal nodes
        for (int n = nodeCount - 1; n >= 0; n--)
        {
            if (tree.IsLeaf(n)) continue;
            tree.MergeChildExtremes(n, curr);
            tree.RecomputeBounds(n, curr);
            stats.nodesVisited++;
            stats.dirtyNodes++;
        }

        // Tree rotations (sequential — structural changes)
        for (int n = 0; n < nodeCount; n++)
        {
            if (tree.IsLeaf(n)) continue;
            int L = tree.nodes[n].left;
            int R = tree.nodes[n].right;
            RefitRotateUpdater_TryRotate(tree, n, L, R, curr);
            RefitRotateUpdater_TryRotate(tree, n, R, L, curr);
        }

        sw.Stop();
        stats.updateTimeMs = sw.Elapsed.TotalMilliseconds;
        return stats;
    }

    // Rotation logic duplicated to avoid coupling to original class internals
    private static void RefitRotateUpdater_TryRotate(BVHTree tree, int parent,
        int child, int other, Vector3[] curr)
    {
        if (tree.IsLeaf(child)) return;
        int grandL = tree.nodes[child].left;
        int grandR = tree.nodes[child].right;
        float currentCost = BVHTree.GetSurfaceArea(tree.nodes[child].bounds);

        Bounds combA = tree.nodes[other].bounds;
        combA.Encapsulate(tree.nodes[grandR].bounds);
        float costA = BVHTree.GetSurfaceArea(combA);

        Bounds combB = tree.nodes[grandL].bounds;
        combB.Encapsulate(tree.nodes[other].bounds);
        float costB = BVHTree.GetSurfaceArea(combB);

        if (costA < currentCost && costA <= costB)
            PerformSwap(tree, parent, child, other, grandL, curr);
        else if (costB < currentCost)
            PerformSwap(tree, parent, child, other, grandR, curr);
    }

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
        if (tree.nodes[parent].left == outsider)
            tree.nodes[parent].left = grandchild;
        else
            tree.nodes[parent].right = grandchild;
        tree.nodes[outsider].parent = child;
        tree.nodes[grandchild].parent = parent;
        tree.MergeChildExtremes(child, curr);
        tree.RecomputeBounds(child, curr);
        tree.MergeChildExtremes(parent, curr);
        tree.RecomputeBounds(parent, curr);
    }
}

// ============================================================
//  3. PARALLEL KINETIC BVH
// ============================================================
// Velocity computation is embarrassingly parallel.
// Certificate verification per leaf is also parallel (each leaf independent).

[BurstCompile]
public struct ComputeVelocityJob : IJobParallelFor
{
    [ReadOnly] public NativeArray<Vector3> curr;
    [ReadOnly] public NativeArray<Vector3> prev;
    public float invDt;
    [WriteOnly] public NativeArray<Vector3> vel;

    public void Execute(int i)
    {
        vel[i] = (curr[i] - prev[i]) * invDt;
    }
}

[BurstCompile]
public struct KineticLeafCheckJob : IJobParallelFor
{
    [ReadOnly] public NativeArray<Vector3> curr;
    [ReadOnly] public NativeArray<Vector3> vel;
    [ReadOnly] public NativeArray<int> extremes;
    [ReadOnly] public NativeArray<int> meshTris;
    [ReadOnly] public NativeArray<int> triIndices;
    [ReadOnly] public NativeArray<int> triStart;
    [ReadOnly] public NativeArray<int> triCount;
    [ReadOnly] public NativeArray<int> leafNodeIndices;
    public float safeDt;

    [NativeDisableParallelForRestriction]
    public NativeArray<int> dirty;  // per-node: 0 or 1

    public void Execute(int jobIdx)
    {
        int n = leafNodeIndices[jobIdx];
        int b = n * 6;
        int start = triStart[n];
        int count = triCount[n];
        bool leafDirty = false;

        for (int a = 0; a < 6 && !leafDirty; a++)
        {
            int extremeV = extremes[b + a];
            int axisIdx = a / 2;
            bool isMin = (a % 2 == 0);
            float ePos = GetAxis(curr[extremeV], axisIdx);
            float eVel = GetAxis(vel[extremeV], axisIdx);

            for (int t = start; t < start + count && !leafDirty; t++)
            {
                int triBase = triIndices[t];
                for (int i = 0; i < 3; i++)
                {
                    int v = meshTris[triBase + i];
                    if (v == extremeV) continue;
                    float vPos = GetAxis(curr[v], axisIdx);
                    if (isMin && vPos < ePos) { leafDirty = true; break; }
                    if (!isMin && vPos > ePos) { leafDirty = true; break; }
                    float vVel = GetAxis(vel[v], axisIdx);
                    float relVel = isMin ? (eVel - vVel) : (vVel - eVel);
                    if (relVel > 0.0001f)
                    {
                        float dist = isMin ? (vPos - ePos) : (ePos - vPos);
                        float tFail = dist / relVel;
                        if (tFail >= 0f && tFail <= safeDt)
                        { leafDirty = true; break; }
                    }
                }
            }
        }

        if (!leafDirty)
        {
            for (int a = 0; a < 6; a++)
            {
                int ev = extremes[b + a];
                Vector3 v = vel[ev];
                if (v.x * v.x + v.y * v.y + v.z * v.z > 0.0001f)
                { leafDirty = true; break; }
            }
        }

        if (leafDirty)
            dirty[n] = 1;
    }

    private static float GetAxis(Vector3 v, int axis)
    {
        if (axis == 0) return v.x;
        if (axis == 1) return v.y;
        return v.z;
    }
}

public static class ParallelKineticUpdater
{
    private static int[] leafIndicesCache;
    private static int leafCount;

    public static UpdateStats Update(BVHTree tree, Vector3[] prev, Vector3[] curr,
                                     int[] meshTris, float dt)
    {
        UpdateStats stats = default;
        var sw = Stopwatch.StartNew();
        int N = curr.Length;
        int nodeCount = tree.nodeCount;
        float safeDt = Mathf.Max(dt, 0.00001f);

        // Build leaf list
        if (leafIndicesCache == null || leafIndicesCache.Length < nodeCount)
            leafIndicesCache = new int[nodeCount];
        leafCount = 0;
        for (int i = 0; i < nodeCount; i++)
            if (tree.IsLeaf(i))
                leafIndicesCache[leafCount++] = i;

        // Allocate
        var nCurr = new NativeArray<Vector3>(N, Allocator.TempJob);
        nCurr.CopyFrom(curr);
        var nPrev = new NativeArray<Vector3>(N, Allocator.TempJob);
        nPrev.CopyFrom(prev);
        var nVel = new NativeArray<Vector3>(N, Allocator.TempJob);

        // Parallel velocity computation
        new ComputeVelocityJob
        {
            curr = nCurr,
            prev = nPrev,
            invDt = 1f / safeDt,
            vel = nVel
        }.Schedule(N, 512).Complete();

        // Prepare leaf check
        var nExtremes = new NativeArray<int>(nodeCount * 6, Allocator.TempJob);
        NativeArray<int>.Copy(tree.extremes, nExtremes, nodeCount * 6);
        var nTris = new NativeArray<int>(meshTris.Length, Allocator.TempJob);
        nTris.CopyFrom(meshTris);
        var nTriIndices = new NativeArray<int>(tree.triIndices.Length, Allocator.TempJob);
        nTriIndices.CopyFrom(tree.triIndices);
        var nTriStart = new NativeArray<int>(nodeCount, Allocator.TempJob);
        var nTriCount = new NativeArray<int>(nodeCount, Allocator.TempJob);
        var nDirty = new NativeArray<int>(nodeCount, Allocator.TempJob);
        for (int i = 0; i < nodeCount; i++)
        {
            nTriStart[i] = tree.nodes[i].triStart;
            nTriCount[i] = tree.nodes[i].triCount;
            nDirty[i] = 0;
        }
        var nLeafIndices = new NativeArray<int>(leafCount, Allocator.TempJob);
        NativeArray<int>.Copy(leafIndicesCache, 0, nLeafIndices, 0, leafCount);

        // Parallel leaf certificate check
        new KineticLeafCheckJob
        {
            curr = nCurr,
            vel = nVel,
            extremes = nExtremes,
            meshTris = nTris,
            triIndices = nTriIndices,
            triStart = nTriStart,
            triCount = nTriCount,
            leafNodeIndices = nLeafIndices,
            safeDt = safeDt,
            dirty = nDirty,
        }.Schedule(leafCount, 32).Complete();

        // Copy dirty flags back and propagate upward (sequential)
        for (int i = 0; i < nodeCount; i++)
            tree.nodes[i].dirty = false;

        for (int i = 0; i < nodeCount; i++)
        {
            if (nDirty[i] == 1)
            {
                tree.nodes[i].dirty = true;
                int parent = tree.nodes[i].parent;
                while (parent >= 0 && !tree.nodes[parent].dirty)
                {
                    tree.nodes[parent].dirty = true;
                    parent = tree.nodes[parent].parent;
                }
            }
        }

        // Stats
        stats.verticesChecked = N; // velocity computed for all
        for (int i = 0; i < leafCount; i++)
        {
            stats.nodesVisited++;
            int n = leafIndicesCache[i];
            stats.verticesChecked += tree.nodes[n].triCount * 3;
        }

        nCurr.Dispose(); nPrev.Dispose(); nVel.Dispose();
        nExtremes.Dispose(); nTris.Dispose(); nTriIndices.Dispose();
        nTriStart.Dispose(); nTriCount.Dispose(); nDirty.Dispose();
        nLeafIndices.Dispose();

        // Bottom-up recompute (sequential)
        for (int n = nodeCount - 1; n >= 0; n--)
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

// ============================================================
//  4. PARALLEL DYNAMIC FAT-BOUNDS
// ============================================================
// Leaf scanning is embarrassingly parallel. Propagation is sequential.

[BurstCompile]
public struct FatBoundsLeafCheckJob : IJobParallelFor
{
    [ReadOnly] public NativeArray<Vector3> verts;
    [ReadOnly] public NativeArray<int> meshTris;
    [ReadOnly] public NativeArray<int> triIndices;
    [ReadOnly] public NativeArray<int> triStart;
    [ReadOnly] public NativeArray<int> triCount;
    [ReadOnly] public NativeArray<int> leafNodeIndices;
    [ReadOnly] public NativeArray<float> fatMin;
    [ReadOnly] public NativeArray<float> fatMax;

    // Per-leaf output: tight bounds (6 floats per leaf: minX,minY,minZ,maxX,maxY,maxZ)
    [NativeDisableParallelForRestriction]
    public NativeArray<float> tightBounds;

    [NativeDisableParallelForRestriction]
    public NativeArray<int> escaped; // 1 if escaped, per leaf job index

    public void Execute(int jobIdx)
    {
        int n = leafNodeIndices[jobIdx];
        int start = triStart[n];
        int count = triCount[n];

        float tMinX = float.MaxValue, tMinY = float.MaxValue, tMinZ = float.MaxValue;
        float tMaxX = float.MinValue, tMaxY = float.MinValue, tMaxZ = float.MinValue;

        for (int t = start; t < start + count; t++)
        {
            int triBase = triIndices[t];
            for (int i = 0; i < 3; i++)
            {
                Vector3 p = verts[meshTris[triBase + i]];
                if (p.x < tMinX) tMinX = p.x;
                if (p.y < tMinY) tMinY = p.y;
                if (p.z < tMinZ) tMinZ = p.z;
                if (p.x > tMaxX) tMaxX = p.x;
                if (p.y > tMaxY) tMaxY = p.y;
                if (p.z > tMaxZ) tMaxZ = p.z;
            }
        }

        int ob = jobIdx * 6;
        tightBounds[ob] = tMinX; tightBounds[ob + 1] = tMinY; tightBounds[ob + 2] = tMinZ;
        tightBounds[ob + 3] = tMaxX; tightBounds[ob + 4] = tMaxY; tightBounds[ob + 5] = tMaxZ;

        int fb = n * 3;
        escaped[jobIdx] =
            (tMinX < fatMin[fb] || tMinY < fatMin[fb + 1] || tMinZ < fatMin[fb + 2] ||
             tMaxX > fatMax[fb] || tMaxY > fatMax[fb + 1] || tMaxZ > fatMax[fb + 2]) ? 1 : 0;
    }
}

public static class ParallelDynamicBVHUpdater
{
    private static float[] fatMin;
    private static float[] fatMax;
    private static int lastNodeCount;
    private static int[] leafIndicesCache;
    private static int leafCount;

    private const float MARGIN_FRACTION = 0.15f;

    public static UpdateStats Update(BVHTree tree, Vector3[] curr, int[] meshTris)
    {
        UpdateStats stats = default;
        var sw = Stopwatch.StartNew();
        int nodeCount = tree.nodeCount;

        EnsureStorage(nodeCount);

        if (lastNodeCount != nodeCount)
        {
            FullRefitWithFat(tree, curr, meshTris, ref stats);
            sw.Stop();
            stats.updateTimeMs = sw.Elapsed.TotalMilliseconds;
            return stats;
        }

        // Build leaf list
        if (leafIndicesCache == null || leafIndicesCache.Length < nodeCount)
            leafIndicesCache = new int[nodeCount];
        leafCount = 0;
        for (int i = 0; i < nodeCount; i++)
            if (tree.IsLeaf(i))
                leafIndicesCache[leafCount++] = i;

        // Parallel leaf scan
        var nVerts = new NativeArray<Vector3>(curr.Length, Allocator.TempJob);
        nVerts.CopyFrom(curr);
        var nTris = new NativeArray<int>(meshTris.Length, Allocator.TempJob);
        nTris.CopyFrom(meshTris);
        var nTriIndices = new NativeArray<int>(tree.triIndices.Length, Allocator.TempJob);
        nTriIndices.CopyFrom(tree.triIndices);
        var nTriStart = new NativeArray<int>(nodeCount, Allocator.TempJob);
        var nTriCount = new NativeArray<int>(nodeCount, Allocator.TempJob);
        for (int i = 0; i < nodeCount; i++)
        {
            nTriStart[i] = tree.nodes[i].triStart;
            nTriCount[i] = tree.nodes[i].triCount;
        }
        var nLeafIndices = new NativeArray<int>(leafCount, Allocator.TempJob);
        NativeArray<int>.Copy(leafIndicesCache, 0, nLeafIndices, 0, leafCount);
        var nFatMin = new NativeArray<float>(nodeCount * 3, Allocator.TempJob);
        nFatMin.CopyFrom(fatMin.AsSpan(0, nodeCount * 3).ToArray());
        var nFatMax = new NativeArray<float>(nodeCount * 3, Allocator.TempJob);
        nFatMax.CopyFrom(fatMax.AsSpan(0, nodeCount * 3).ToArray());
        var nTight = new NativeArray<float>(leafCount * 6, Allocator.TempJob);
        var nEscaped = new NativeArray<int>(leafCount, Allocator.TempJob);

        new FatBoundsLeafCheckJob
        {
            verts = nVerts,
            meshTris = nTris,
            triIndices = nTriIndices,
            triStart = nTriStart,
            triCount = nTriCount,
            leafNodeIndices = nLeafIndices,
            fatMin = nFatMin,
            fatMax = nFatMax,
            tightBounds = nTight,
            escaped = nEscaped,
        }.Schedule(leafCount, 64).Complete();

        // Process escaped leaves (sequential propagation)
        for (int i = 0; i < nodeCount; i++)
            tree.nodes[i].dirty = false;

        for (int li = 0; li < leafCount; li++)
        {
            stats.nodesVisited++;
            int n = leafIndicesCache[li];
            stats.verticesChecked += tree.nodes[n].triCount * 3;

            if (nEscaped[li] == 0) continue;

            tree.RecomputeLeafExtremes(n, curr, meshTris);
            tree.RecomputeBounds(n, curr);
            SetFatBounds(n, tree.nodes[n].bounds);
            tree.nodes[n].dirty = true;
            stats.dirtyNodes++;

            int parent = tree.nodes[n].parent;
            while (parent >= 0)
            {
                stats.nodesVisited++;
                tree.MergeChildExtremes(parent, curr);
                tree.RecomputeBounds(parent, curr);
                tree.nodes[parent].dirty = true;
                stats.dirtyNodes++;
                SetFatBounds(parent, tree.nodes[parent].bounds);
                // Check containment
                Bounds pb = tree.nodes[parent].bounds;
                int pfb = parent * 3;
                if (pb.min.x >= fatMin[pfb] && pb.min.y >= fatMin[pfb + 1] &&
                    pb.min.z >= fatMin[pfb + 2] && pb.max.x <= fatMax[pfb] &&
                    pb.max.y <= fatMax[pfb + 1] && pb.max.z <= fatMax[pfb + 2])
                    break;
                parent = tree.nodes[parent].parent;
            }
        }

        nVerts.Dispose(); nTris.Dispose(); nTriIndices.Dispose();
        nTriStart.Dispose(); nTriCount.Dispose(); nLeafIndices.Dispose();
        nFatMin.Dispose(); nFatMax.Dispose(); nTight.Dispose(); nEscaped.Dispose();

        sw.Stop();
        stats.updateTimeMs = sw.Elapsed.TotalMilliseconds;
        return stats;
    }

    private static void EnsureStorage(int nodeCount)
    {
        if (fatMin == null || fatMin.Length < nodeCount * 3)
        {
            fatMin = new float[nodeCount * 3];
            fatMax = new float[nodeCount * 3];
            lastNodeCount = 0;
        }
    }

    private static void SetFatBounds(int n, Bounds tight)
    {
        Vector3 diag = tight.size;
        float margin = Mathf.Max(diag.x, Mathf.Max(diag.y, diag.z)) * MARGIN_FRACTION;
        int fb = n * 3;
        fatMin[fb] = tight.min.x - margin;
        fatMin[fb + 1] = tight.min.y - margin;
        fatMin[fb + 2] = tight.min.z - margin;
        fatMax[fb] = tight.max.x + margin;
        fatMax[fb + 1] = tight.max.y + margin;
        fatMax[fb + 2] = tight.max.z + margin;
    }

    private static void FullRefitWithFat(BVHTree tree, Vector3[] curr,
        int[] meshTris, ref UpdateStats stats)
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
                tree.MergeChildExtremes(n, curr);
            tree.RecomputeBounds(n, curr);
            SetFatBounds(n, tree.nodes[n].bounds);
        }
        lastNodeCount = tree.nodeCount;
    }

    public static void Reset() { lastNodeCount = 0; }
}