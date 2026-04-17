// BVHTree.cs — Place in Assets/Scripts/
using UnityEngine;
using System;
using System.Collections.Generic;

public struct UpdateStats
{
    public int nodesVisited;
    public int verticesChecked;
    public int dirtyNodes;
    public double updateTimeMs;
}

public class BVHTree
{
    public struct NodeData
    {
        public Bounds bounds;
        public int left, right, parent, depth;
        public int triStart, triCount;
        public bool dirty;
    }

    public NodeData[] nodes;
    public int[] extremes;       // 6 per node
    public int[] triIndices;
    public int[] vertexToLeaf;
    public Bounds[] fatBounds;   // Larsson: enlarged bounds with margin
    public int nodeCount;
    public int maxTrisPerLeaf;
    public int maxDepth;

    private int capacity;
    private List<int> tempTris;

    public bool IsLeaf(int i) => nodes[i].left == -1;

    // ==== Construction ====
    public void Build(Vector3[] verts, int[] tris, int maxTrisPerLeaf = 10,
                      int maxDepth = 20, float fatMargin = 0.05f)
    {
        this.maxTrisPerLeaf = maxTrisPerLeaf;
        this.maxDepth = maxDepth;

        int triCount = tris.Length / 3;
        capacity = Mathf.Max(triCount * 2 + 1, 64);

        nodes = new NodeData[capacity];
        extremes = new int[capacity * 6];
        fatBounds = new Bounds[capacity];
        tempTris = new List<int>(triCount);
        vertexToLeaf = new int[verts.Length];
        Array.Fill(vertexToLeaf, -1);
        nodeCount = 0;

        var all = new List<int>(triCount);
        for (int i = 0; i < tris.Length; i += 3) all.Add(i);

        BuildRecursive(all, verts, tris, -1, 0);
        triIndices = tempTris.ToArray();
        tempTris = null;

        // Initialize fat bounds
        for (int i = 0; i < nodeCount; i++)
        {
            Bounds fb = nodes[i].bounds;
            fb.Expand(fatMargin);
            fatBounds[i] = fb;
        }
    }

    private int BuildRecursive(List<int> tris, Vector3[] verts, int[] meshTris,
                               int parentIdx, int depth)
    {
        int idx = nodeCount++;
        if (idx >= capacity) Grow();

        nodes[idx].parent = parentIdx;
        nodes[idx].depth = depth;
        nodes[idx].left = -1;
        nodes[idx].right = -1;

        Vector3 bmin = Vector3.positiveInfinity, bmax = Vector3.negativeInfinity;
        int b = idx * 6;
        int firstV = meshTris[tris[0]];
        for (int a = 0; a < 6; a++) extremes[b + a] = firstV;

        foreach (int triBase in tris)
        {
            for (int i = 0; i < 3; i++)
            {
                int v = meshTris[triBase + i];
                Vector3 p = verts[v];
                if (p.x < bmin.x) { bmin.x = p.x; extremes[b] = v; }
                if (p.x > bmax.x) { bmax.x = p.x; extremes[b + 1] = v; }
                if (p.y < bmin.y) { bmin.y = p.y; extremes[b + 2] = v; }
                if (p.y > bmax.y) { bmax.y = p.y; extremes[b + 3] = v; }
                if (p.z < bmin.z) { bmin.z = p.z; extremes[b + 4] = v; }
                if (p.z > bmax.z) { bmax.z = p.z; extremes[b + 5] = v; }
            }
        }
        nodes[idx].bounds = new Bounds();
        nodes[idx].bounds.SetMinMax(bmin, bmax);

        if (tris.Count <= maxTrisPerLeaf || depth >= maxDepth)
        {
            MakeLeaf(idx, tris, meshTris);
            return idx;
        }

        Vector3 size = bmax - bmin;
        int axis = (size.y > size.x && size.y > size.z) ? 1 : (size.z > size.x ? 2 : 0);
        float split = bmin[axis] + size[axis] * 0.5f;

        var leftList = new List<int>();
        var rightList = new List<int>();
        foreach (int triBase in tris)
        {
            float c = (verts[meshTris[triBase]][axis]
                     + verts[meshTris[triBase + 1]][axis]
                     + verts[meshTris[triBase + 2]][axis]) / 3f;
            if (c < split) leftList.Add(triBase); else rightList.Add(triBase);
        }

        if (leftList.Count == 0 || rightList.Count == 0)
        {
            MakeLeaf(idx, tris, meshTris);
            return idx;
        }

        nodes[idx].left = BuildRecursive(leftList, verts, meshTris, idx, depth + 1);
        nodes[idx].right = BuildRecursive(rightList, verts, meshTris, idx, depth + 1);
        return idx;
    }

    private void MakeLeaf(int idx, List<int> tris, int[] meshTris)
    {
        nodes[idx].triStart = tempTris.Count;
        nodes[idx].triCount = tris.Count;
        tempTris.AddRange(tris);

        foreach (int triBase in tris)
            for (int i = 0; i < 3; i++)
            {
                int v = meshTris[triBase + i];
                if (vertexToLeaf[v] == -1) vertexToLeaf[v] = idx;
            }
    }

    // ==== Helpers for updaters ====

    public void RecomputeLeafExtremes(int n, Vector3[] verts, int[] meshTris)
    {
        int start = nodes[n].triStart, count = nodes[n].triCount, b = n * 6;
        int first = meshTris[triIndices[start]];
        for (int a = 0; a < 6; a++) extremes[b + a] = first;

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

    public void MergeChildExtremes(int n, Vector3[] verts)
    {
        int l = nodes[n].left, r = nodes[n].right;
        int b = n * 6, lb = l * 6, rb = r * 6;
        extremes[b]     = (verts[extremes[lb]].x     < verts[extremes[rb]].x)     ? extremes[lb]     : extremes[rb];
        extremes[b + 1] = (verts[extremes[lb + 1]].x > verts[extremes[rb + 1]].x) ? extremes[lb + 1] : extremes[rb + 1];
        extremes[b + 2] = (verts[extremes[lb + 2]].y < verts[extremes[rb + 2]].y) ? extremes[lb + 2] : extremes[rb + 2];
        extremes[b + 3] = (verts[extremes[lb + 3]].y > verts[extremes[rb + 3]].y) ? extremes[lb + 3] : extremes[rb + 3];
        extremes[b + 4] = (verts[extremes[lb + 4]].z < verts[extremes[rb + 4]].z) ? extremes[lb + 4] : extremes[rb + 4];
        extremes[b + 5] = (verts[extremes[lb + 5]].z > verts[extremes[rb + 5]].z) ? extremes[lb + 5] : extremes[rb + 5];
    }

    public void RecomputeBounds(int n, Vector3[] verts)
    {
        int b = n * 6;
        Vector3 lo = new Vector3(verts[extremes[b]].x, verts[extremes[b + 2]].y, verts[extremes[b + 4]].z);
        Vector3 hi = new Vector3(verts[extremes[b + 1]].x, verts[extremes[b + 3]].y, verts[extremes[b + 5]].z);
        nodes[n].bounds.SetMinMax(lo, hi);
    }

    /// <summary>Recompute bounds as union of children (no extremes needed).</summary>
    public void RecomputeBoundsFromChildren(int n)
    {
        if (IsLeaf(n)) return;
        Bounds lb = nodes[nodes[n].left].bounds;
        Bounds rb = nodes[nodes[n].right].bounds;
        Vector3 mn = Vector3.Min(lb.min, rb.min);
        Vector3 mx = Vector3.Max(lb.max, rb.max);
        nodes[n].bounds.SetMinMax(mn, mx);
    }

    /// <summary>Recompute leaf bounds by scanning all contained vertices.</summary>
    public void RecomputeLeafBounds(int n, Vector3[] verts, int[] meshTris)
    {
        int start = nodes[n].triStart, count = nodes[n].triCount;
        Vector3 mn = Vector3.positiveInfinity, mx = Vector3.negativeInfinity;
        for (int t = start; t < start + count; t++)
        {
            int triBase = triIndices[t];
            for (int i = 0; i < 3; i++)
            {
                Vector3 p = verts[meshTris[triBase + i]];
                mn = Vector3.Min(mn, p);
                mx = Vector3.Max(mx, p);
            }
        }
        nodes[n].bounds.SetMinMax(mn, mx);
    }

    // ==== Tree Rotations (Kopta 2012) ====
    // At node N with left=L, right=R:
    // If R is internal (children RL,RR): try swap L↔RL, try swap L↔RR
    // If L is internal (children LL,LR): try swap R↔LL, try swap R↔LR
    // Accept the swap that gives the best (lowest) SA for N.

    public bool TryRotateNode(int n)
    {
        if (IsLeaf(n)) return false;
        int L = nodes[n].left, R = nodes[n].right;

        float bestSA = GetSurfaceArea(nodes[n].bounds);
        int bestSwapChild = -1, bestSwapGrand = -1;

        // Try swaps involving R's children
        if (!IsLeaf(R))
        {
            TrySwap(L, nodes[R].left, R, ref bestSA, ref bestSwapChild, ref bestSwapGrand);
            TrySwap(L, nodes[R].right, R, ref bestSA, ref bestSwapChild, ref bestSwapGrand);
        }
        // Try swaps involving L's children
        if (!IsLeaf(L))
        {
            TrySwap(R, nodes[L].left, L, ref bestSA, ref bestSwapChild, ref bestSwapGrand);
            TrySwap(R, nodes[L].right, L, ref bestSA, ref bestSwapChild, ref bestSwapGrand);
        }

        if (bestSwapChild < 0) return false;

        // Perform the swap
        int grandParent = bestSwapGrand == nodes[n].left ? nodes[n].left : nodes[n].right;
        // Identify which child of N is being swapped out
        bool swapIsLeft = (bestSwapChild == L);

        // The grandchild comes up to N, the child goes down to grandParent
        if (bestSwapGrand == nodes[grandParent].left)
            nodes[grandParent].left = bestSwapChild;
        else
            nodes[grandParent].right = bestSwapChild;
        nodes[bestSwapChild].parent = grandParent;

        if (swapIsLeft)
            nodes[n].left = bestSwapGrand;
        else
            nodes[n].right = bestSwapGrand;
        nodes[bestSwapGrand].parent = n;

        // Recompute bounds for grandParent, then N
        RecomputeBoundsFromChildren(grandParent);
        RecomputeBoundsFromChildren(n);
        return true;
    }

    private void TrySwap(int child, int grandchild, int otherChild,
                         ref float bestSA, ref int bestSwapChild, ref int bestSwapGrand)
    {
        // If we swap child and grandchild:
        // otherChild's bounds become union(child, sibling_of_grandchild)
        int siblingOfGrand = (nodes[otherChild].left == grandchild)
            ? nodes[otherChild].right : nodes[otherChild].left;

        // New otherChild bounds = union(child, siblingOfGrand)
        Bounds newOtherBounds = nodes[child].bounds;
        newOtherBounds.Encapsulate(nodes[siblingOfGrand].bounds);

        // New N bounds = union(grandchild, newOtherBounds)
        Bounds newNBounds = nodes[grandchild].bounds;
        newNBounds.Encapsulate(newOtherBounds);

        float sa = GetSurfaceArea(newNBounds);
        if (sa < bestSA - 0.0001f)
        {
            bestSA = sa;
            bestSwapChild = child;
            bestSwapGrand = grandchild;
        }
    }

    public static float GetSurfaceArea(Bounds b)
    {
        Vector3 d = b.size;
        return 2f * (d.x * d.y + d.y * d.z + d.z * d.x);
    }

    private void Grow()
    {
        capacity *= 2;
        Array.Resize(ref nodes, capacity);
        Array.Resize(ref extremes, capacity * 6);
        Array.Resize(ref fatBounds, capacity);
    }
}