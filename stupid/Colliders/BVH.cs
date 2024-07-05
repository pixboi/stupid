using System;
using System.Collections.Generic;
using stupid.Maths;

namespace stupid.Colliders
{
    public class BVHNode
    {
        public SBounds Bounds;
        public BVHNode Left;
        public BVHNode Right;
        public int ObjectIndex; // Index of the object in the leaf node, -1 for internal nodes

        public bool IsLeaf() => ObjectIndex != -1;
    }

    public class BVH
    {
        public BVHNode Root;
        private List<SRigidbody> _rigidbodies;
        private int _frameCount;
        private const int RebuildThreshold = 100; // Rebuild after 100 frames

        public BVH(List<SRigidbody> rigidbodies)
        {
            _rigidbodies = rigidbodies;
            Rebuild(_rigidbodies);
        }

        public void Rebuild(List<SRigidbody> rigidbodies)
        {
            _rigidbodies = rigidbodies;

            int[] indices = new int[_rigidbodies.Count];
            for (int i = 0; i < indices.Length; i++)
            {
                indices[i] = i;
            }

            Root = BuildBVH(indices, 0, indices.Length);
        }

        private BVHNode BuildBVH(int[] indices, int start, int end)
        {
            int count = end - start;
            if (count == 0)
            {
                return null;
            }

            BVHNode node = new BVHNode();

            if (count == 1)
            {
                int index = indices[start];
                node.Bounds = _rigidbodies[index].collider.GetBounds();
                node.ObjectIndex = _rigidbodies[index].index;
                return node;
            }

            // Compute the bounding box that encompasses all objects
            SBounds combinedBounds = _rigidbodies[indices[start]].collider.GetBounds();
            for (int i = start + 1; i < end; i++)
            {
                combinedBounds = SBounds.Union(combinedBounds, _rigidbodies[indices[i]].collider.GetBounds());
            }
            node.Bounds = combinedBounds;

            // Find the axis to split along
            int axis = combinedBounds.MaximumExtent();

            // Select the median index to split at
            int mid = (start + end) / 2;

            // Find the median element and use it as a pivot
            QuickSelect(indices, start, end - 1, mid, axis);

            // Recursively build the tree
            node.Left = BuildBVH(indices, start, mid);
            node.Right = BuildBVH(indices, mid, end);

            return node;
        }

        private void QuickSelect(int[] indices, int left, int right, int k, int axis)
        {
            while (left < right)
            {
                int pivotIndex = Partition(indices, left, right, axis);
                if (pivotIndex == k)
                    return;
                else if (pivotIndex < k)
                    left = pivotIndex + 1;
                else
                    right = pivotIndex - 1;
            }
        }

        private int Partition(int[] indices, int left, int right, int axis)
        {
            f32 pivot = GetAxisValue(_rigidbodies[indices[right]].collider.GetBounds().Center, axis);
            int i = left;
            for (int j = left; j < right; j++)
            {
                if (GetAxisValue(_rigidbodies[indices[j]].collider.GetBounds().Center, axis).CompareTo(pivot) < 0)
                {
                    Swap(indices, i, j);
                    i++;
                }
            }
            Swap(indices, i, right);
            return i;
        }

        private void Swap(int[] indices, int i, int j)
        {
            int temp = indices[i];
            indices[i] = indices[j];
            indices[j] = temp;
        }

        private f32 GetAxisValue(Vector3S vector, int axis)
        {
            return axis switch
            {
                0 => vector.x,
                1 => vector.y,
                _ => vector.z,
            };
        }

        // Refit method to update the BVH nodes
        public void Refit()
        {
            RefitNode(Root);
        }

        private void RefitNode(BVHNode node)
        {
            if (node == null)
            {
                return;
            }

            if (node.IsLeaf())
            {
                node.Bounds = _rigidbodies[node.ObjectIndex].collider.GetBounds();
            }
            else
            {
                RefitNode(node.Left);
                RefitNode(node.Right);

                node.Bounds = node.Left != null ? node.Left.Bounds : default;
                if (node.Right != null)
                {
                    node.Bounds = SBounds.Union(node.Bounds, node.Right.Bounds);
                }
            }
        }

        public void Update(List<SRigidbody> rigidbodies)
        {
            _frameCount++;

            if (_frameCount >= RebuildThreshold)
            {
                Rebuild(rigidbodies);
                _frameCount = 0;
            }
            else
            {
                Refit();
            }
        }

        // Query method to find all objects intersecting with a given bounds
        public List<int> Query(SBounds queryBounds)
        {
            List<int> result = new List<int>(_rigidbodies.Count);
            QueryNode(Root, queryBounds, result);
            return result;
        }

        private void QueryNode(BVHNode node, SBounds queryBounds, List<int> result)
        {
            if (node == null)
            {
                return;
            }

            if (node.Bounds.Intersects(queryBounds))
            {
                if (node.IsLeaf())
                {
                    result.Add(node.ObjectIndex);
                }
                else
                {
                    QueryNode(node.Left, queryBounds, result);
                    QueryNode(node.Right, queryBounds, result);
                }
            }
        }

        // Traverse the BVH and collect bounds into a preallocated array
        public SBounds[] CollectBounds(ref SBounds[] boundsArray)
        {
            int nodeCount = CountNodes(Root);
            if (boundsArray.Length < nodeCount)
            {
                Array.Resize(ref boundsArray, nodeCount);
            }

            int index = 0;
            CollectBoundsRecursive(Root, boundsArray, ref index);
            return boundsArray;
        }

        private void CollectBoundsRecursive(BVHNode node, SBounds[] boundsArray, ref int index)
        {
            if (node == null) return;
            if (index >= boundsArray.Length) return;

            boundsArray[index++] = node.Bounds;

            if (node.Left != null)
            {
                CollectBoundsRecursive(node.Left, boundsArray, ref index);
            }

            if (node.Right != null)
            {
                CollectBoundsRecursive(node.Right, boundsArray, ref index);
            }
        }

        private int CountNodes(BVHNode node)
        {
            if (node == null) return 0;
            return 1 + CountNodes(node.Left) + CountNodes(node.Right);
        }
    }
}