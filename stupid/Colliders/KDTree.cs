using System;
using System.Collections.Generic;
using SoftFloat;
using stupid.Maths;

namespace stupid.Colliders
{
    public class KDTreeNode
    {
        public SBounds Bounds;
        public KDTreeNode Left;
        public KDTreeNode Right;
        public int ObjectIndex;  // Index of the object in the leaf node, -1 for internal nodes

        public KDTreeNode()
        {
            ObjectIndex = -1;
        }
    }

    public class KDTree
    {
        public KDTreeNode Root;
        private readonly List<SRigidbody> _rigidbodies;
        private Stack<KDTreeNode> _nodePool;
        private int _nodeCount;
        private Dictionary<int, Vector3S> _previousPositions;

        public KDTree(List<SRigidbody> rigidbodies)
        {
            _rigidbodies = rigidbodies;
            _nodePool = new Stack<KDTreeNode>(rigidbodies.Count * 2);
            _previousPositions = new Dictionary<int, Vector3S>(rigidbodies.Count);
            for (int i = 0; i < rigidbodies.Count * 2; i++)
            {
                _nodePool.Push(new KDTreeNode());
            }
            InitializePreviousPositions();
            Rebuild();
        }

        private void InitializePreviousPositions()
        {
            foreach (var rigidbody in _rigidbodies)
            {
                _previousPositions[rigidbody.index] = rigidbody.position;
            }
        }

        public void Rebuild()
        {
            int[] indices = new int[_rigidbodies.Count];
            for (int i = 0; i < indices.Length; i++)
            {
                indices[i] = i;
            }
            _nodeCount = 0;
            Root = BuildKDTree(indices, 0, indices.Length, 0);
        }

        private KDTreeNode BuildKDTree(int[] indices, int start, int end, int depth)
        {
            int count = end - start;
            if (count == 0)
            {
                return null;
            }

            KDTreeNode node = _nodePool.Count > 0 ? _nodePool.Pop() : new KDTreeNode();
            _nodeCount++;

            if (count == 1)
            {
                int index = indices[start];
                node.Bounds = _rigidbodies[index].collider.GetBounds();
                node.ObjectIndex = _rigidbodies[index].index;
                return node;
            }

            // Determine axis to split on
            int axis = depth % 3;

            // Select the median index to split at
            int mid = (start + end) / 2;

            // Find the median element and use it as a pivot
            QuickSelect(indices, start, end - 1, mid, axis);

            // Compute the bounding box that encompasses all objects
            SBounds combinedBounds = _rigidbodies[indices[start]].collider.GetBounds();
            for (int i = start + 1; i < end; i++)
            {
                combinedBounds = SBounds.Union(combinedBounds, _rigidbodies[indices[i]].collider.GetBounds());
            }
            node.Bounds = combinedBounds;

            // Recursively build the tree
            node.Left = BuildKDTree(indices, start, mid, depth + 1);
            node.Right = BuildKDTree(indices, mid, end, depth + 1);

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
            sfloat pivot = GetAxisValue(_rigidbodies[indices[right]].collider.GetBounds().Center, axis);
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

        private sfloat GetAxisValue(Vector3S vector, int axis)
        {
            return axis switch
            {
                0 => vector.x,
                1 => vector.y,
                _ => vector.z,
            };
        }

        public void Update()
        {
            foreach (var rigidbody in _rigidbodies)
            {
                if (HasMovedSignificantly(rigidbody))
                {
                    Remove(rigidbody.index);
                    Insert(rigidbody.index);
                    _previousPositions[rigidbody.index] = rigidbody.position;
                }
            }
        }

        private bool HasMovedSignificantly(SRigidbody rigidbody)
        {
            var previousPosition = _previousPositions[rigidbody.index];
            var currentPosition = rigidbody.position;
            sfloat threshold = (sfloat)0.1f; // Define a threshold for significant movement
            return (currentPosition - previousPosition).MagnitudeSquared() > threshold * threshold;
        }

        private void Remove(int index)
        {
            Root = Remove(Root, index);
        }

        private KDTreeNode Remove(KDTreeNode node, int index)
        {
            if (node == null)
            {
                return null;
            }

            if (node.ObjectIndex == index)
            {
                _nodePool.Push(node);
                return null;
            }

            node.Left = Remove(node.Left, index);
            node.Right = Remove(node.Right, index);

            return node;
        }

        private void Insert(int index)
        {
            Root = Insert(Root, index, 0);
        }

        private KDTreeNode Insert(KDTreeNode node, int index, int depth)
        {
            if (node == null)
            {
                KDTreeNode newNode = _nodePool.Count > 0 ? _nodePool.Pop() : new KDTreeNode();
                newNode.Bounds = _rigidbodies[index].collider.GetBounds();
                newNode.ObjectIndex = _rigidbodies[index].index;
                return newNode;
            }

            int axis = depth % 3;
            sfloat axisValue = GetAxisValue(_rigidbodies[index].collider.GetBounds().Center, axis);

            if (axisValue.CompareTo(GetAxisValue(node.Bounds.Center, axis)) < 0)
            {
                node.Left = Insert(node.Left, index, depth + 1);
            }
            else
            {
                node.Right = Insert(node.Right, index, depth + 1);
            }

            node.Bounds = SBounds.Union(node.Bounds, _rigidbodies[index].collider.GetBounds());
            return node;
        }

        // Query method to find all objects intersecting with a given bounds
        public List<int> Query(SBounds queryBounds)
        {
            List<int> result = new List<int>(_rigidbodies.Count);
            QueryNode(Root, queryBounds, result);
            return result;
        }

        private void QueryNode(KDTreeNode node, SBounds queryBounds, List<int> result)
        {
            if (node == null)
            {
                return;
            }

            if (node.Bounds.Intersects(queryBounds))
            {
                if (node.ObjectIndex != -1)
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

        // Traverse the KD-tree and collect bounds into a preallocated array
        public SBounds[] CollectBounds(ref SBounds[] boundsArray)
        {
            if (boundsArray.Length < _nodeCount)
            {
                Array.Resize(ref boundsArray, _nodeCount);
            }

            int index = 0;
            CollectBoundsRecursive(Root, boundsArray, ref index);
            return boundsArray;
        }

        private void CollectBoundsRecursive(KDTreeNode node, SBounds[] boundsArray, ref int index)
        {
            if (node == null) return;

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

        // Method to release KD-tree nodes back to the pool for reuse
        public void ReleaseNodes()
        {
            ReleaseNodeRecursive(Root);
            Root = null;
        }

        private void ReleaseNodeRecursive(KDTreeNode node)
        {
            if (node == null) return;

            if (node.Left != null)
            {
                ReleaseNodeRecursive(node.Left);
            }

            if (node.Right != null)
            {
                ReleaseNodeRecursive(node.Right);
            }

            _nodePool.Push(node);
        }
    }
}
