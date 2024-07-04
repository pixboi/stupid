using System;
using System.Collections.Generic;
using SoftFloat;
using stupid.Maths;

namespace stupid.Colliders
{
    public class BVHNode
    {
        public SBounds Bounds;
        public BVHNode Left;
        public BVHNode Right;
        public int ObjectIndex;  // Index of the object in the leaf node, -1 for internal nodes

        public BVHNode()
        {
            ObjectIndex = -1;
        }
    }

    public class BVH
    {
        public BVHNode Root;
        private List<SRigidbody> _rigidbodies;
        private int _nodeCount;

        public BVH(List<SRigidbody> rigidbodies)
        {
            _rigidbodies = rigidbodies;
            Rebuild();
        }

        public void Rebuild()
        {
            int[] objectIndices = new int[_rigidbodies.Count];
            for (int i = 0; i < _rigidbodies.Count; i++)
            {
                objectIndices[i] = i;
            }
            _nodeCount = 0;
            Root = BuildBVH(_rigidbodies, objectIndices, 0, _rigidbodies.Count);
        }

        private BVHNode BuildBVH(List<SRigidbody> rigidbodies, int[] objectIndices, int start, int end)
        {
            int count = end - start;
            if (count == 0)
            {
                return null;
            }

            BVHNode node = new BVHNode();
            _nodeCount++;

            if (count == 1)
            {
                int index = objectIndices[start];
                node.Bounds = rigidbodies[index].collider.GetBounds();
                node.ObjectIndex = rigidbodies[index].index;
                return node;
            }

            // Compute the bounding box that encompasses all objects
            SBounds combinedBounds = rigidbodies[objectIndices[start]].collider.GetBounds();
            for (int i = start + 1; i < end; i++)
            {
                combinedBounds = SBounds.Union(combinedBounds, rigidbodies[objectIndices[i]].collider.GetBounds());
            }
            node.Bounds = combinedBounds;

            // Find the axis to split along
            int axis = combinedBounds.MaximumExtent();

            // Sort objects along the chosen axis using custom quicksort
            QuickSort(rigidbodies, objectIndices, start, end - 1, axis);

            // Split the list into two halves
            int mid = start + count / 2;

            node.Left = BuildBVH(rigidbodies, objectIndices, start, mid);
            node.Right = BuildBVH(rigidbodies, objectIndices, mid, end);

            return node;
        }

        private void QuickSort(List<SRigidbody> rigidbodies, int[] indices, int low, int high, int axis)
        {
            if (low < high)
            {
                int pi = Partition(rigidbodies, indices, low, high, axis);
                QuickSort(rigidbodies, indices, low, pi - 1, axis);
                QuickSort(rigidbodies, indices, pi + 1, high, axis);
            }
        }

        private int Partition(List<SRigidbody> rigidbodies, int[] indices, int low, int high, int axis)
        {
            sfloat pivot = GetAxisValue(rigidbodies[indices[high]].collider.GetBounds().Center, axis);
            int i = low - 1;

            for (int j = low; j < high; j++)
            {
                sfloat centerValue = GetAxisValue(rigidbodies[indices[j]].collider.GetBounds().Center, axis);
                if (centerValue.CompareTo(pivot) < 0)
                {
                    i++;
                    Swap(indices, i, j);
                }
            }

            Swap(indices, i + 1, high);
            return i + 1;
        }

        private void Swap(int[] array, int i, int j)
        {
            int temp = array[i];
            array[i] = array[j];
            array[j] = temp;
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

        // Example query method to find all objects intersecting with a given bounds
        public List<int> Query(SBounds queryBounds)
        {
            List<int> result = new List<int>();
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

        // Traverse the BVH and collect bounds into a preallocated array
        public SBounds[] CollectBounds(ref SBounds[] boundsArray)
        {
            int index = 0;
            CollectBoundsRecursive(Root, boundsArray, ref index);
            return boundsArray;
        }

        private void CollectBoundsRecursive(BVHNode node, SBounds[] boundsArray, ref int index)
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
    }
}
