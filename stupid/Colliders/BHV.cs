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

        public BVH(List<SRigidbody> rigidbodies)
        {
            _rigidbodies = rigidbodies;
            Rebuild();
        }

        public void Rebuild()
        {
            List<int> objectIndices = new List<int>();
            for (int i = 0; i < _rigidbodies.Count; i++)
            {
                objectIndices.Add(i);
            }
            Root = BuildBVH(_rigidbodies, objectIndices);
        }

        private BVHNode BuildBVH(List<SRigidbody> rigidbodies, List<int> objectIndices)
        {
            if (objectIndices.Count == 0)
            {
                return null;
            }

            BVHNode node = new BVHNode();

            if (objectIndices.Count == 1)
            {
                int index = objectIndices[0];
                node.Bounds = rigidbodies[index].collider.GetBounds();
                node.ObjectIndex = rigidbodies[index].index;
                return node;
            }

            // Compute the bounding box that encompasses all objects
            SBounds combinedBounds = rigidbodies[objectIndices[0]].collider.GetBounds();
            foreach (int index in objectIndices)
            {
                combinedBounds = SBounds.Union(combinedBounds, rigidbodies[index].collider.GetBounds());
            }
            node.Bounds = combinedBounds;

            // Find the axis to split along
            int axis = combinedBounds.MaximumExtent();

            // Sort objects along the chosen axis
            objectIndices.Sort((a, b) =>
            {
                sfloat centerA = GetAxisValue(rigidbodies[a].collider.GetBounds().Center, axis);
                sfloat centerB = GetAxisValue(rigidbodies[b].collider.GetBounds().Center, axis);
                return centerA.CompareTo(centerB);
            });

            // Split the list into two halves
            int mid = objectIndices.Count / 2;
            List<int> leftIndices = objectIndices.GetRange(0, mid);
            List<int> rightIndices = objectIndices.GetRange(mid, objectIndices.Count - mid);

            node.Left = BuildBVH(rigidbodies, leftIndices);
            node.Right = BuildBVH(rigidbodies, rightIndices);

            return node;
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

        // Traverse the BVH and collect bounds
        public List<SBounds> CollectBounds()
        {
            List<SBounds> boundsList = new List<SBounds>();
            Queue<BVHNode> queue = new Queue<BVHNode>();
            queue.Enqueue(Root);

            while (queue.Count > 0)
            {
                BVHNode currentNode = queue.Dequeue();
                if (currentNode == null)
                {
                    continue;
                }

                boundsList.Add(currentNode.Bounds);

                if (currentNode.Left != null)
                {
                    queue.Enqueue(currentNode.Left);
                }

                if (currentNode.Right != null)
                {
                    queue.Enqueue(currentNode.Right);
                }
            }

            return boundsList;
        }
    }
}
