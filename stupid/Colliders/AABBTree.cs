using stupid.Maths;
using System;
using System.Collections.Generic;

namespace stupid.Colliders
{
    public class AABBTree
    {
        public AABBNode Root { get; private set; }
        public AABBTree(List<SRigidbody> bodies)
        {
            if (bodies.Count > 0)
            {
                Root = Build(bodies, 0, bodies.Count);
            }
        }

        private AABBNode Build(List<SRigidbody> bodies, int start, int count)
        {
            if (count == 1)
            {
                return new AABBNode(bodies[start]);
            }

            SBounds overallBox = CalculateOverallBoundingBox(bodies, start, count);
            int axis = overallBox.MaximumExtent();
            bodies.Sort(start, count, new AxisComparer(axis));

            int mid = start + count / 2;
            var leftNode = Build(bodies, start, mid - start);
            var rightNode = Build(bodies, mid, start + count - mid);

            return new AABBNode(null)
            {
                Left = leftNode,
                Right = rightNode,
                Box = SBounds.Union(leftNode.Box, rightNode.Box)
            };
        }

        private SBounds CalculateOverallBoundingBox(List<SRigidbody> bodies, int start, int count)
        {
            Vector3S min = new Vector3S(f32.maxValue, f32.maxValue, f32.maxValue);
            Vector3S max = new Vector3S(f32.minValue, f32.minValue, f32.minValue);

            for (int i = start; i < start + count; i++)
            {
                SBounds bounds = bodies[i].collider.GetBounds();
                min = Vector3S.Min(min, bounds.min);
                max = Vector3S.Max(max, bounds.max);
            }

            return new SBounds(min, max);
        }

        public void Insert(SRigidbody body)
        {
            Root = Insert(Root, body);
            RefitAndBalance();
        }

        private AABBNode Insert(AABBNode node, SRigidbody body)
        {
            if (node == null) return new AABBNode(body);

            if (node.IsLeaf)
            {
                var oldLeaf = new AABBNode(node.Data);
                var newLeaf = new AABBNode(body);

                var combinedBox = SBounds.Union(oldLeaf.Box, newLeaf.Box);
                node.Data = null;

                node.Left = oldLeaf;
                node.Right = newLeaf;
                node.Box = combinedBox;
            }
            else
            {
                SBounds bodyBounds = body.collider.GetBounds();
                f32 leftUnionSize = SBounds.Union(node.Left.Box, bodyBounds).Size.SqrMagnitude;
                f32 rightUnionSize = SBounds.Union(node.Right.Box, bodyBounds).Size.SqrMagnitude;

                if (leftUnionSize < rightUnionSize)
                {
                    node.Left = Insert(node.Left, body);
                }
                else
                {
                    node.Right = Insert(node.Right, body);
                }

                node.Box = SBounds.Union(node.Left.Box, node.Right.Box);
            }

            return node;
        }

        public void Remove(SRigidbody body)
        {
            Root = Remove(Root, body.collider.GetBounds(), body);
            RefitAndBalance();
        }

        private AABBNode Remove(AABBNode node, SBounds box, SRigidbody body)
        {
            if (node == null) return null;

            if (node.Data == body && node.Box.Equals(box))
            {
                if (node.IsLeaf) return null;

                if (node.Left != null && node.Right != null)
                {
                    var successor = GetSuccessor(node.Right);
                    node.Data = successor.Data;
                    node.Right = Remove(node.Right, successor.Box, successor.Data);
                    node.Box = SBounds.Union(node.Left.Box, node.Right.Box);
                }
                else
                {
                    return node.Left ?? node.Right;
                }
            }
            else
            {
                node.Left = Remove(node.Left, box, body);
                node.Right = Remove(node.Right, box, body);
                node.Box = SBounds.Union(node.Left.Box, node.Right.Box);
            }

            return node;
        }

        private AABBNode GetSuccessor(AABBNode node)
        {
            while (node.Left != null)
            {
                node = node.Left;
            }
            return node;
        }

        public void RefitAndBalance()
        {
            Refit(Root);
            Root = BalanceTree(Root);
        }

        private void Refit(AABBNode node)
        {
            if (node == null) return;

            if (node.IsLeaf)
            {
                node.Box = node.ObjectBounds;
            }
            else
            {
                Refit(node.Left);
                Refit(node.Right);
                node.Box = SBounds.Union(node.Left?.Box ?? node.ObjectBounds, node.Right?.Box ?? node.ObjectBounds);
            }
        }

        private AABBNode RotateLeft(AABBNode node)
        {
            var newRoot = node.Right;
            node.Right = newRoot.Left;
            newRoot.Left = node;

            node.Box = SBounds.Union(node.Left?.Box ?? node.ObjectBounds, node.Right?.Box ?? node.ObjectBounds);
            newRoot.Box = SBounds.Union(newRoot.Left?.Box ?? newRoot.ObjectBounds, newRoot.Right?.Box ?? newRoot.ObjectBounds);

            return newRoot;
        }

        private AABBNode RotateRight(AABBNode node)
        {
            var newRoot = node.Left;
            node.Left = newRoot.Right;
            newRoot.Right = node;

            node.Box = SBounds.Union(node.Left?.Box ?? node.ObjectBounds, node.Right?.Box ?? node.ObjectBounds);
            newRoot.Box = SBounds.Union(newRoot.Left?.Box ?? newRoot.ObjectBounds, newRoot.Right?.Box ?? newRoot.ObjectBounds);

            return newRoot;
        }

        private AABBNode Balance(AABBNode node)
        {
            if (node == null) return null;

            int balanceFactor = GetHeight(node.Left) - GetHeight(node.Right);

            if (balanceFactor > 1)
            {
                if (GetHeight(node.Left.Left) < GetHeight(node.Left.Right))
                {
                    node.Left = RotateLeft(node.Left);
                }
                return RotateRight(node);
            }
            if (balanceFactor < -1)
            {
                if (GetHeight(node.Right.Left) > GetHeight(node.Right.Right))
                {
                    node.Right = RotateRight(node.Right);
                }
                return RotateLeft(node);
            }

            return node;
        }

        private int GetHeight(AABBNode node)
        {
            if (node == null) return 0;
            return Math.Max(GetHeight(node.Left), GetHeight(node.Right)) + 1;
        }

        private AABBNode BalanceTree(AABBNode node)
        {
            if (node == null) return null;

            node.Left = BalanceTree(node.Left);
            node.Right = BalanceTree(node.Right);

            return Balance(node);
        }

        private class AxisComparer : IComparer<SRigidbody>
        {
            private readonly int axis;

            public AxisComparer(int axis)
            {
                this.axis = axis;
            }

            public int Compare(SRigidbody a, SRigidbody b)
            {
                SBounds boundsA = a.collider.GetBounds();
                SBounds boundsB = b.collider.GetBounds();

                return axis switch
                {
                    0 => boundsA.min.x.CompareTo(boundsB.min.x),
                    1 => boundsA.min.y.CompareTo(boundsB.min.y),
                    2 => boundsA.min.z.CompareTo(boundsB.min.z),
                    _ => throw new ArgumentException("Invalid axis"),
                };
            }
        }

        public void Rebuild(List<SRigidbody> bodies)
        {
            Root = Build(bodies, 0, bodies.Count);
        }

        private void CollectBodies(AABBNode node, List<SRigidbody> bodies)
        {
            if (node == null) return;

            if (node.IsLeaf)
            {
                bodies.Add(node.Data);
            }
            else
            {
                CollectBodies(node.Left, bodies);
                CollectBodies(node.Right, bodies);
            }
        }

        public List<SRigidbody> QueryRay(Ray ray)
        {
            List<SRigidbody> result = new List<SRigidbody>();
            if (Root != null && Root.Box.IntersectRay(ray))
            {
                Stack<AABBNode> stack = new Stack<AABBNode>();
                stack.Push(Root);

                while (stack.Count > 0)
                {
                    AABBNode node = stack.Pop();

                    if (node.Box.IntersectRay(ray))
                    {
                        if (node.IsLeaf)
                        {
                            if (node.Data.collider.GetBounds().IntersectRay(ray))
                            {
                                result.Add(node.Data);
                            }
                        }
                        else
                        {
                            if (node.Left != null && node.Left.Box.IntersectRay(ray))
                            {
                                stack.Push(node.Left);
                            }
                            if (node.Right != null && node.Right.Box.IntersectRay(ray))
                            {
                                stack.Push(node.Right);
                            }
                        }
                    }
                }
            }
            return result;
        }
    }
}
