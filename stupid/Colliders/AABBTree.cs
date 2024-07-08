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
                Root = Build(bodies);
            }
        }

        private AABBNode Build(List<SRigidbody> bodies)
        {
            if (bodies.Count == 1)
            {
                return new AABBNode(bodies[0]);
            }

            SBounds overallBox = CalculateOverallBoundingBox(bodies);
            int axis = overallBox.MaximumExtent();
            bodies.Sort((a, b) => CompareByAxis(a.collider.GetBounds(), b.collider.GetBounds(), axis));

            int mid = bodies.Count / 2;
            List<SRigidbody> leftBodies = bodies.GetRange(0, mid);
            List<SRigidbody> rightBodies = bodies.GetRange(mid, bodies.Count - mid);

            var leftNode = Build(leftBodies);
            var rightNode = Build(rightBodies);

            return new AABBNode(null)
            {
                Left = leftNode,
                Right = rightNode,
                Box = SBounds.Union(leftNode.Box, rightNode.Box)
            };
        }

        private int CompareByAxis(SBounds a, SBounds b, int axis)
        {
            return axis switch
            {
                0 => a.min.x.CompareTo(b.min.x),
                1 => a.min.y.CompareTo(b.min.y),
                2 => a.min.z.CompareTo(b.min.z),
                _ => throw new ArgumentException("Invalid axis"),
            };
        }

        private SBounds CalculateOverallBoundingBox(List<SRigidbody> bodies)
        {
            Vector3S min = new Vector3S(f32.maxValue, f32.maxValue, f32.maxValue);
            Vector3S max = new Vector3S(f32.minValue, f32.minValue, f32.minValue);

            foreach (var body in bodies)
            {
                SBounds bounds = body.collider.GetBounds();
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

                if (CompareByAxis(oldLeaf.Box, newLeaf.Box, combinedBox.MaximumExtent()) < 0)
                {
                    node.Left = oldLeaf;
                    node.Right = newLeaf;
                }
                else
                {
                    node.Left = newLeaf;
                    node.Right = oldLeaf;
                }

                node.Box = combinedBox;
            }
            else
            {
                if (SBounds.Union(node.Left.Box, body.collider.GetBounds()).Size.SqrMagnitude < SBounds.Union(node.Right.Box, body.collider.GetBounds()).Size.SqrMagnitude)
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
                node.Box = SBounds.Union(node.Left.Box, node.Right.Box);
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
    }
}
