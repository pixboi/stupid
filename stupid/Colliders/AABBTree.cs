using stupid.Colliders;
using System.Collections.Generic;

namespace stupid
{
    public class AABBTree
    {
        public AABBNode Root { get; private set; }

        public void Insert(SRigidbody body)
        {
            var node = new AABBNode(body);
            if (Root == null)
            {
                Root = node;
            }
            else
            {
                InsertNode(Root, node);
            }
        }

        private void InsertNode(AABBNode root, AABBNode node)
        {
            if (root.IsLeaf)
            {
                var newParent = new AABBNode(SBounds.Union(root.Bounds, node.Bounds))
                {
                    Left = root,
                    Right = node
                };

                root.Parent = newParent;
                node.Parent = newParent;

                if (root == this.Root)
                {
                    this.Root = newParent;
                }
            }
            else
            {
                var leftUnion = SBounds.Union(root.Left.Bounds, node.Bounds);
                var rightUnion = SBounds.Union(root.Right.Bounds, node.Bounds);

                var leftGrowth = leftUnion.Size.SqrMagnitude - root.Left.Bounds.Size.SqrMagnitude;
                var rightGrowth = rightUnion.Size.SqrMagnitude - root.Right.Bounds.Size.SqrMagnitude;

                if (leftGrowth < rightGrowth)
                {
                    InsertNode(root.Left, node);
                }
                else
                {
                    InsertNode(root.Right, node);
                }

                root.Bounds = SBounds.Union(root.Left.Bounds, root.Right.Bounds);
            }
        }

        public void Remove(SRigidbody body)
        {
            var node = FindNode(Root, body);
            if (node == null) return;

            RemoveNode(node);
        }

        private void RemoveNode(AABBNode node)
        {
            if (node.Parent == null)
            {
                Root = null;
                return;
            }

            var parent = node.Parent;
            var sibling = parent.Left == node ? parent.Right : parent.Left;

            if (parent.Parent == null)
            {
                Root = sibling;
                sibling.Parent = null;
            }
            else
            {
                var grandparent = parent.Parent;
                if (grandparent.Left == parent)
                {
                    grandparent.Left = sibling;
                }
                else
                {
                    grandparent.Right = sibling;
                }

                sibling.Parent = grandparent;

                UpdateBoundsUpwards(grandparent);
            }
        }

        private void UpdateBoundsUpwards(AABBNode node)
        {
            while (node != null)
            {
                if (node.Left != null && node.Right != null)
                {
                    node.Bounds = SBounds.Union(node.Left.Bounds, node.Right.Bounds);
                }
                node = node.Parent;
            }
        }

        private AABBNode FindNode(AABBNode root, SRigidbody body)
        {
            if (root == null) return null;

            if (root.IsLeaf)
            {
                return root.Body == body ? root : null;
            }

            var leftResult = FindNode(root.Left, body);
            if (leftResult != null) return leftResult;

            return FindNode(root.Right, body);
        }
    }
}
