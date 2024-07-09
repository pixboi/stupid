using stupid.Colliders;
using System;

namespace stupid.Trees
{
    public class AABBNode
    {
        public AABBNode Parent { get; set; }
        public AABBNode Left { get; set; }
        public AABBNode Right { get; set; }
        public RigidbodyS Body { get; set; }
        public BoundsS Bounds { get; set; }
        public bool IsLeaf => Body != null;

        public AABBNode(RigidbodyS body)
        {
            Body = body;
            Bounds = body.GetBounds();
        }

        public AABBNode(BoundsS bounds)
        {
            Bounds = bounds;
        }
    }
}