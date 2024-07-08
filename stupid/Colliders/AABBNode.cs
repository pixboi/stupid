using stupid.Colliders;
using System;

namespace stupid
{
    public class AABBNode
    {
        public AABBNode Parent { get; set; }
        public AABBNode Left { get; set; }
        public AABBNode Right { get; set; }
        public SRigidbody Body { get; set; }
        public SBounds Bounds { get; set; }
        public bool IsLeaf => Body != null;

        public AABBNode(SRigidbody body)
        {
            Body = body;
            Bounds = body.GetBounds();
        }

        public AABBNode(SBounds bounds)
        {
            Bounds = bounds;
        }
    }
}