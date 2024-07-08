using stupid.Maths;

namespace stupid.Colliders
{
    public class AABBNode
    {
        public AABBNode Left { get; set; }
        public AABBNode Right { get; set; }
        public SRigidbody Data { get; set; }
        public SBounds Box { get; set; }
        public SBounds ObjectBounds => Data?.collider.GetBounds() ?? Box;

        public AABBNode(SRigidbody data)
        {
            Data = data;
            Box = ObjectBounds;
            Left = null;
            Right = null;
        }

        public bool IsLeaf => Left == null && Right == null;
    }
}
