using SoftFloat;

namespace stupid
{
    public class Rigidbody
    {
        public readonly int index;

        public Vector3S position;

        public Vector3S velocity;

        public sfloat mass = sfloat.one;

        public bool useGravity = true;

        public bool isKinematic = false;

        public ICollider collider;

        public Rigidbody(int index, Vector3S position = default, Vector3S velocity = default)
        {
            this.index = index;
            this.position = position;
            this.velocity = velocity;
        }

        public void Attach(ICollider collider)
        {
            this.collider = collider;
        }
    }
}