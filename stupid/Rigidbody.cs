using SoftFloat;

namespace stupid
{
    public class Rigidbody
    {
        //Runtime
        public readonly int index;
        public override int GetHashCode()
        {
            return index;
        }

        public Vector3S position;

        public Vector3S velocity;

        public ICollider collider;
        //Settings

        public sfloat mass = sfloat.one;

        public bool useGravity = true;

        public bool isKinematic = false;

        public readonly sfloat sleepThreshold = (sfloat)0.1f; // non calced values can be reg floats?
        public bool isSleeping { get; private set; }
        public void WakeUp() => isSleeping = false;
        public void Sleep()
        {
            isSleeping = true;
            this.velocity = Vector3S.zero;
        }

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