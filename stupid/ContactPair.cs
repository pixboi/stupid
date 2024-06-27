using SoftFloat;

namespace stupid
{
    public struct Contact
    {
        public Vector3S point;
        public Vector3S normal;
        public sfloat penetrationDepth;
    }

    public struct ContactPair
    {
        public Rigidbody bodyA;
        public Rigidbody bodyB;
        public Contact contact;
    }
}
