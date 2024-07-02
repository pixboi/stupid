using SoftFloat;

namespace stupid
{
    public struct BodyPair
    {
        public int BodyA;
        public int BodyB;

        public BodyPair(int bodyA, int bodyB)
        {
            if (bodyA < bodyB)
            {
                BodyA = bodyA;
                BodyB = bodyB;
            }
            else
            {
                BodyA = bodyB;
                BodyB = bodyA;
            }
        }

        public override bool Equals(object obj)
        {
            if (obj is BodyPair other)
            {
                return BodyA == other.BodyA && BodyB == other.BodyB;
            }
            return false;
        }

        public override int GetHashCode()
        {
            return BodyA.GetHashCode() ^ BodyB.GetHashCode();
        }
    }

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

        public ContactPair(Rigidbody a, Rigidbody b)
        {
            if (a.index < b.index)
            {
                bodyA = a;
                bodyB = b;
            }
            else
            {
                bodyA = b;
                bodyB = a;
            }
        }

        public override bool Equals(object obj)
        {
            if (obj is ContactPair other)
            {
                return bodyA == other.bodyA && bodyB == other.bodyB;
            }
            return false;
        }

        public override int GetHashCode()
        {
            return bodyA.GetHashCode() ^ bodyB.GetHashCode();
        }
    }
}
