using SoftFloat;

namespace stupid
{
    public struct BodyPair
    {
        public int aIndex;
        public int bIndex;

        public BodyPair(int bodyA, int bodyB)
        {
            if (bodyA < bodyB)
            {
                aIndex = bodyA;
                bIndex = bodyB;
            }
            else
            {
                aIndex = bodyB;
                bIndex = bodyA;
            }
        }

        public override bool Equals(object obj)
        {
            if (obj is BodyPair other)
            {
                return aIndex == other.aIndex && bIndex == other.bIndex;
            }
            return false;
        }

        public override int GetHashCode()
        {
            return aIndex.GetHashCode() ^ bIndex.GetHashCode();
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
