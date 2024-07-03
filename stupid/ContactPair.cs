﻿using SoftFloat;
using stupid.Maths;

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
        public SRigidbody bodyA;
        public SRigidbody bodyB;

        public ContactPair(SRigidbody a, SRigidbody b)
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
