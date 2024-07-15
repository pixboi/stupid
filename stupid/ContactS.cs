
using stupid.Maths;
using System;
using System.Collections.Generic;

namespace stupid
{
    public readonly struct ContactManifoldS : IEquatable<ContactManifoldS>
    {
        public readonly Collidable a;
        public readonly Collidable b;
        public readonly ContactS[] contacts;
        public readonly int count;

        public ContactManifoldS(Collidable a, Collidable b, ContactS[] contact, int count)
        {
            this.a = a;
            this.b = b;
            this.contacts = contact;
            this.count = count;
        }

        public override bool Equals(object? obj)
        {
            return obj is ContactManifoldS s && Equals(s);
        }

        public bool Equals(ContactManifoldS other)
        {
            return EqualityComparer<Collidable>.Default.Equals(a, other.a) &&
                   EqualityComparer<Collidable>.Default.Equals(b, other.b);
        }

        public override int GetHashCode()
        {
            return HashCode.Combine(a, b);
        }
    }

    public struct ContactS
    {
        public Vector3S point;
        public Vector3S normal;
        public f32 penetrationDepth;

        // Cached impulses for warm starting
        public Vector3S cachedImpulse;
        public f32 cachedNormalImpulse;
        public f32 cachedFrictionImpulse;

        public ContactS(Vector3S point, Vector3S normal, f32 penetrationDepth)
        {
            this.point = point;
            this.normal = normal;
            this.penetrationDepth = penetrationDepth;
            this.cachedImpulse = Vector3S.zero;
            this.cachedNormalImpulse = f32.zero;
            this.cachedFrictionImpulse = f32.zero;
        }
    }

}
