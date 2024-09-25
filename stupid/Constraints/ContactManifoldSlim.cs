using stupid;
using stupid.Broadphase;
using stupid.Maths;
using System.Runtime;
using System.Runtime.CompilerServices;

namespace stupid.Constraints
{
    public readonly struct ContactManifoldSlim
    {
        public readonly RigidbodyS a, b; // 16
        public readonly Vector3S normal; // 24 
        public readonly f32 penetrationDepth, friction; // 16
        public readonly int startIndex, contactCount; // 8

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ContactManifoldSlim(RigidbodyS a, Collidable b, in Vector3S normal, in f32 penetrationDepth, in WorldSettings settings, in f32 inverseDt, int startIndex = -1, int contactCount = -1)
        {
            if (contactCount < 1) throw new System.ArgumentException("ZERO CONTACTS?");

            this.a = a;
            this.b = b.isDynamic ? (RigidbodyS)b : null;
            this.normal = normal;
            this.penetrationDepth = penetrationDepth;
            this.startIndex = startIndex;
            this.contactCount = contactCount;
            this.friction = MathS.Sqrt(a.material.staticFriction * b.material.staticFriction);
        }

        // Prestep calculations for all contacts
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void CalculatePrestep(ref ContactSlim[] contacts)
        {
            for (int i = startIndex; i < startIndex + contactCount; i++)
            {
                ref var c = ref contacts[i];
                c.CalculatePrestep(a, b, this);
            }
        }

        // Warmup for iterative solvers
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Warmup(ref ContactSlim[] contacts)
        {
            for (int i = startIndex; i < startIndex + contactCount; i++)
            {
                contacts[i].WarmStart(a, b, this.normal);
            }
        }

        // Retain impulse data from the old manifold
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void RetainData(ref ContactSlim[] contacts, in ContactSlim[] oldContacts, in ContactManifoldSlim old)
        {
            for (int i = this.startIndex; i < this.startIndex + this.contactCount; i++)
            {
                ref var c = ref contacts[i];

                // Reduce loop nesting and optimize featureID checks
                for (int j = old.startIndex; j < old.startIndex + old.contactCount; j++)
                {
                    ref var o = ref oldContacts[j];

                    if (c.featureId == o.featureId)
                    {
                        c.accumulatedImpulse = o.accumulatedImpulse;
                        c.accumulatedFriction = o.accumulatedFriction;
                        c.tangent = o.tangent;
                        break; // Once the match is found, stop iterating.
                    }
                }
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Resolve(ref ContactSlim[] contacts, in f32 inverseDt, in WorldSettings settings, in bool useBias)
        {
            var end = startIndex + contactCount;

            var bias = f32.zero;

            if (useBias)
            {
                var separation = MathS.Min(f32.zero, this.penetrationDepth + settings.DefaultContactOffset);
                bias = MathS.Max(settings.Baumgartner * separation * inverseDt, -settings.DefaultMaxDepenetrationVelocity);
            }

            // Resolve impulse for all contacts first
            for (int i = startIndex; i < end; i++)
            {
                ref var c = ref contacts[i];
                c.SolveImpulse(a, b, normal, bias);
            }

            // Resolve impulse for all contacts first
            for (int i = startIndex; i < end; i++)
            {
                ref var c = ref contacts[i];
                c.SolveFriction(a, b, friction);
            }
        }
    }
}
