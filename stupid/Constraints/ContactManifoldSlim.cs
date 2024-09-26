using stupid.Maths;
using System;
using System.Collections.Generic;
using System.Runtime.CompilerServices;

namespace stupid.Constraints
{
    public struct ContactManifoldSlim
    {
        public readonly int aIndex, bIndex;
        public readonly Vector3S normal; // 24 
        public readonly f32 penetrationDepth, friction; // 16
        public readonly int startIndex, contactCount; // 8
        public readonly f32 bias;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ContactManifoldSlim(Collidable a, Collidable b, in Vector3S normal, in f32 penetrationDepth, in WorldSettings settings, in f32 inverseDt, int startIndex = -1, int contactCount = -1)
        {
            if (contactCount < 1) throw new System.ArgumentException("ZERO CONTACTS?");
            this.aIndex = a.index;
            this.bIndex = b.index;
            this.normal = normal;
            this.penetrationDepth = penetrationDepth;
            this.startIndex = startIndex;
            this.contactCount = contactCount;
            this.friction = MathS.Sqrt(a.material.staticFriction * b.material.staticFriction);

            var separation = MathS.Min(f32.zero, this.penetrationDepth + settings.DefaultContactOffset);
            this.bias = MathS.Max(settings.Baumgartner * separation * inverseDt, -settings.DefaultMaxDepenetrationVelocity);
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

        // Prestep calculations for all contacts
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void CalculatePrestep(in RigidbodyData a, in RigidbodyData b, ref ContactSlim[] contacts)
        {
            for (int i = startIndex; i < startIndex + contactCount; i++)
            {
                ref var c = ref contacts[i];
                c.CalculatePrestep(a, b, this);
            }
        }

        // Warmup for iterative solvers
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Warmup(ref RigidbodyData a, ref RigidbodyData b, ref Span<ContactSlim> contacts)
        {
            for (int i = startIndex; i < startIndex + contactCount; i++)
            {
                ref var c = ref contacts[i];
                c.WarmStart(ref a, ref b, this.normal);
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Resolve(ref RigidbodyData aBody, ref RigidbodyData bBody, ref Span<ContactSlim> contacts, in bool useBias)
        {
            var end = startIndex + contactCount;
            var biassi = useBias ? this.bias : f32.zero;

            // Resolve impulse for all contacts first
            for (int i = startIndex; i < end; i++)
            {
                ref var c = ref contacts[i];
                c.SolveImpulse(ref aBody, ref bBody, normal, biassi);
                c.SolveFriction(ref aBody, ref bBody, friction);
            }
        }
    }
}
