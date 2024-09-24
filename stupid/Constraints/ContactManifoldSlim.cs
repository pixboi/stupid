using stupid;
using stupid.Broadphase;
using stupid.Maths;
using System.Runtime.CompilerServices;

namespace stupid.Constraints
{
    public readonly struct ContactManifoldSlim
    {
        public readonly RigidbodyS a, b;
        public readonly Vector3S normal;
        public readonly f32 penetrationDepth, friction;
        public readonly int startIndex, contactCount;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ContactManifoldSlim(RigidbodyS a, Collidable b, in Vector3S normal, in f32 penetrationDepth, int startIndex = -1, int contactCount = -1)
        {
            this.a = a;
            this.b = b.isDynamic ? (RigidbodyS)b : null;
            this.normal = normal;
            this.penetrationDepth = penetrationDepth;
            this.startIndex = startIndex;
            this.contactCount = contactCount;
            this.friction = MathS.Sqrt(a.material.staticFriction * b.material.staticFriction);

            if (contactCount < 1)
            {
                throw new System.ArgumentException("ZERO CONTACTS?");
            }
        }

        // Retain impulse data from the old manifold
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void RetainData(ref ContactSlim[] contacts, in ContactSlim[] oldContacts, in ContactManifoldSlim old)
        {
            for (int i = this.startIndex; i < this.startIndex + this.contactCount; i++)
            {
                ref var c = ref contacts[i];

                for (int j = old.startIndex; j < old.startIndex + old.contactCount; j++)
                {
                    ref var o = ref oldContacts[j];

                    if (Transfer(ref c, o))
                    {
                        break; // Exit the inner loop once a match is found
                    }
                }
            }
        }

        // Retain impulse data from the old manifold
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void RetainDataNormal(ref ContactSlimNormal[] contacts, in ContactSlimNormal[] oldContacts, in ContactManifoldSlim old)
        {
            for (int i = this.startIndex; i < this.startIndex + this.contactCount; i++)
            {
                ref var c = ref contacts[i];

                for (int j = old.startIndex; j < old.startIndex + old.contactCount; j++)
                {
                    ref var o = ref oldContacts[j];
                    if (c.Transfer(o))
                    {
                        break;
                    }
                }
            }
        }

        // Retain impulse data from the old manifold
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void RetainDataFriction(ref ContactSlimFriction[] contacts, in ContactSlimFriction[] oldContacts, in ContactManifoldSlim old)
        {
            for (int i = this.startIndex; i < this.startIndex + this.contactCount; i++)
            {
                ref var c = ref contacts[i];

                for (int j = old.startIndex; j < old.startIndex + old.contactCount; j++)
                {
                    ref var o = ref oldContacts[j];
                    if (c.Transfer(o))
                    {
                        break;
                    }
                }
            }
        }

        // Transfer impulse data from an old contact to a new one
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private bool Transfer(ref ContactSlim c, in ContactSlim old)
        {
            if (c.featureId == old.featureId)
            {
                c.accumulatedImpulse = old.accumulatedImpulse;
                c.accumulatedFriction = old.accumulatedFriction;
                c.tangent = old.tangent;
                return true;
            }

            return false;
        }


        // Prestep calculations for all contacts
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void CalculatePrestep(ref ContactSlim[] contacts)
        {
            for (int i = startIndex; i < startIndex + contactCount; i++)
            {
                ref var c = ref contacts[i];
                var ra = c.point - a.transform.position;
                var rb = b != null ? c.point - b.transform.position : Vector3S.zero;

                c.CalculatePrestep(a, b, ra, rb, this);
            }
        }

        // Prestep calculations for all contacts
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void CalculatePrestep2(ref ContactSlimNormal[] c0, ref ContactSlimFriction[] c1)
        {
            for (int i = startIndex; i < startIndex + contactCount; i++)
            {
                ref var c = ref c0[i];
                var ra = c.point - a.transform.position;
                var rb = c.point - b.transform.position;

                c.CalculatePrestep(a, b, ra, rb, this.normal);

                ref var f = ref c1[i]; // Using ref to avoid copying the struct
                f.CalculatePrestep(a, b, ra, rb, this.normal);
            }
        }

        // Warmup for iterative solvers
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Warmup(ref ContactSlim[] contacts)
        {
            for (int i = startIndex; i < startIndex + contactCount; i++)
            {
                ref var c = ref contacts[i];
                var ra = c.point - a.transform.position;
                var rb = b != null ? c.point - b.transform.position : Vector3S.zero;

                c.WarmStart(a, b, ra, rb, this.normal);
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Warmup2(ref ContactSlimNormal[] c0, ref ContactSlimFriction[] c1)
        {
            for (int i = startIndex; i < startIndex + contactCount; i++)
            {
                var c = c0[i];
                var ra = c.point - a.transform.position;
                var rb = c.point - b.transform.position;

                c0[i].WarmStart(a, b, ra, rb, this.normal);
                c1[i].WarmStart(a, b, ra, rb);
            }
        }

        public void Resolve2(ref ContactSlimNormal[] c0, ref ContactSlimFriction[] c1, in f32 inverseDt, in WorldSettings settings, in bool bias)
        {
            var end = startIndex + contactCount;

            for (int i = this.startIndex; i < end; i++)
            {
                ref var c = ref c0[i]; // Using ref to avoid copying the struct
                var ra = c.point - a.transform.position;
                var rb = c.point - b.transform.position;

                c.SolveImpulse(a, b, ra, rb, this.normal, inverseDt, settings, penetrationDepth, bias);

                ref var f = ref c1[i]; // Using ref to avoid copying the struct
                f.SolveFriction(a, b, ra, rb, c.accumulatedImpulse, this.friction);
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Resolve(ref ContactSlim[] contacts, in f32 inverseDt, in WorldSettings settings, in bool bias)
        {
            for (int i = this.startIndex; i < startIndex + contactCount; i++)
            {
                ref var c = ref contacts[i]; // Using ref to avoid copying the struct
                var ra = c.point - a.transform.position;
                var rb = b != null ? c.point - b.transform.position : Vector3S.zero;

                c.SolveImpulse(a, b, ra, rb, inverseDt, settings, penetrationDepth, normal, bias);
                c.SolveFriction(a, b, ra, rb, friction);
            }

        }
    }
}