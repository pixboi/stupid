using stupid;
using stupid.Broadphase;
using stupid.Maths;
using System.Runtime.CompilerServices;

namespace stupid.Constraints
{
    public struct ContactManifoldSlim
    {
        public readonly RigidbodyS a;
        public readonly Collidable b;
        public readonly Vector3S normal;
        public readonly f32 penetrationDepth;
        public readonly int contactCount;
        public ContactSlim c0, c1, c2, c3;

        public ContactManifoldSlim(RigidbodyS a, Collidable b, in ContactData[] data, int contactCount)
        {
            this.a = a;
            this.b = b;
            normal = data[0].normal;
            penetrationDepth = data[0].penetrationDepth;
            this.contactCount = contactCount;

            if (contactCount < 1)
            {
                throw new System.ArgumentException("ZERO CONTACTS?");
            }

            var bb = b.isDynamic ? (RigidbodyS)b : null;

            c0 = new ContactSlim(data[0]);
            c1 = new ContactSlim(data[1]);
            c2 = new ContactSlim(data[2]);
            c3 = new ContactSlim(data[3]);
        }

        public IntPair ToPair => new IntPair(a.index, b.index);

        // Indexer with a direct array-like access pattern using unsafe pointers
        public ref ContactSlim this[int i]
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                if (i < 0 || i >= 4) throw new System.ArgumentOutOfRangeException();
                unsafe
                {
                    fixed (ContactSlim* ptr = &c0)
                    {
                        return ref *(ptr + i);
                    }
                }
            }
        }

        // Retain impulse data from the old manifold
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void RetainData(in ContactManifoldSlim old)
        {
            for (int i = 0; i < contactCount; i++)
            {
                ref var c = ref this[i];

                for (int j = 0; j < old.contactCount; j++)
                {
                    var o = old[j];
                    if (Transfer(ref c, o))
                    {
                        break; // Exit the inner loop once a match is found
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
        public void CalculatePrestep(RigidbodyS a, RigidbodyS b)
        {
            for (int i = 0; i < contactCount; i++)
            {
                ref var contact = ref this[i];
                contact.CalculatePrestep(a, b, normal);
            }
        }

        // Warmup for iterative solvers
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Warmup()
        {
            var bb = b.isDynamic ? (RigidbodyS)b : null;

            for (int i = 0; i < contactCount; i++)
            {
                this[i].WarmStart(normal, a, bb);
            }
        }

        public void Resolve(f32 inverseDt, WorldSettings settings, bool bias)
        {
            var bb = b.isDynamic ? (RigidbodyS)b : null;
            var friction = MathS.Sqrt(a.material.staticFriction * b.material.staticFriction);

            if (contactCount == 1)
            {
                c0.SolveImpulse(a, bb, inverseDt, settings, penetrationDepth, normal, bias);
                c0.SolveFriction(a, bb, friction);
                return;
            }

            if (contactCount >= 1)
            {
                c0.SolveImpulse(a, bb, inverseDt, settings, penetrationDepth, normal, bias);
                c0.SolveFriction(a, bb, friction);

            }

            if (contactCount >= 2)
            {
                c1.SolveImpulse(a, bb, inverseDt, settings, penetrationDepth, normal, bias);
                c1.SolveFriction(a, bb, friction);

            }

            if (contactCount >= 3)
            {
                c2.SolveImpulse(a, bb, inverseDt, settings, penetrationDepth, normal, bias);
                c2.SolveFriction(a, bb, friction);

            }

            if (contactCount >= 4)
            {
                c3.SolveImpulse(a, bb, inverseDt, settings, penetrationDepth, normal, bias);
                c3.SolveFriction(a, bb, friction);

            }

            return;

            if (contactCount >= 1)
            {
                c0.SolveFriction(a, bb, friction);
            }

            if (contactCount >= 2)
            {
                c1.SolveFriction(a, bb, friction);
            }

            if (contactCount >= 3)
            {
                c2.SolveFriction(a, bb, friction);
            }

            if (contactCount >= 4)
            {
                c3.SolveFriction(a, bb, friction);
            }
        }
    }
}