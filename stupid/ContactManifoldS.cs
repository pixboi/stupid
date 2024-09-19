using stupid.Maths;
using System.Diagnostics.Contracts;
using System.Runtime.CompilerServices;

namespace stupid.Colliders
{
    public struct ContactManifoldS
    {
        public readonly Collidable a, b;
        public readonly RigidbodyS ab, bb;
        public readonly f32 friction, restitution;


        public readonly int contactCount;
        public ContactS one, two, three, four;

        // Constructor
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ContactManifoldS(in Collidable a, in Collidable b, int contactCount, in ContactS[] contactCache)
        {
            this.a = a;
            this.b = b;
            this.ab = a.isDynamic ? (RigidbodyS)a : null;
            this.bb = b.isDynamic ? (RigidbodyS)b : null;

            this.friction = (a.material.staticFriction + b.material.staticFriction) * f32.half;
            this.restitution = (a.material.restitution + b.material.restitution) * f32.half;

            this.contactCount = contactCount;
            this.one = contactCache[0];
            this.two = contactCache[1];
            this.three = contactCache[2];
            this.four = contactCache[3];
        }

        // Indexer with a direct array-like access pattern using unsafe pointers
        public ref ContactS this[int i]
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                if (i < 0 || i >= 4) throw new System.ArgumentOutOfRangeException();
                unsafe
                {
                    fixed (ContactS* ptr = &one)
                    {
                        return ref *(ptr + i);
                    }
                }
            }
        }

        // Returns an IntPair for indexing
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public IntPair ToPair() => new IntPair(a.index, b.index);

        // Copy contacts to an array
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void CopyToArray(ref ContactS[] array)
        {
            if (array.Length < contactCount)
            {
                throw new System.ArgumentException("The target array is too small to copy contacts.");
            }

            for (int i = 0; i < contactCount; i++)
            {
                array[i] = this[i];
            }
        }

        // Retain impulse data from the old manifold
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void RetainData(in ContactManifoldS old)
        {
            for (int i = 0; i < this.contactCount; i++)
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
        private bool Transfer(ref ContactS c, in ContactS old)
        {
            if (c.featureId == old.featureId)
            {
                c.accumulatedImpulse = old.accumulatedImpulse;
                c.accumulatedFriction = old.accumulatedFriction;
                c.prevTangent = old.tangent;
                return true;
            }
            return false;
        }

        // Prestep calculations for all contacts
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void CalculatePrestep()
        {
            for (int i = 0; i < this.contactCount; i++)
            {
                ref var contact = ref this[i];
                contact.CalculatePrestep(a, b);
            }
        }

        // Warmup for iterative solvers
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Warmup()
        {
            for (int i = 0; i < this.contactCount; i++)
            {
                this[i].WarmStart(ab, b);
            }
        }

        // Resolve impulses and friction
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void ResolveAll(in f32 inverseDt, in WorldSettings settings, in bool bias)
        {
            if (contactCount == 0)
            {
                throw new System.ArgumentException("ContactManifoldS: Attempted to resolve with no contacts in manifold.");
            }

            if (contactCount >= 1)
            {
                one.SolveImpulse(ab, b, inverseDt, settings, bias);
            }

            if (contactCount >= 2)
            {
                two.SolveImpulse(ab, b, inverseDt, settings, bias);
            }

            if (contactCount >= 3)
            {
                three.SolveImpulse(ab, b, inverseDt, settings, bias);
            }

            if (contactCount >= 4)
            {
                four.SolveImpulse(ab, b, inverseDt, settings, bias);
            }


            if (contactCount >= 1)
            {
                one.SolveFriction(ab, b, inverseDt, friction, settings, bias);
            }

            if (contactCount >= 2)
            {
                two.SolveFriction(ab, b, inverseDt, friction, settings, bias);
            }

            if (contactCount >= 3)
            {
                three.SolveFriction(ab, b, inverseDt, friction, settings, bias);
            }

            if (contactCount >= 4)
            {
                four.SolveFriction(ab, b, inverseDt, friction, settings, bias);
            }

            /*

            // Solve impulses with offset
            for (int i = 0; i < contactCount; i++)
            {
                ref var contact = ref this[(i) % contactCount];
                contact.SolveImpulse(ab, b, inverseDt, settings, bias);
            }

            // Solve friction with offset
            for (int i = 0; i < contactCount; i++)
            {
                ref var contact = ref this[(i) % contactCount];
                contact.SolveFriction(ab, b, inverseDt, friction, settings, bias);
            }
            */
        }
    }
}
