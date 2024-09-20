﻿using stupid.Broadphase;
using stupid.Maths;
using System.Runtime.CompilerServices;

namespace stupid.Constraints
{
    public struct ContactManifoldS
    {
        public readonly RigidbodyS a;
        public readonly Collidable b;
        public readonly int contactCount;
        public ContactS one, two, three, four;

        // Constructor
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ContactManifoldS(RigidbodyS a, Collidable b, int contactCount, in ContactS[] contactCache)
        {
            this.a = a;
            this.b = b;
            this.contactCount = (byte)contactCount;

            one = contactCache[0];
            two = contactCache[1];
            three = contactCache[2];
            four = contactCache[3];
        }

        public IntPair ToPair => new IntPair(a.index, b.index);

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
            for (int i = 0; i < contactCount; i++)
            {
                ref var contact = ref this[i];
                contact.CalculatePrestep(a, b);
            }
        }

        // Warmup for iterative solvers
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Warmup()
        {
            for (int i = 0; i < contactCount; i++)
            {
                this[i].WarmStart(a, b);
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

            Vector3S avgPoint = Vector3S.zero;
            f32 sumAccum = f32.zero;

            if (contactCount >= 1)
            {
                one.SolveImpulse(a, b, inverseDt, settings, bias);
                avgPoint += one.point;
                sumAccum += one.accumulatedImpulse;
            }

            if (contactCount >= 2)
            {
                two.SolveImpulse(a, b, inverseDt, settings, bias);
                avgPoint += two.point;
                sumAccum += two.accumulatedImpulse;
            }

            if (contactCount >= 3)
            {
                three.SolveImpulse(a, b, inverseDt, settings, bias);
                avgPoint += three.point;
                sumAccum += three.accumulatedImpulse;
            }

            if (contactCount >= 4)
            {
                four.SolveImpulse(a, b, inverseDt, settings, bias);
                avgPoint += four.point;
                sumAccum += four.accumulatedImpulse;
            }

            var friction = MathS.Sqrt(a.material.staticFriction * b.material.staticFriction);



            if (contactCount >= 1)
            {
                one.SolveFriction(a, b, inverseDt, friction, settings, bias);
            }

            if (contactCount >= 2)
            {
                two.SolveFriction(a, b, inverseDt, friction, settings, bias);
            }

            if (contactCount >= 3)
            {
                three.SolveFriction(a, b, inverseDt, friction, settings, bias);
            }

            if (contactCount >= 4)
            {
                four.SolveFriction(a, b, inverseDt, friction, settings, bias);
            }

        }
    }
}
