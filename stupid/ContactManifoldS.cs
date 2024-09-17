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

        public int iterationCount;
        public ContactS one, two, three, four;

        public ContactS this[int i]
        {
            get => i switch
            {
                0 => one,
                1 => two,
                2 => three,
                3 => four,
                _ => throw new System.ArgumentOutOfRangeException()
            };
            set
            {
                switch (i)
                {
                    case 0: one = value; break;
                    case 1: two = value; break;
                    case 2: three = value; break;
                    case 3: four = value; break;
                    default: throw new System.ArgumentOutOfRangeException();
                }
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ContactManifoldS(Collidable a, Collidable b, int contactCount, in ContactS[] contactCache)
        {
            this.a = a;
            this.b = b;
            this.ab = a.isDynamic ? (RigidbodyS)a : null;
            this.bb = b.isDynamic ? (RigidbodyS)b : null;

            this.friction = (a.material.staticFriction + b.material.staticFriction) * f32.half;
            this.restitution = (a.material.restitution + b.material.restitution) * f32.half;

            this.contactCount = contactCount;
            this.one = contactCount > 0 ? contactCache[0] : default;
            this.two = contactCount > 1 ? contactCache[1] : default;
            this.three = contactCount > 2 ? contactCache[2] : default;
            this.four = contactCount > 3 ? contactCache[3] : default;

            this.iterationCount = 0;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public IntPair ToPair() => new IntPair(a.index, b.index);

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

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void RetainData(in ContactManifoldS old)
        {
            for (int i = 0; i < this.contactCount; i++)
            {
                var c = this[i];

                for (int j = 0; j < old.contactCount; j++)
                {
                    var o = old[j];
                    if (Transfer(ref c, o))
                    {
                        this[i] = c;
                        break; // Exit the inner loop once a match is found
                    }
                }
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        bool Transfer(ref ContactS c, in ContactS old)
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

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void CalculatePrestep()
        {
            for (int i = 0; i < this.contactCount; i++)
            {
                var contact = this[i];
                contact.CalculatePrestep(a, b);
                this[i] = contact;
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void SubtickUpdate()
        {
            for (int i = 0; i < this.contactCount; i++)
            {
                var contact = this[i];
                contact.SubtickUpdate(a, b);
                this[i] = contact;
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Warmup()
        {
            for (int i = 0; i < this.contactCount; i++) this[i].WarmStart(ab, b);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Resolve(in f32 inverseDt, in WorldSettings settings, in bool bias)
        {
            if (contactCount == 0)
            {
                throw new System.ArgumentException("ContactManifoldS: Attempted to resolve with no contacts in manifold.");
            }

            if (contactCount == 1)
            {
                one.SolveImpulse(ab, b, inverseDt, settings, bias);
                one.SolveFriction(ab, b, inverseDt, friction, settings, one.accumulatedImpulse, bias);
                return;
            }

            //f32 sumAccumulatedImpulses = f32.zero; //For that extra sticky feel
            // Determine the starting offset based on iterationCount
            int offset = iterationCount % contactCount;

            // Solve impulses with offset
            for (int i = 0; i < contactCount; i++)
            {
                int index = (i + offset) % contactCount;
                var contact = this[index];
                contact.SolveImpulse(ab, b, inverseDt, settings, bias);
                this[index] = contact;

            }

            // Solve friction with offset
            for (int i = 0; i < contactCount; i++)
            {
                int index = (i + offset) % contactCount;
                var contact = this[index];
                contact.SolveFriction(ab, b, inverseDt, friction, settings, contact.accumulatedImpulse, bias);
                this[index] = contact;
            }

            this.iterationCount++;
        }

    }
}
