using stupid;
using stupid.Broadphase;
using stupid.Maths;
using System.Runtime.CompilerServices;

namespace stupid.Constraints
{
    public readonly struct ContactManifoldSlim
    {
        public readonly RigidbodyS a;
        public readonly Collidable b;
        public readonly Vector3S normal;
        public readonly f32 penetrationDepth;
        public readonly int startIndex, contactCount;

        public ContactManifoldSlim(RigidbodyS a, Collidable b, in Vector3S normal, in f32 penetrationDepth, int startIndex = -1, int contactCount = -1)
        {
            this.a = a;
            this.b = b;
            this.normal = normal;
            this.penetrationDepth = penetrationDepth;
            this.startIndex = startIndex;
            this.contactCount = contactCount;

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
                var c = contacts[i];

                for (int j = old.startIndex; j < old.startIndex + old.contactCount; j++)
                {
                    var o = oldContacts[j];

                    if (Transfer(ref c, o))
                    {
                        contacts[i] = c;
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
        public void CalculatePrestep(ref ContactSlim[] contacts, RigidbodyS a, RigidbodyS b)
        {
            for (int i = startIndex; i < startIndex + contactCount; i++)
            {
                var contact = contacts[i];
                contact.CalculatePrestep(a, b, this);
                contacts[i] = contact;
            }
        }

        // Warmup for iterative solvers
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Warmup(ContactSlim[] contacts)
        {
            var bb = b.isDynamic ? (RigidbodyS)b : null;

            for (int i = startIndex; i < startIndex + contactCount; i++)
            {
                contacts[i].WarmStart(normal, a, bb);
            }
        }

        public void Resolve(ref ContactSlim[] contacts, in f32 inverseDt, in WorldSettings settings, in bool bias)
        {
            var bb = b.isDynamic ? (RigidbodyS)b : null;
            var friction = MathS.Sqrt(a.material.staticFriction * b.material.staticFriction);

            for (int i = this.startIndex; i < startIndex + contactCount; i++)
            {
                var c = contacts[i];
                c.SolveImpulse(a, bb, inverseDt, settings, penetrationDepth, normal, bias);
                contacts[i] = c;
            }

            for (int i = this.startIndex; i < startIndex + contactCount; i++)
            {
                var c = contacts[i];
                c.SolveFriction(a, bb, friction);
                contacts[i] = c;
            }

        }
    }
}