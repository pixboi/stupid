using stupid;
using stupid.Maths;
using System.Drawing;
using System.Runtime.CompilerServices;

namespace stupid.Constraints
{
    public struct ContactSlim
    {
        public readonly Vector3S point; //24
        public Vector3S tangent; //24
        public f32 normalMass, accumulatedImpulse; // 16
        public f32 tangentMass, accumulatedFriction; // 16
        public readonly int featureId; //1

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ContactSlim(Collidable a, Collidable b, in ContactData data)
        {
            featureId = data.featureId;
            accumulatedImpulse = f32.zero;
            normalMass = f32.zero;
            accumulatedFriction = f32.zero;
            tangentMass = f32.zero;
            tangent = Vector3S.zero;
            point = data.point;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3S CalculateContactVelocity(in RigidbodyData a, in RigidbodyData b, in Vector3S ra, in Vector3S rb)
        {
            var av = a.velocity + Vector3S.Cross(a.angularVelocity, ra);
            var bv = b.isDynamic ? b.velocity + Vector3S.Cross(b.angularVelocity, rb) : Vector3S.zero;
            return bv - av;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3S CalculateTangent(in Vector3S normal, in Vector3S contactVelocity)
        {
            // Calculate relative velocity at the contact point
            Vector3S normalVelocity = normal * Vector3S.Dot(contactVelocity, normal);
            Vector3S tangentialVelocity = contactVelocity - normalVelocity;
            return tangentialVelocity.Normalize();
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static f32 CalculateTangentMass(in RigidbodyData a, in RigidbodyData b, in Vector3S ra, in Vector3S rb, in Vector3S tangent)
        {
            // Precompute cross products for mass calculation
            var raCrossTangent = Vector3S.Cross(ra, tangent);
            var tangentMass = a.inverseMass + Vector3S.Dot(Vector3S.Cross(a.inertiaWorld * raCrossTangent, ra), tangent);

            if (b.isDynamic)
            {
                var rbCrossTangent = Vector3S.Cross(rb, tangent);
                tangentMass += b.inverseMass + Vector3S.Dot(Vector3S.Cross(b.inertiaWorld * rbCrossTangent, rb), tangent);
            }

            return tangentMass > f32.zero ? -(f32.one / tangentMass) : f32.zero;
        }

        public static f32 CalculateNormalMass(in RigidbodyData a, in RigidbodyData b, in Vector3S ra, in Vector3S rb, in Vector3S normal)
        {
            //I think this is fine, since normal or inertia doesnt change between frames
            Vector3S raCrossNormal = Vector3S.Cross(ra, normal);
            f32 angularMassA = Vector3S.Dot(raCrossNormal, a.inertiaWorld * raCrossNormal);
            f32 effectiveMass = a.inverseMass + angularMassA;

            if (b.isDynamic)
            {
                Vector3S rbCrossNormal = Vector3S.Cross(rb, normal);
                f32 angularMassB = Vector3S.Dot(rbCrossNormal, b.inertiaWorld * rbCrossNormal);
                effectiveMass += b.inverseMass + angularMassB;
            }

            return effectiveMass > f32.zero ? -(f32.one / effectiveMass) : f32.zero;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ApplyImpulse(ref RigidbodyData a, ref RigidbodyData b, in Vector3S impulse, in Vector3S ra, in Vector3S rb)
        {
            a.velocity -= impulse * a.inverseMass; // A moves away
            a.angularVelocity -= a.inertiaWorld * Vector3S.Cross(ra, impulse);

            if (b.isDynamic)
            {
                b.velocity += impulse * b.inverseMass; // B moves along normal
                b.angularVelocity += b.inertiaWorld * Vector3S.Cross(rb, impulse);
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void CalculatePrestep(in RigidbodyData a, in RigidbodyData b, in ContactManifoldSlim manifold)
        {
            var ra = this.point - a.position;
            var rb = this.point - b.position;

            this.normalMass = CalculateNormalMass(a, b, ra, rb, manifold.normal);

            var contactVelocity = CalculateContactVelocity(a, b, ra, rb);
            this.tangent = CalculateTangent(manifold.normal, contactVelocity);
            this.tangentMass = CalculateTangentMass(a, b, ra, rb, tangent);
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void WarmStart(ref RigidbodyData a, ref RigidbodyData b, in Vector3S normal)
        {
            var ra = this.point - a.position;
            var rb = this.point - b.position;

            Vector3S warmImpulse = (normal * accumulatedImpulse) + (tangent * accumulatedFriction);
            ApplyImpulse(ref a, ref b, warmImpulse, ra, rb);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void SolveImpulse(ref RigidbodyData a, ref RigidbodyData b, in Vector3S normal, in f32 bias, in f32 friction)
        {
            var ra = this.point - a.position;
            var rb = this.point - b.position;
            var contactVelocity = CalculateContactVelocity(a, b, ra, rb);

            var relativeNormalVelocity = Vector3S.Dot(contactVelocity, normal);

            var impulse = this.normalMass * (relativeNormalVelocity + bias);
            var oldImpulse = this.accumulatedImpulse;
            this.accumulatedImpulse = MathS.Max(oldImpulse + impulse, f32.zero);
            impulse = this.accumulatedImpulse - oldImpulse;

            var added = (normal * impulse);
            ApplyImpulse(ref a, ref b, added, ra, rb);
        }

        public void SolveFriction(ref RigidbodyData a, ref RigidbodyData b, in Vector3S normal, in f32 bias, in f32 friction)
        {
            var ra = this.point - a.position;
            var rb = this.point - b.position;
            var contactVelocity = CalculateContactVelocity(a, b, ra, rb);

            var couloumbMax = this.accumulatedImpulse * friction;

            var impulse = tangentMass * Vector3S.Dot(contactVelocity, tangent);
            var oldImpulse = this.accumulatedFriction;
            this.accumulatedFriction = MathS.Clamp(oldImpulse + impulse, -couloumbMax, couloumbMax);
            impulse = this.accumulatedFriction - oldImpulse;

            var added = (tangent * impulse);
            ApplyImpulse(ref a, ref b, added, ra, rb);
        }
    }
}