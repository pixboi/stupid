using stupid;
using stupid.Maths;
using System.Drawing;
using System.Runtime.CompilerServices;

namespace stupid.Constraints
{
    public struct ContactSlim
    {
        public readonly int featureId; //4
        public Vector3S point, tangent, ra, rb; // 4 * 24 = 96
        public f32 tangentMass, accumulatedFriction, normalMass, accumulatedImpulse; // 4 * 8 = 32

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ContactSlim(in ContactData data)
        {
            point = data.point;
            featureId = data.featureId;
            accumulatedImpulse = f32.zero;
            normalMass = f32.zero;
            tangentMass = f32.zero;
            tangent = Vector3S.zero;
            accumulatedFriction = f32.zero;
            ra = Vector3S.zero;
            rb = Vector3S.zero;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void CalculatePrestep(RigidbodyS a, RigidbodyS b, in ContactManifoldSlim manifold)
        {
            this.ra = this.point - a.transform.position;
            this.rb = b != null ? this.point - b.transform.position : Vector3S.zero;

            Vector3S raCrossNormal = Vector3S.Cross(ra, manifold.normal);
            f32 angularMassA = Vector3S.Dot(raCrossNormal, a.tensor.inertiaWorld * raCrossNormal);
            f32 effectiveMass = a.inverseMass + angularMassA;

            if (b != null)
            {
                Vector3S rbCrossNormal = Vector3S.Cross(rb, manifold.normal);
                f32 angularMassB = Vector3S.Dot(rbCrossNormal, b.tensor.inertiaWorld * rbCrossNormal);
                effectiveMass += b.inverseMass + angularMassB;
            }

            this.normalMass = effectiveMass > f32.zero ? f32.one / effectiveMass : f32.zero;

            // Calculate relative velocity at the contact point
            var contactVelocity = CalculateContactVelocity(a, b, ra, rb);
            Vector3S normalVelocity = manifold.normal * Vector3S.Dot(contactVelocity, manifold.normal);
            Vector3S tangentialVelocity = contactVelocity - normalVelocity;
            f32 tangentMag = tangentialVelocity.sqrMagnitude;

            //In retain, the previous tangent is stored IN THIS.TANGENT!
            var oldTangent = this.tangent;
            var newTangent = tangentialVelocity.Normalize();
            var blend = MathS.Clamp(tangentMag, f32.zero, f32.small) / f32.small;
            this.tangent = Vector3S.Lerp(oldTangent, newTangent, blend).Normalize();

            // Precompute cross products for mass calculation
            var raCrossTangent = Vector3S.Cross(ra, tangent);
            var tMass = a.inverseMass + Vector3S.Dot(Vector3S.Cross(a.tensor.inertiaWorld * raCrossTangent, ra), tangent);

            if (b != null)
            {
                var rbCrossTangent = Vector3S.Cross(rb, tangent);
                tMass += b.inverseMass + Vector3S.Dot(Vector3S.Cross(b.tensor.inertiaWorld * rbCrossTangent, rb), tangent);
            }

            this.tangentMass = tMass > f32.zero ? f32.one / tMass : f32.zero;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void WarmStart(RigidbodyS a, RigidbodyS b, in Vector3S normal)
        {
            Vector3S warmImpulse = normal * accumulatedImpulse + tangent * accumulatedFriction;
            ApplyImpulse(a, b, warmImpulse, ra, rb);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3S CalculateContactVelocity(RigidbodyS a, RigidbodyS b, in Vector3S ra, in Vector3S rb)
        {
            var av = a.velocity + Vector3S.Cross(a.angularVelocity, ra);
            var bv = b != null ? b.velocity + Vector3S.Cross(b.angularVelocity, rb) : Vector3S.zero;
            return bv - av;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ApplyImpulse(RigidbodyS a, RigidbodyS b, in Vector3S impulse, in Vector3S ra, in Vector3S rb)
        {
            a.velocity -= impulse * a.inverseMass; // A moves away
            a.angularVelocity -= a.tensor.inertiaWorld * Vector3S.Cross(ra, impulse);

            if (b != null)
            {
                b.velocity += impulse * b.inverseMass; // B moves along normal
                b.angularVelocity += b.tensor.inertiaWorld * Vector3S.Cross(rb, impulse);
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void SolveImpulse(RigidbodyS a, RigidbodyS b, in Vector3S normal, f32 bias)
        {
            //var contactVelocity = CalculateContactVelocity(a, b, ra, rb);
            var av = a.velocity + Vector3S.Cross(a.angularVelocity, ra);
            var bv = b != null ? b.velocity + Vector3S.Cross(b.angularVelocity, rb) : Vector3S.zero;
            var contactVelocity = bv - av;

            var vn = Vector3S.Dot(contactVelocity, normal);

            var incremental = -this.normalMass * (vn + bias);
            var newImpulse = MathS.Max(incremental + this.accumulatedImpulse, f32.zero);
            incremental = newImpulse - this.accumulatedImpulse;
            this.accumulatedImpulse = newImpulse;

            var impulse = normal * incremental;

            a.velocity -= impulse * a.inverseMass; // A moves away
            a.angularVelocity -= a.tensor.inertiaWorld * Vector3S.Cross(ra, impulse);

            if (b != null)
            {
                b.velocity += impulse * b.inverseMass; // B moves along normal
                b.angularVelocity += b.tensor.inertiaWorld * Vector3S.Cross(rb, impulse);
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void SolveFriction(RigidbodyS a, RigidbodyS b, in f32 friction)
        {
            var av = a.velocity + Vector3S.Cross(a.angularVelocity, ra);
            var bv = b != null ? b.velocity + Vector3S.Cross(b.angularVelocity, rb) : Vector3S.zero;
            var contactVelocity = bv - av;

            var vt = Vector3S.Dot(contactVelocity, this.tangent);
            var incrementalFriction = -this.tangentMass * vt;

            var couloumbMax = this.accumulatedImpulse * friction;
            var newImpulse = MathS.Clamp(this.accumulatedFriction + incrementalFriction, -couloumbMax, couloumbMax);
            incrementalFriction = newImpulse - this.accumulatedFriction;
            this.accumulatedFriction = newImpulse;

            var impulse = this.tangent * incrementalFriction;

            a.velocity -= impulse * a.inverseMass; // A moves away
            a.angularVelocity -= a.tensor.inertiaWorld * Vector3S.Cross(ra, impulse);

            if (b != null)
            {
                b.velocity += impulse * b.inverseMass; // B moves along normal
                b.angularVelocity += b.tensor.inertiaWorld * Vector3S.Cross(rb, impulse);
            }
        }


    }
}