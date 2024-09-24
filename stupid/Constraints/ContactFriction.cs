using stupid.Maths;
using System.Runtime.CompilerServices;

namespace stupid.Constraints
{
    public struct ContactFriction
    {
        public Vector3S tangent; //24
        public f32 tangentMass; //8
        public f32 accumulatedFriction; //8
        public readonly int featureId; //4

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ContactFriction(in ContactData data)
        {
            tangent = Vector3S.zero;
            tangentMass = f32.zero;
            accumulatedFriction = f32.zero;
            featureId = data.featureId;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void CalculatePrestep(RigidbodyS a, RigidbodyS b, in Vector3S ra, in Vector3S rb, in Vector3S normal)
        {
            var contactVelocity = ContactSlim.CalculateContactVelocity(a, b, ra, rb);
            Vector3S tangentialVelocity = contactVelocity - (normal * Vector3S.Dot(contactVelocity, normal));

            f32 tangentMag = tangentialVelocity.sqrMagnitude;

            // Retain the previous tangent direction
            var oldTangent = this.tangent;
            var newTangent = tangentialVelocity.Normalize();
            var blend = MathS.Clamp(tangentMag, f32.zero, f32.small) / f32.small;
            this.tangent = Vector3S.Lerp(oldTangent, newTangent, blend).Normalize();

            var raCrossTangent = Vector3S.Cross(ra, this.tangent);
            var tMass = a.inverseMass + Vector3S.Dot(Vector3S.Cross(a.tensor.inertiaWorld * raCrossTangent, ra), this.tangent);

            if (b != null)
            {
                var rbCrossTangent = Vector3S.Cross(rb, this.tangent);
                tMass += b.inverseMass + Vector3S.Dot(Vector3S.Cross(b.tensor.inertiaWorld * rbCrossTangent, rb), this.tangent);
            }

            this.tangentMass = tMass > f32.zero ? f32.one / tMass : f32.zero;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void SolveFriction(RigidbodyS a, RigidbodyS b, in Vector3S ra, in Vector3S rb, in f32 accumulatedImpulse, in f32 friction)
        {
            var contactVelocity = ContactSlim.CalculateContactVelocity(a, b, ra, rb);
            var vt = Vector3S.Dot(contactVelocity, this.tangent);

            var incrementalFriction = -this.tangentMass * vt;

            var coulombMax = accumulatedImpulse * friction;
            var newImpulse = MathS.Clamp(this.accumulatedFriction + incrementalFriction, -coulombMax, coulombMax);
            incrementalFriction = newImpulse - this.accumulatedFriction;
            this.accumulatedFriction = newImpulse;

            var impulse = this.tangent * incrementalFriction;
            ContactSlim.ApplyImpulse(a, b, impulse, ra, rb);
        }
    }
}
