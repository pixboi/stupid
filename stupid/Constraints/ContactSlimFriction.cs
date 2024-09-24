using stupid.Maths;
using System.Runtime.CompilerServices;

namespace stupid.Constraints
{
   

    public struct ContactSlimFriction
    {
        public Vector3S tangent;
        public f32 tangentMass;
        public f32 accumulatedFriction;
        public readonly int featureId;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ContactSlimFriction(in ContactData data)
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
            var oldTangent = tangent;
            var newTangent = tangentialVelocity.Normalize();
            var blend = MathS.Clamp(tangentMag, f32.zero, f32.small) / f32.small;
            tangent = Vector3S.Lerp(oldTangent, newTangent, blend).Normalize();

            var raCrossTangent = Vector3S.Cross(ra, tangent);
            var tMass = a.inverseMass + Vector3S.Dot(Vector3S.Cross(a.tensor.inertiaWorld * raCrossTangent, ra), tangent);

            if (b != null)
            {
                var rbCrossTangent = Vector3S.Cross(rb, tangent);
                tMass += b.inverseMass + Vector3S.Dot(Vector3S.Cross(b.tensor.inertiaWorld * rbCrossTangent, rb), tangent);
            }

            tangentMass = tMass > f32.zero ? f32.one / tMass : f32.zero;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void SolveFriction(RigidbodyS a, RigidbodyS b, in Vector3S ra, in Vector3S rb, in f32 accumulatedImpulse, in f32 friction)
        {
            var contactVelocity = ContactSlim.CalculateContactVelocity(a, b, ra, rb);
            var vt = Vector3S.Dot(contactVelocity, tangent);

            var incrementalFriction = -tangentMass * vt;

            var coulombMax = accumulatedImpulse * friction;
            var newImpulse = MathS.Clamp(accumulatedFriction + incrementalFriction, -coulombMax, coulombMax);
            incrementalFriction = newImpulse - accumulatedFriction;
            accumulatedFriction = newImpulse;

            var impulse = tangent * incrementalFriction;
            ContactSlim.ApplyImpulse(a, b, impulse, ra, rb);
        }
    }
}
