using stupid.Maths;
using System.Runtime.CompilerServices;

namespace stupid.Constraints
{
    public struct ContactSlimFriction
    {
        public readonly byte featureId;
        public Vector3S tangent;
        public f32 tangentMass, accumulatedFriction;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ContactSlimFriction(in ContactData data)
        {
            featureId = data.featureId;
            tangentMass = f32.zero;
            tangent = Vector3S.zero;
            accumulatedFriction = f32.zero;
        }

        public bool Transfer(in ContactSlimFriction old)
        {
            if (old.featureId == this.featureId)
            {
                this.tangent = old.tangent;
                this.accumulatedFriction = old.accumulatedFriction;
                return true;

            }

            return false;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void CalculatePrestep(RigidbodyS a, RigidbodyS b, in Vector3S ra, in Vector3S rb, in Vector3S normal)
        {
            // Calculate relative velocity at the contact point
            var contactVelocity = ContactSlim.CalculateContactVelocity(a, b, ra, rb);
            Vector3S normalVelocity = normal * Vector3S.Dot(contactVelocity, normal);
            Vector3S tangentialVelocity = contactVelocity - normalVelocity;
            f32 tangentMag = tangentialVelocity.sqrMagnitude;

            //In retain, the previous tangent is stored IN THIS.TANGENT!
            var oldTangent = tangent;
            var newTangent = tangentialVelocity.Normalize();
            var blend = MathS.Clamp(tangentMag, f32.zero, f32.small) / f32.small;
            this.tangent = Vector3S.Lerp(oldTangent, newTangent, blend).Normalize();

            // Precompute cross products for mass calculation
            var raCrossTangent = Vector3S.Cross(ra, tangent);
            var tmass = a.inverseMass + Vector3S.Dot(Vector3S.Cross(a.tensor.inertiaWorld * raCrossTangent, ra), tangent);

            if (b != null)
            {
                var rbCrossTangent = Vector3S.Cross(rb, tangent);
                tmass += b.inverseMass + Vector3S.Dot(Vector3S.Cross(b.tensor.inertiaWorld * rbCrossTangent, rb), tangent);
            }

            tangentMass = tmass > f32.zero ? f32.one / tmass : f32.zero;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void WarmStart(in RigidbodyS a, in RigidbodyS b, in Vector3S ra, in Vector3S rb)
        {
            Vector3S warmImpulse = tangent * this.accumulatedFriction;
            ContactSlim.ApplyImpulse(a, b, warmImpulse, ra, rb);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void SolveFriction(in RigidbodyS a, in RigidbodyS b, in Vector3S ra, in Vector3S rb, in f32 accumulatedImpulse, in f32 friction)
        {
            var contactVelocity = ContactSlim.CalculateContactVelocity(a, b, ra, rb);

            var vt = Vector3S.Dot(contactVelocity, tangent);
            var incrementalFriction = -tangentMass * vt;

            var couloumbMax = accumulatedImpulse * friction;
            var newImpulse = MathS.Clamp(accumulatedFriction + incrementalFriction, -couloumbMax, couloumbMax);
            incrementalFriction = newImpulse - accumulatedFriction;
            accumulatedFriction = newImpulse;

            var impulse = tangent * incrementalFriction;
            ContactSlim.ApplyImpulse(a, b, impulse, ra, rb);
        }
    }
}