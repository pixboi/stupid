using stupid;
using stupid.Maths;
using System.Drawing;
using System.Runtime.CompilerServices;

namespace stupid.Constraints
{
    public struct ContactSlim
    {
        public readonly Vector3S ra; // 1 * 24 = 24
        public Vector3S tangent; // 24
        public f32 tangentMass, accumulatedFriction, normalMass, accumulatedImpulse; // 4 * 8 = 32
        public readonly int featureId; //4

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ContactSlim(Collidable a, Collidable b, in ContactData data)
        {
            featureId = data.featureId;
            accumulatedImpulse = f32.zero;
            normalMass = f32.zero;
            tangentMass = f32.zero;
            tangent = Vector3S.zero;
            accumulatedFriction = f32.zero;
            ra = data.point - a.transform.position;
            //rb = data.point - b.transform.position;
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3S CalculateContactVelocity(in RigidbodyData a, in RigidbodyData b, in Vector3S ra, in Vector3S rb)
        {
            var av = a.velocity + Vector3S.Cross(a.angularVelocity, ra);
            var bv = b.isDynamic ? b.velocity + Vector3S.Cross(b.angularVelocity, rb) : Vector3S.zero;
            return bv - av;
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
            var rb = (a.position + this.ra) - b.position;

            Vector3S raCrossNormal = Vector3S.Cross(ra, manifold.normal);
            f32 angularMassA = Vector3S.Dot(raCrossNormal, a.inertiaWorld * raCrossNormal);
            f32 effectiveMass = a.inverseMass + angularMassA;

            if (b.isDynamic)
            {
                Vector3S rbCrossNormal = Vector3S.Cross(rb, manifold.normal);
                f32 angularMassB = Vector3S.Dot(rbCrossNormal, b.inertiaWorld * rbCrossNormal);
                effectiveMass += b.inverseMass + angularMassB;
            }

            this.normalMass = effectiveMass > f32.zero ? f32.one / effectiveMass : f32.zero;
            this.normalMass = -this.normalMass;

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
            var tMass = a.inverseMass + Vector3S.Dot(Vector3S.Cross(a.inertiaWorld * raCrossTangent, ra), tangent);

            if (b.isDynamic)
            {
                var rbCrossTangent = Vector3S.Cross(rb, tangent);
                tMass += b.inverseMass + Vector3S.Dot(Vector3S.Cross(b.inertiaWorld * rbCrossTangent, rb), tangent);
            }

            this.tangentMass = tMass > f32.zero ? f32.one / tMass : f32.zero;
            this.tangentMass = -this.tangentMass;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void WarmStart(ref RigidbodyData a, ref RigidbodyData b, in Vector3S normal)
        {
            var rb = (a.position + this.ra) - b.position;

            Vector3S warmImpulse = normal * accumulatedImpulse + tangent * accumulatedFriction;
            ApplyImpulse(ref a, ref b, warmImpulse, ra, rb);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void SolveAll(ref RigidbodyData a, ref RigidbodyData b, in Vector3S normal, in f32 bias, in f32 friction)
        {
            var rb = (a.position + this.ra) - b.position;

            // Precompute velocities
            Vector3S av = a.velocity + Vector3S.Cross(a.angularVelocity, ra);
            Vector3S bv = b.isDynamic ? b.velocity + Vector3S.Cross(b.angularVelocity, rb) : Vector3S.zero;
            Vector3S contactVelocity = bv - av;

            // -------------------- Impulse ---------------------
            var relativeNormalVelocity = Vector3S.Dot(contactVelocity, normal);

            var imp = this.normalMass * (relativeNormalVelocity + bias);
            var oldAccumulatedImpulse = this.accumulatedImpulse;
            this.accumulatedImpulse = MathS.Max(oldAccumulatedImpulse + imp, f32.zero);
            imp = this.accumulatedImpulse - oldAccumulatedImpulse;

            // -------------------- Friction ---------------------
            // Compute the frictional impulse and clamp it with the accumulated friction
            // var relativeTangentVelocity = Vector3S.Dot(contactVelocity, this.tangent);
            //var fric = this.tangentMass * relativeTangentVelocity;

            Vector3S normalVelocity = normal * Vector3S.Dot(contactVelocity, normal);
            Vector3S tangentialVelocity = contactVelocity - normalVelocity;
            f32 tangentMag = tangentialVelocity.sqrMagnitude;
            var tangent = tangentialVelocity.Normalize();

            var fric = this.tangentMass * Vector3S.Dot(contactVelocity, this.tangent);
            var maxFric = this.accumulatedImpulse * friction;
            var oldAccumulatedFriction = this.accumulatedFriction;
            this.accumulatedFriction = MathS.Clamp(oldAccumulatedFriction + fric, -maxFric, maxFric);
            fric = this.accumulatedFriction - oldAccumulatedFriction;

            // ------------------- Apply Impulses ------------------
            // Total impulse
            var totalImpulse = Vector3S.MultiplyAndAddBatch(normal, imp, this.tangent, fric);

            // Apply impulses to object A
            a.velocity -= (totalImpulse * a.inverseMass);
            a.angularVelocity -= (a.inertiaWorld * Vector3S.Cross(ra, totalImpulse));

            // Apply impulses to object B if dynamic
            if (b.isDynamic)
            {
                b.velocity += (totalImpulse * b.inverseMass);
                b.angularVelocity += (b.inertiaWorld * Vector3S.Cross(rb, totalImpulse));
            }
        }
    }
}