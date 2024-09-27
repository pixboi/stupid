using stupid;
using stupid.Maths;
using System.Drawing;
using System.Runtime.CompilerServices;

namespace stupid.Constraints
{
    public struct ContactSlim
    {
        public readonly Vector3S ra, rb; // 2 * 24 = 48
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
            rb = data.point - b.transform.position;
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
            Vector3S warmImpulse = normal * accumulatedImpulse + tangent * accumulatedFriction;
            ApplyImpulse(ref a, ref b, warmImpulse, ra, rb);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void SolveAll(ref RigidbodyData a, ref RigidbodyData b, in Vector3S normal, in f32 bias, in f32 friction)
        {
            // Precompute velocities
            Vector3S av = Vector3S.CrossAdd(a.velocity, a.angularVelocity, ra);
            Vector3S bv = b.isDynamic ? Vector3S.CrossAdd(b.velocity, b.angularVelocity, rb) : Vector3S.zero;
            Vector3S contactVelocity = bv - av;

            // -------------------- Impulse ---------------------
            // Relative normal velocity
            var relativeNormalVelocity = Vector3S.Dot(contactVelocity, normal);

            // Compute the new impulse directly and clamp it
            var imp = this.normalMass * (relativeNormalVelocity + bias);
            var oldAccumulatedImpulse = this.accumulatedImpulse;
            this.accumulatedImpulse = MathS.Max(oldAccumulatedImpulse + imp, f32.zero);
            imp = this.accumulatedImpulse - oldAccumulatedImpulse;

            // -------------------- Friction ---------------------
            // Tangent velocity calculation
            // var relativeTangentVelocity = Vector3S.Dot(contactVelocity, this.tangent);

            // Compute the frictional impulse and clamp it with the accumulated friction
            //var fric = this.tangentMass * relativeTangentVelocity;
            var fric = Vector3S.DotMultiply(contactVelocity, this.tangent, this.tangentMass);
            var maxFric = this.accumulatedImpulse * friction;
            var oldAccumulatedFriction = this.accumulatedFriction;
            this.accumulatedFriction = MathS.Clamp(oldAccumulatedFriction + fric, -maxFric, maxFric);
            fric = this.accumulatedFriction - oldAccumulatedFriction;

            // ------------------- Apply Impulses ------------------
            // Total impulse
            var totalImpulse = Vector3S.MultiplyAndAddBatch(normal, imp, this.tangent, fric);

            // Apply impulses to object A
            a.velocity.SubtractInPlace(totalImpulse * a.inverseMass);
            a.angularVelocity.SubtractInPlace(a.inertiaWorld * Vector3S.Cross(ra, totalImpulse));

            // Apply impulses to object B if dynamic
            if (b.isDynamic)
            {
                b.velocity.AddInPlace(totalImpulse * b.inverseMass);
                b.angularVelocity.AddInPlace(b.inertiaWorld * Vector3S.Cross(rb, totalImpulse));
            }
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void SolveImpulse(ref RigidbodyData a, ref RigidbodyData b, in Vector3S normal, in f32 bias)
        {
            var av = a.velocity + Vector3S.Cross(a.angularVelocity, ra);
            var bv = b.isDynamic ? b.velocity + Vector3S.Cross(b.angularVelocity, rb) : Vector3S.zero;
            var contactVelocity = bv - av;

            var vn = Vector3S.Dot(contactVelocity, normal);

            var incremental = -this.normalMass * (vn + bias);
            var newImpulse = MathS.Max(incremental + this.accumulatedImpulse, f32.zero);
            incremental = newImpulse - this.accumulatedImpulse;
            this.accumulatedImpulse = newImpulse;

            var impulse = normal * incremental;

            a.velocity -= impulse * a.inverseMass; // A moves away
            a.angularVelocity -= a.inertiaWorld * Vector3S.Cross(ra, impulse);

            if (b.isDynamic)
            {
                b.velocity += impulse * b.inverseMass; // B moves along normal
                b.angularVelocity += b.inertiaWorld * Vector3S.Cross(rb, impulse);
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void SolveFriction(ref RigidbodyData a, ref RigidbodyData b, in f32 friction)
        {
            var av = a.velocity + Vector3S.Cross(a.angularVelocity, ra);
            var bv = b.isDynamic ? b.velocity + Vector3S.Cross(b.angularVelocity, rb) : Vector3S.zero;
            var contactVelocity = bv - av;

            var vt = Vector3S.Dot(contactVelocity, this.tangent);
            var incrementalFriction = -this.tangentMass * vt;

            var couloumbMax = this.accumulatedImpulse * friction;
            var newImpulse = MathS.Clamp(this.accumulatedFriction + incrementalFriction, -couloumbMax, couloumbMax);
            incrementalFriction = newImpulse - this.accumulatedFriction;
            this.accumulatedFriction = newImpulse;

            var impulse = this.tangent * incrementalFriction;

            a.velocity -= impulse * a.inverseMass; // A moves away
            a.angularVelocity -= a.inertiaWorld * Vector3S.Cross(ra, impulse);

            if (b.isDynamic)
            {
                b.velocity += impulse * b.inverseMass; // B moves along normal
                b.angularVelocity += b.inertiaWorld * Vector3S.Cross(rb, impulse);
            }
        }


    }
}