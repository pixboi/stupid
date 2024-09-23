using stupid;
using stupid.Maths;
using System.Runtime.CompilerServices;

namespace stupid.Constraints
{
    public struct ContactSlim
    {
        public readonly Vector3S point;
        public Vector3S tangent, ra, rb;
        public f32 normalMass, tangentMass;
        public f32 accumulatedImpulse, accumulatedFriction;
        public readonly byte featureId;

        public ContactSlim(in ContactData data)
        {
            point = data.point;
            featureId = data.featureId;
            accumulatedImpulse = f32.zero;
            normalMass = f32.zero;
            tangentMass = f32.zero;
            tangent = Vector3S.zero;
            accumulatedFriction = f32.zero;
            this.ra = Vector3S.zero;
            this.rb = Vector3S.zero;
        }

        public void CalculatePrestep(RigidbodyS a, RigidbodyS b, in ContactManifoldSlim manifold)
        {
            this.ra = point - a.transform.position;
            this.rb = b != null ? point - b.transform.position : Vector3S.zero;

            Vector3S raCrossNormal = Vector3S.Cross(ra, manifold.normal);
            f32 angularMassA = Vector3S.Dot(raCrossNormal, a.tensor.inertiaWorld * raCrossNormal);
            f32 effectiveMass = a.inverseMass + angularMassA;

            if (b != null)
            {
                Vector3S rbCrossNormal = Vector3S.Cross(rb, manifold.normal);
                f32 angularMassB = Vector3S.Dot(rbCrossNormal, b.tensor.inertiaWorld * rbCrossNormal);
                effectiveMass += b.inverseMass + angularMassB;
            }

            normalMass = effectiveMass > f32.zero ? f32.one / effectiveMass : f32.zero;

            // Calculate relative velocity at the contact point
            var contactVelocity = CalculateContactVelocity(a, b, ra, rb);
            Vector3S normalVelocity = manifold.normal * Vector3S.Dot(contactVelocity, manifold.normal);
            Vector3S tangentialVelocity = contactVelocity - normalVelocity;
            f32 tangentMag = tangentialVelocity.sqrMagnitude;

            //In retain, the previous tangent is stored IN THIS.TANGENT!
            var oldTangent = tangent;
            var newTangent = tangentialVelocity.Normalize();

            //Max at one, at one, it should prefer the oldTangent
            // var dirSimilarity = (Vector3S.Dot(oldTangent, newTangent) + f32.one) * f32.half;
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
        public void WarmStart(in Vector3S normal, in RigidbodyS a, in RigidbodyS b)
        {
            Vector3S warmImpulse = normal * accumulatedImpulse + tangent * accumulatedFriction;
            ApplyImpulse(a, b, warmImpulse, ra, rb);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static public Vector3S CalculateContactVelocity(in RigidbodyS a, in RigidbodyS b, in Vector3S ra, in Vector3S rb)
        {
            var av = a.velocity + Vector3S.Cross(a.angularVelocity, ra);
            var bv = b != null ? b.velocity + Vector3S.Cross(b.angularVelocity, rb) : Vector3S.zero;
            return bv - av;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void SolveImpulse(in RigidbodyS a, in RigidbodyS b, in f32 inverseDt, in WorldSettings settings, in f32 penetrationDepth, in Vector3S normal, bool useBias = true)
        {
            f32 bias = f32.zero;
            if (penetrationDepth > f32.zero)
            {
                bias = penetrationDepth * inverseDt;
            }
            else if (useBias)
            {
                var separation = MathS.Min(f32.zero, penetrationDepth + settings.DefaultContactOffset);
                bias = MathS.Max(settings.Baumgartner * separation * inverseDt, -settings.DefaultMaxDepenetrationVelocity);
            }

            var av = a.velocity + Vector3S.Cross(a.angularVelocity, ra);
            var bv = b != null ? b.velocity + Vector3S.Cross(b.angularVelocity, rb) : Vector3S.zero;
            var contactVelocity = bv - av;

            var vn = Vector3S.Dot(contactVelocity, normal);

            //var incremental = -normalMass * (vn + bias);
            var incremental = f32.AddAndMultiply(vn, bias, -normalMass);
            var newImpulse = MathS.Max(incremental + accumulatedImpulse, f32.zero);
            incremental = newImpulse - accumulatedImpulse;
            accumulatedImpulse = newImpulse;

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
        public void SolveFriction(in RigidbodyS a, in RigidbodyS b, in f32 friction)
        {
            var av = a.velocity + Vector3S.Cross(a.angularVelocity, ra);
            var bv = b != null ? b.velocity + Vector3S.Cross(b.angularVelocity, rb) : Vector3S.zero;
            var contactVelocity = bv - av;

            var vt = Vector3S.Dot(contactVelocity, tangent);
            var incrementalFriction = -tangentMass * vt;

            var couloumbMax = accumulatedImpulse * friction;
            var newImpulse = MathS.Clamp(accumulatedFriction + incrementalFriction, -couloumbMax, couloumbMax);
            incrementalFriction = newImpulse - accumulatedFriction;
            accumulatedFriction = newImpulse;

            var impulse = tangent * incrementalFriction;

            a.velocity -= impulse * a.inverseMass; // A moves away
            a.angularVelocity -= a.tensor.inertiaWorld * Vector3S.Cross(ra, impulse);

            if (b != null)
            {
                b.velocity += impulse * b.inverseMass; // B moves along normal
                b.angularVelocity += b.tensor.inertiaWorld * Vector3S.Cross(rb, impulse);
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void ApplyImpulse(in RigidbodyS a, in RigidbodyS b, in Vector3S impulse, in Vector3S ra, in Vector3S rb)
        {
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