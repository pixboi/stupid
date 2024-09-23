using stupid.Maths;
using System.Runtime.CompilerServices;

namespace stupid.Constraints
{
    public struct ContactSlimNormal
    {
        public readonly Vector3S point;
        public readonly byte featureId;
        public f32 normalMass, accumulatedImpulse;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ContactSlimNormal(in ContactData data)
        {
            this.point = data.point;
            this.featureId = data.featureId;
            this.accumulatedImpulse = f32.zero;
            this.normalMass = f32.zero;
        }

        public bool Transfer(in ContactSlimNormal old)
        {
            if (old.featureId == this.featureId)
            {
                this.accumulatedImpulse = old.accumulatedImpulse;
                return true;
            }

            return false;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void CalculatePrestep(RigidbodyS a, RigidbodyS b, in Vector3S ra, in Vector3S rb, in Vector3S normal)
        {
            Vector3S raCrossNormal = Vector3S.Cross(ra, normal);
            f32 angularMassA = Vector3S.Dot(raCrossNormal, a.tensor.inertiaWorld * raCrossNormal);
            f32 effectiveMass = a.inverseMass + angularMassA;

            if (b != null)
            {
                Vector3S rbCrossNormal = Vector3S.Cross(rb, normal);
                f32 angularMassB = Vector3S.Dot(rbCrossNormal, b.tensor.inertiaWorld * rbCrossNormal);
                effectiveMass += b.inverseMass + angularMassB;
            }

            this.normalMass = effectiveMass > f32.zero ? f32.one / effectiveMass : f32.zero;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void WarmStart(in RigidbodyS a, in RigidbodyS b, in Vector3S ra, in Vector3S rb, in Vector3S normal)
        {
            Vector3S warmImpulse = normal * accumulatedImpulse;
            ContactSlim.ApplyImpulse(a, b, warmImpulse, ra, rb);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void SolveImpulse(in RigidbodyS a, in RigidbodyS b, in Vector3S rb, in Vector3S ra, in Vector3S normal, in f32 inverseDt, in WorldSettings settings, in f32 penetrationDepth, bool useBias = true)
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

            var contactVelocity = ContactSlim.CalculateContactVelocity(a, b, ra, rb);
            var vn = Vector3S.Dot(contactVelocity, normal);

            var incremental = -normalMass * (vn + bias);
            var newImpulse = MathS.Max(incremental + accumulatedImpulse, f32.zero);
            incremental = newImpulse - accumulatedImpulse;
            accumulatedImpulse = newImpulse;

            var impulse = normal * incremental;
            ContactSlim.ApplyImpulse(a, b, impulse, ra, rb);
        }
    }
}