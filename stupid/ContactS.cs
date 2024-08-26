using stupid.Maths;
using System.Runtime;

namespace stupid.Colliders
{
    public struct ContactS
    {
        public readonly Collidable a, b;
        public readonly RigidbodyS ab, bb;

        //Some seem to use 2 points, point on a, and point on b, this could improve stability?
        //Some use max 4 contacts per manifold, and solve them like 1 = biggest pen, 2 = furthest from 1, 3 = furthest to a line segment between 1-2
        public readonly Vector3S point, normal, ra, rb;
        public f32 massNormal, massTangent;
        public readonly f32 penetrationDepth;
        public readonly int featureId;

        // Cached impulses for warm starting
        public f32 accumulatedImpulse, accumulatedFriction;
        public ContactS(Vector3S point, Vector3S normal, f32 penetrationDepth, Collidable a, Collidable b, int featureId = 0)
        {
            this.a = a;
            this.b = b;
            this.point = point;
            this.normal = normal;
            this.penetrationDepth = penetrationDepth;
            this.ra = this.point - a.transform.position;
            this.rb = this.point - b.transform.position;
            this.featureId = featureId;

            this.accumulatedImpulse = f32.zero;
            this.accumulatedFriction = f32.zero;
            this.massNormal = f32.zero;
            this.massTangent = f32.zero;

            if (!a.isDynamic && !b.isDynamic)
            {
                this.ab = null;
                this.bb = null;
                return;
            }

            this.ab = a.isDynamic ? (RigidbodyS)this.a : null;
            this.bb = b.isDynamic ? (RigidbodyS)this.b : null;

            //This effective mass could be updated, if we intend to use substepping with the contact point update. the tensor is affected by the 
            //integration, also the localPoint should come now from a matrix calc, since we have rotated bit

            f32 invMassA = ab.inverseMass;
            f32 invMassB = bb != null ? bb.inverseMass : f32.zero;

            // Linear effective mass
            f32 effectiveMass = invMassA + invMassB;

            // Angular effective mass for body A
            Vector3S raCrossNormal = Vector3S.Cross(this.ra, this.normal);
            f32 angularMassA = Vector3S.Dot(raCrossNormal, ab.tensor.inertiaWorld * raCrossNormal);
            effectiveMass += angularMassA;

            // Angular effective mass for body B
            if (bb != null)
            {
                Vector3S rbCrossNormal = Vector3S.Cross(this.rb, this.normal);
                f32 angularMassB = Vector3S.Dot(rbCrossNormal, bb.tensor.inertiaWorld * rbCrossNormal);
                effectiveMass += angularMassB;
            }

            // Invert to get effective mass
            this.massNormal = effectiveMass > f32.zero ? f32.one / effectiveMass : f32.zero;
        }

        f32 CalculateSeparation(f32 slop)
        {
            // Compute the current contact separation for a sub-step
            Vector3S worldPointA = a.transform.position + this.ra;
            Vector3S worldPointB = b.transform.position + this.rb;
            f32 separation = Vector3S.Dot(worldPointB - worldPointA, this.normal) + this.penetrationDepth;
            return MathS.Min(separation + slop, f32.zero);
        }

        public void SolveImpulse(f32 inverseDt, in WorldSettings settings, bool bias = true)
        {
            Vector3S contactVelocity = ab.velocity + Vector3S.Cross(ab.angularVelocity, this.ra);
            if (bb != null) contactVelocity -= bb.velocity + Vector3S.Cross(bb.angularVelocity, this.rb);

            f32 vn = Vector3S.Dot(contactVelocity, this.normal);
            if (vn > f32.zero) return;

            f32 baum = f32.zero;

            // bias = S2_MAX(s2_baumgarte * inv_h * S2_MIN(0.0f, cp->separation + s2_linearSlop), -s2_maxBaumgarteVelocity);
            if (bias)
            {
                var separation = CalculateSeparation(settings.DefaultContactOffset);
                baum = settings.Baumgartner * separation * inverseDt;
            }

            //Accumulate
            f32 impulse = -this.massNormal * (vn + baum);
            f32 newImpulse = MathS.Max(impulse + this.accumulatedImpulse, f32.zero);
            impulse = newImpulse - this.accumulatedImpulse;
            this.accumulatedImpulse = newImpulse;

            Vector3S normalImpulse = this.normal * impulse;
            ab.velocity += normalImpulse * ab.inverseMass;
            ab.angularVelocity += ab.tensor.inertiaWorld * Vector3S.Cross(this.ra, normalImpulse);

            if (bb != null)
            {
                bb.velocity -= normalImpulse * bb.inverseMass;
                bb.angularVelocity -= bb.tensor.inertiaWorld * Vector3S.Cross(this.rb, normalImpulse);
            }
        }

        public void SolveFriction(in WorldSettings settings, f32 friction)
        {
            Vector3S contactVelocity = ab.velocity + Vector3S.Cross(ab.angularVelocity, this.ra);
            if (bb != null) contactVelocity -= bb.velocity + Vector3S.Cross(bb.angularVelocity, this.rb);

            f32 vn = Vector3S.Dot(contactVelocity, this.normal);

            // Calculate the velocity along the normal
            Vector3S normalVelocity = this.normal * vn;
            Vector3S tangentialVelocity = contactVelocity - normalVelocity;
            if (tangentialVelocity.sqrMagnitude == f32.zero) return;


            Vector3S tangent = tangentialVelocity.Normalize();

            // Calculate the friction denominator
            f32 invMassA = ab.inverseMass;
            f32 invMassB = bb != null ? bb.inverseMass : f32.zero;
            f32 tangentMass = invMassA + Vector3S.Dot(Vector3S.Cross(ab.tensor.inertiaWorld * Vector3S.Cross(this.ra, tangent), this.ra), tangent);
            if (bb != null) tangentMass += invMassB + Vector3S.Dot(Vector3S.Cross(bb.tensor.inertiaWorld * Vector3S.Cross(this.rb, tangent), this.rb), tangent);

            // Calculate the friction impulse scalar
            f32 frictionImpulseScalar = Vector3S.Dot(tangentialVelocity, tangent) / tangentMass;
            frictionImpulseScalar = -frictionImpulseScalar;

            // Compute the maximum friction impulse (Coulomb's law)
            f32 coulombMax = this.accumulatedImpulse * friction;
            f32 oldFriction = this.accumulatedFriction;
            this.accumulatedFriction = MathS.Clamp(oldFriction + frictionImpulseScalar, -coulombMax, coulombMax);
            f32 appliedFrictionImpulse = this.accumulatedFriction - oldFriction;

            Vector3S frictionImpulse = tangent * appliedFrictionImpulse;
            ab.velocity += frictionImpulse * ab.inverseMass;
            ab.angularVelocity += ab.tensor.inertiaWorld * Vector3S.Cross(this.ra, frictionImpulse);

            if (bb != null)
            {
                bb.velocity -= frictionImpulse * bb.inverseMass;
                bb.angularVelocity -= bb.tensor.inertiaWorld * Vector3S.Cross(this.rb, frictionImpulse);
            }
        }

    }
}
