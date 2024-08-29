using stupid.Maths;
using System.Runtime;

namespace stupid.Colliders
{
    public readonly struct ContactS
    {
        //Some seem to use 2 points, point on a, and point on b, this could improve stability?
        //Some use max 4 contacts per manifold, and solve them like 1 = biggest pen, 2 = furthest from 1, 3 = furthest to a line segment between 1-2
        public readonly Vector3S point, normal, tangent, ra, rb;
        public readonly f32 massNormal;
        public readonly f32 penetrationDepth;
        public readonly int featureId;

        public ContactS(Vector3S point, Vector3S normal, f32 penetrationDepth, Collidable a, Collidable b, int featureId = 0)
        {
            this.point = point;
            this.normal = normal;
            this.penetrationDepth = penetrationDepth;
            this.ra = this.point - a.transform.position;
            this.rb = this.point - b.transform.position;
            this.featureId = featureId;
            this.massNormal = f32.zero;

            var t1 = Vector3S.Cross(normal, Vector3S.forward);
            var t2 = Vector3S.Cross(normal, Vector3S.right);

            if (t1.sqrMagnitude > t2.sqrMagnitude)
            {
                this.tangent = t1.NormalizeInPlace();
            }
            else this.tangent = t2.NormalizeInPlace();

            if (!a.isDynamic && !b.isDynamic) return;

            var ab = (RigidbodyS)a;
            var bb = b.isDynamic ? (RigidbodyS)b : null;

            // Invert to get effective mass
            this.massNormal = CalculateMassNormal(ab, bb);
        }

        public f32 CalculateMassNormal(RigidbodyS ab, RigidbodyS bb)
        {
            f32 invMassA = ab.inverseMass;
            f32 invMassB = bb != null ? bb.inverseMass : f32.zero;

            // Linear effective mass
            f32 effectiveMass = invMassA + invMassB;

            Vector3S raCrossNormal = Vector3S.Cross(this.ra, this.normal);
            f32 angularMassA = Vector3S.Dot(raCrossNormal, ab.tensor.inertiaWorld * raCrossNormal);
            effectiveMass += angularMassA;

            if (bb != null)
            {
                Vector3S rbCrossNormal = Vector3S.Cross(this.rb, this.normal);
                f32 angularMassB = Vector3S.Dot(rbCrossNormal, bb.tensor.inertiaWorld * rbCrossNormal);
                effectiveMass += angularMassB;
            }

            var final = effectiveMass > f32.zero ? f32.one / effectiveMass : f32.zero;

            return final;
        }

        public f32 CalculateSeparation(in Vector3S aPosition, in Vector3S bPosition, in f32 slop)
        {
            // Compute the current contact separation for a sub-step
            Vector3S worldPointA = aPosition + this.ra;
            Vector3S worldPointB = bPosition + this.rb;
            f32 separation = Vector3S.Dot(worldPointB - worldPointA, this.normal) + this.penetrationDepth;
            return MathS.Min(f32.zero, separation + slop);
        }

        public void SolveImpulseStatic(in RigidbodyS a, in Collidable b, ref f32 accumulatedImpulse, in f32 inverseDt, in WorldSettings settings, bool bias = true)
        {
            Vector3S contactVelocity = a.velocity + Vector3S.Cross(a.angularVelocity, this.ra);
            f32 vn = Vector3S.Dot(contactVelocity, this.normal);
            if (vn > f32.zero) return;

            f32 baum = f32.zero;

            if (bias)
            {
                var separation = CalculateSeparation(a.transform.position, b.transform.position, settings.DefaultContactOffset);
                baum = settings.Baumgartner * separation * inverseDt;
            }

            //Accumulate
            f32 impulse = -this.massNormal * (vn + baum);
            f32 newImpulse = MathS.Max(impulse + accumulatedImpulse, f32.zero);
            impulse = newImpulse - accumulatedImpulse;
            accumulatedImpulse = newImpulse;

            Vector3S normalImpulse = this.normal * impulse;
            a.velocity += normalImpulse * a.inverseMass;
            a.angularVelocity += a.tensor.inertiaWorld * Vector3S.Cross(this.ra, normalImpulse);
        }

        public void SolveImpulse(in RigidbodyS a, in RigidbodyS b, ref f32 accumulatedImpulse, in f32 inverseDt, in WorldSettings settings, bool bias = true)
        {
            Vector3S contactVelocity = a.velocity + Vector3S.Cross(a.angularVelocity, this.ra);
            contactVelocity -= b.velocity + Vector3S.Cross(b.angularVelocity, this.rb);

            f32 vn = Vector3S.Dot(contactVelocity, this.normal);
            if (vn > f32.zero) return;

            f32 baum = f32.zero;

            if (bias)
            {
                var separation = CalculateSeparation(a.transform.position, b.transform.position, settings.DefaultContactOffset);
                baum = settings.Baumgartner * separation * inverseDt;
            }

            //Accumulate
            f32 impulse = -this.massNormal * (vn + baum);
            f32 newImpulse = MathS.Max(impulse + accumulatedImpulse, f32.zero);
            impulse = newImpulse - accumulatedImpulse;
            accumulatedImpulse = newImpulse;

            Vector3S normalImpulse = this.normal * impulse;
            a.velocity += normalImpulse * a.inverseMass;
            a.angularVelocity += a.tensor.inertiaWorld * Vector3S.Cross(this.ra, normalImpulse);

            b.velocity -= normalImpulse * b.inverseMass;
            b.angularVelocity -= b.tensor.inertiaWorld * Vector3S.Cross(this.rb, normalImpulse);
        }

        public void SolveFrictionStatic(in RigidbodyS a, in Collidable b, in f32 accumulatedImpulse, ref f32 accumulatedFriction, in WorldSettings settings, f32 friction)
        {
            Vector3S contactVelocity = a.velocity + Vector3S.Cross(a.angularVelocity, this.ra);

            f32 vn = Vector3S.Dot(contactVelocity, this.normal);

            // Calculate the velocity along the normal
            Vector3S normalVelocity = this.normal * vn;
            Vector3S tangentialVelocity = contactVelocity - normalVelocity;
            if (tangentialVelocity.sqrMagnitude < f32.epsilon) return;

            Vector3S tangent = tangentialVelocity.Normalize();

            // Calculate the friction denominator
            f32 invMassA = a.inverseMass;
            f32 tangentMass = invMassA + Vector3S.Dot(Vector3S.Cross(a.tensor.inertiaWorld * Vector3S.Cross(this.ra, tangent), this.ra), tangent);

            // Calculate the friction impulse scalar
            f32 frictionImpulseScalar = Vector3S.Dot(tangentialVelocity, tangent) / tangentMass;
            frictionImpulseScalar = -frictionImpulseScalar;

            // Compute the maximum friction impulse (Coulomb's law)
            f32 coulombMax = accumulatedImpulse * friction;
            f32 oldFriction = accumulatedFriction;
            accumulatedFriction = MathS.Clamp(oldFriction + frictionImpulseScalar, -coulombMax, coulombMax);
            f32 appliedFrictionImpulse = accumulatedFriction - oldFriction;

            Vector3S frictionImpulse = tangent * appliedFrictionImpulse;
            a.velocity += frictionImpulse * a.inverseMass;
            a.angularVelocity += a.tensor.inertiaWorld * Vector3S.Cross(this.ra, frictionImpulse);
        }

        public void SolveFriction(in RigidbodyS a, in RigidbodyS b, in f32 accumulatedImpulse, ref f32 accumulatedFriction, in WorldSettings settings, f32 friction)
        {
            Vector3S contactVelocity = a.velocity + Vector3S.Cross(a.angularVelocity, this.ra);
            contactVelocity -= b.velocity + Vector3S.Cross(b.angularVelocity, this.rb);

            f32 vn = Vector3S.Dot(contactVelocity, this.normal);

            // Calculate the velocity along the normal
            Vector3S normalVelocity = this.normal * vn;
            Vector3S tangentialVelocity = contactVelocity - normalVelocity;
            if (tangentialVelocity.sqrMagnitude < f32.epsilon) return;

            Vector3S tangent = tangentialVelocity.Normalize();

            // Calculate the friction denominator
            f32 invMassA = a.inverseMass;
            f32 invMassB = b.inverseMass;
            f32 tangentMass = invMassA + Vector3S.Dot(Vector3S.Cross(a.tensor.inertiaWorld * Vector3S.Cross(this.ra, tangent), this.ra), tangent);
            tangentMass += invMassB + Vector3S.Dot(Vector3S.Cross(b.tensor.inertiaWorld * Vector3S.Cross(this.rb, tangent), this.rb), tangent);

            // Calculate the friction impulse scalar
            f32 frictionImpulseScalar = Vector3S.Dot(tangentialVelocity, tangent) / tangentMass;
            frictionImpulseScalar = -frictionImpulseScalar;

            // Compute the maximum friction impulse (Coulomb's law)
            f32 coulombMax = accumulatedImpulse * friction;
            f32 oldFriction = accumulatedFriction;
            accumulatedFriction = MathS.Clamp(oldFriction + frictionImpulseScalar, -coulombMax, coulombMax);
            f32 appliedFrictionImpulse = accumulatedFriction - oldFriction;

            Vector3S frictionImpulse = tangent * appliedFrictionImpulse;
            a.velocity += frictionImpulse * a.inverseMass;
            a.angularVelocity += a.tensor.inertiaWorld * Vector3S.Cross(this.ra, frictionImpulse);

            b.velocity -= frictionImpulse * b.inverseMass;
            b.angularVelocity -= b.tensor.inertiaWorld * Vector3S.Cross(this.rb, frictionImpulse);
        }

    }
}
