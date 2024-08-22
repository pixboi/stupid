using stupid.Maths;

namespace stupid.Colliders
{
    public struct ContactS
    {
        public readonly Collidable a, b;
        public readonly RigidbodyS ab, bb;
        public readonly Vector3S point, normal, ra, rb;
        public readonly f32 penetrationDepth, effectiveMass;
        public readonly int featureId;

        // Cached impulses for warm starting
        public f32 accumulatedImpulse, accumulatedFriction;

        public Vector3S aVelocity, aAngular, bVelocity, bAngular;
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
            this.effectiveMass = f32.zero;

            this.aVelocity = Vector3S.zero;
            this.aAngular = Vector3S.zero;

            this.bVelocity = Vector3S.zero;
            this.bAngular = Vector3S.zero;

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
            this.effectiveMass = effectiveMass > f32.zero ? f32.one / effectiveMass : f32.zero;
        }

        Vector3S CalculateRelativeContactVelocity()
        {
            Vector3S relativeVelocity = ab.velocity + Vector3S.Cross(ab.angularVelocity, this.ra);
            if (bb != null) relativeVelocity -= bb.velocity + Vector3S.Cross(bb.angularVelocity, this.rb);
            return relativeVelocity;
        }

        f32 CalculateSeparation(f32 slop)
        {
            // Compute the current contact separation for a sub-step
            Vector3S worldPointA = a.transform.position + this.ra;
            Vector3S worldPointB = b.transform.position + this.rb;
            f32 separation = Vector3S.Dot(worldPointB - worldPointA, this.normal) + this.penetrationDepth;
            return MathS.Max(separation - slop, f32.zero);
        }

        public void SolveImpulse(f32 deltaTime, in WorldSettings settings, bool bias = true)
        {
            Vector3S contactVelocity = CalculateRelativeContactVelocity();
            f32 vn = Vector3S.Dot(contactVelocity, this.normal);
            if (vn > f32.zero) return;

            var separation = CalculateSeparation(settings.DefaultContactOffset);

            //Accumulate
            f32 incrementalImpulse = -this.effectiveMass;
            if (bias)
            {
                f32 baum = -settings.Baumgartner * separation / deltaTime;
                incrementalImpulse *= (vn + baum);
            }
            else
            {
                incrementalImpulse *= vn;
            }

            f32 newAccumulatedImpulse = MathS.Max(f32.zero, this.accumulatedImpulse + incrementalImpulse);
            f32 appliedImpulse = newAccumulatedImpulse - this.accumulatedImpulse;
            this.accumulatedImpulse = newAccumulatedImpulse;

            Vector3S normalImpulse = this.normal * appliedImpulse;

            this.aVelocity += normalImpulse * ab.inverseMass;
            this.aAngular += ab.tensor.inertiaWorld * Vector3S.Cross(this.ra, normalImpulse);

            if (bb != null)
            {
                this.bVelocity -= normalImpulse * bb.inverseMass;
                this.bAngular -= bb.tensor.inertiaWorld * Vector3S.Cross(this.rb, normalImpulse);
            }
        }

        public void SolveFriction(f32 friction)
        {
            Vector3S contactVelocity = CalculateRelativeContactVelocity();
            f32 vn = Vector3S.Dot(contactVelocity, this.normal);
            if (vn > f32.zero) return;

            // Calculate the velocity along the normal
            Vector3S normalVelocity = this.normal * vn;

            // Calculate the tangential velocity (relative velocity minus the normal component)
            Vector3S tangentialVelocity = contactVelocity - normalVelocity;

            // Check if tangential velocity is significant to avoid unnecessary calculations
            if (tangentialVelocity.Magnitude() <= f32.epsilon) return;

            // Normalize the tangential velocity to get the friction direction (tangent)
            Vector3S tangent = tangentialVelocity.Normalize();

            // Calculate the friction denominator
            f32 invMassA = ab.inverseMass;
            f32 invMassB = bb != null ? bb.inverseMass : f32.zero;
            f32 frictionDenominator = invMassA + Vector3S.Dot(Vector3S.Cross(ab.tensor.inertiaWorld * Vector3S.Cross(this.ra, tangent), this.ra), tangent);
            if (bb != null) frictionDenominator += invMassB + Vector3S.Dot(Vector3S.Cross(bb.tensor.inertiaWorld * Vector3S.Cross(this.rb, tangent), this.rb), tangent);


            // Calculate the friction impulse scalar
            f32 frictionImpulseScalar = Vector3S.Dot(tangentialVelocity, tangent) / frictionDenominator;
            frictionImpulseScalar = -frictionImpulseScalar;

            // Compute the maximum friction impulse (Coulomb's law)
            f32 maxFrictionImpulse = this.accumulatedImpulse * friction;

            // Calculate the new friction impulse with clamping
            f32 oldAccumulatedFriction = this.accumulatedFriction;
            this.accumulatedFriction = MathS.Clamp(this.accumulatedFriction + frictionImpulseScalar, -maxFrictionImpulse, maxFrictionImpulse);

            // Calculate the actual friction impulse to apply
            f32 appliedFrictionImpulse = this.accumulatedFriction - oldAccumulatedFriction;
            Vector3S frictionImpulse = tangent * appliedFrictionImpulse;

            this.aVelocity += frictionImpulse * ab.inverseMass;
            this.aAngular += ab.tensor.inertiaWorld * Vector3S.Cross(this.ra, frictionImpulse);

            if (bb != null)
            {
                this.bVelocity -= frictionImpulse * bb.inverseMass;
                this.bAngular -= bb.tensor.inertiaWorld * Vector3S.Cross(this.rb, frictionImpulse);
            }
        }

        //The position solve is immediate
        public void SolvePosition(in f32 slop, in f32 positionCorrect)
        {
            f32 separation = CalculateSeparation(slop);

            //We could divide the position correction as well by iteration count
            Vector3S posCorrect = this.normal * separation * positionCorrect;
            a.transform.position += posCorrect;
            if (bb != null) b.transform.position -= posCorrect;
        }

        public void Actuate()
        {
            if (ab != null)
            {
                ab.velocity += aVelocity;
                ab.angularVelocity += aAngular;
            }

            if (bb != null)
            {
                bb.velocity += bVelocity;
                bb.angularVelocity += bAngular;
            }

            aVelocity = Vector3S.zero;
            aAngular = Vector3S.zero;
            bVelocity = Vector3S.zero;
            bAngular = Vector3S.zero;
        }
    }
}
