using stupid.Maths;
using System;

namespace stupid.Colliders
{
    public struct ContactS
    {
        public Collidable a, b;
        public Vector3S worldPoint, localPointA, localPointB;
        public Vector3S normal;
        public f32 penetrationDepth;

        // Cached impulses for warm starting
        public f32 cachedNormalImpulse;
        public f32 cachedFrictionImpulse;
        public Vector3S cachedImpulse;

        // Precomputed values
        public Vector3S relativeVelocity;
        public f32 effectiveMass;
        public f32 restitution;
        public f32 frictionDenominator;

        private const int MaxIterations = 8;
        private const float Tolerance = 1e-5f;
        private static readonly f32 BaumgarteFactor = f32.FromFloat(0.2f);

        public ContactS(Collidable a, Collidable b, Vector3S point, Vector3S normal, f32 penetrationDepth)
        {
            this.a = a;
            this.b = b;
            this.worldPoint = point;
            this.localPointA = point - a.transform.position;
            this.localPointB = point - b.transform.position;
            this.normal = normal;
            this.penetrationDepth = penetrationDepth;

            // Initialize cached impulses to zero
            this.cachedNormalImpulse = f32.zero;
            this.cachedFrictionImpulse = f32.zero;
            this.cachedImpulse = Vector3S.zero;

            // Initialize precomputed values
            this.relativeVelocity = Vector3S.zero;
            this.effectiveMass = f32.zero;
            this.restitution = f32.zero;
            this.frictionDenominator = f32.zero;
        }

        public void CalculateInternals()
        {
            RigidbodyS bodyA = (RigidbodyS)a;
            RigidbodyS bodyB = b.isDynamic ? (RigidbodyS)b : null;

            // Precompute all necessary values
            this.relativeVelocity = CalculateRelativeVelocityAtContact(bodyA, bodyB);
            this.effectiveMass = CalculateEffectiveMass(bodyA, bodyB);
            this.restitution = CalculateRestitution(bodyA, bodyB);
            this.frictionDenominator = CalculateFrictionDenominator(bodyA, bodyB);
        }

        private Vector3S CalculateRelativeVelocityAtContact(RigidbodyS bodyA, RigidbodyS bodyB)
        {
            Vector3S relativeVelocity = bodyA.velocity + Vector3S.Cross(bodyA.angularVelocity, this.localPointA);
            if (bodyB != null)
            {
                relativeVelocity -= bodyB.velocity + Vector3S.Cross(bodyB.angularVelocity, this.localPointB);
            }
            return relativeVelocity;
        }

        private f32 CalculateEffectiveMass(RigidbodyS bodyA, RigidbodyS bodyB)
        {
            f32 invMassA = bodyA.inverseMass;
            f32 invMassB = bodyB != null ? bodyB.inverseMass : f32.zero;

            Vector3S ra = this.localPointA;
            Vector3S rb = this.localPointB;

            f32 effectiveMass = invMassA + Vector3S.Dot(Vector3S.Cross(bodyA.tensor.inertiaWorld * Vector3S.Cross(ra, this.normal), ra), this.normal);
            if (bodyB != null)
            {
                effectiveMass += invMassB + Vector3S.Dot(Vector3S.Cross(bodyB.tensor.inertiaWorld * Vector3S.Cross(rb, this.normal), rb), this.normal);
            }

            return effectiveMass;
        }

        private f32 CalculateRestitution(RigidbodyS bodyA, RigidbodyS bodyB)
        {
            f32 restitutionB = bodyB != null ? bodyB.material.restitution : f32.zero;
            return MathS.Min(bodyA.material.restitution, restitutionB);
        }

        private f32 CalculateFrictionDenominator(RigidbodyS bodyA, RigidbodyS bodyB)
        {
            f32 invMassA = bodyA.inverseMass;
            f32 invMassB = bodyB != null ? bodyB.inverseMass : f32.zero;

            Vector3S ra = this.localPointA;
            Vector3S rb = this.localPointB;

            f32 frictionDenominator = invMassA + Vector3S.Dot(Vector3S.Cross(bodyA.tensor.inertiaWorld * Vector3S.Cross(ra, this.normal), ra), this.normal);
            if (bodyB != null)
            {
                frictionDenominator += invMassB + Vector3S.Dot(Vector3S.Cross(bodyB.tensor.inertiaWorld * Vector3S.Cross(rb, this.normal), rb), this.normal);
            }

            return frictionDenominator;
        }

        public void ResolveContact(f32 deltaTime)
        {
            RigidbodyS bodyA = (RigidbodyS)a;
            RigidbodyS bodyB = b.isDynamic ? (RigidbodyS)b : null;

            // Iteratively solve the contact
            f32 accumulatedImpulse = this.cachedNormalImpulse;
            for (int iter = 0; iter < MaxIterations; iter++)
            {
                f32 oldImpulse = accumulatedImpulse;

                // Calculate relative velocity at contact point
                Vector3S relativeVelocityAtContact = CalculateRelativeVelocityAtContact(bodyA, bodyB);

                // Calculate velocity along normal
                f32 velocityAlongNormal = Vector3S.Dot(relativeVelocityAtContact, this.normal);
                if (velocityAlongNormal > f32.zero) return; // Moving away, no impulse required

                // Baumgarte stabilization factor
                f32 baumFactor = BaumgarteFactor * this.penetrationDepth / deltaTime;

                // Calculate impulse
                f32 impulse = -(f32.one + this.restitution) * velocityAlongNormal / this.effectiveMass + baumFactor;
                accumulatedImpulse = MathS.Max(f32.zero, oldImpulse + impulse); // Project impulse to be non-negative
                f32 appliedImpulse = accumulatedImpulse - oldImpulse;

                // Apply impulse
                ApplyImpulse(bodyA, bodyB, appliedImpulse);

                // Check for convergence
                if (Math.Abs((float)appliedImpulse) < Tolerance) break;
            }

            // Update cached normal impulse
            this.cachedNormalImpulse = accumulatedImpulse;

            // Handle friction
            ResolveFriction(bodyA, bodyB);
        }

        private void ApplyImpulse(RigidbodyS bodyA, RigidbodyS bodyB, f32 impulse)
        {
            Vector3S normalImpulse = this.normal * impulse;

            bodyA.velocity += normalImpulse * bodyA.inverseMass;
            bodyA.angularVelocity += bodyA.tensor.inertiaWorld * Vector3S.Cross(this.localPointA, normalImpulse);

            if (bodyB != null)
            {
                bodyB.velocity -= normalImpulse * bodyB.inverseMass;
                bodyB.angularVelocity -= bodyB.tensor.inertiaWorld * Vector3S.Cross(this.localPointB, normalImpulse);
            }
        }

        private void ResolveFriction(RigidbodyS bodyA, RigidbodyS bodyB)
        {
            Vector3S relativeVelocityAtContact = CalculateRelativeVelocityAtContact(bodyA, bodyB);
            Vector3S tangentialVelocity = relativeVelocityAtContact - this.normal * Vector3S.Dot(relativeVelocityAtContact, this.normal);

            if (tangentialVelocity.Magnitude() > f32.zero)
            {
                Vector3S tangent = tangentialVelocity.Normalize();
                f32 frictionImpulseScalar = Vector3S.Dot(relativeVelocityAtContact, tangent) / this.frictionDenominator;
                frictionImpulseScalar = -frictionImpulseScalar;

                f32 effectiveFriction = bodyA.material.staticFriction;
                if (bodyB != null)
                {
                    effectiveFriction = (effectiveFriction + bodyB.material.staticFriction) * f32.FromFloat(0.5f);
                }

                Vector3S frictionImpulse = tangent * frictionImpulseScalar;
                f32 maxFrictionImpulse = this.cachedNormalImpulse * effectiveFriction;
                if (frictionImpulse.Magnitude() > maxFrictionImpulse)
                {
                    frictionImpulse = frictionImpulse.Normalize() * maxFrictionImpulse;
                }

                // Apply friction impulses
                bodyA.velocity += frictionImpulse * bodyA.inverseMass;
                bodyA.angularVelocity += bodyA.tensor.inertiaWorld * Vector3S.Cross(this.localPointA, frictionImpulse);

                if (bodyB != null)
                {
                    bodyB.velocity -= frictionImpulse * bodyB.inverseMass;
                    bodyB.angularVelocity -= bodyB.tensor.inertiaWorld * Vector3S.Cross(this.localPointB, frictionImpulse);
                }

                // Update cached friction impulse
                this.cachedFrictionImpulse = frictionImpulseScalar;
            }
        }
    }
}
