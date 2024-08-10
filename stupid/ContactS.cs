using System;
using stupid.Maths;

namespace stupid.Colliders
{
    public struct ContactS
    {
        public Collidable a, b;
        public Vector3S point, normal, localOffsetA, localOffsetB;
        public f32 penetrationDepth;
        public readonly f32 friction, restitution;

        // Cached impulses for warm starting
        public f32 cachedNormalImpulse;

        private static readonly f32 Tolerance = f32.epsilon;
        private static readonly f32 BaumgarteFactor = f32.FromFloat(0.2f);
        private static readonly f32 PositionCorrectionFactor = f32.FromFloat(0.2f); // Position correction factor

        public ContactS(Collidable a, Collidable b, Vector3S point, Vector3S normal, f32 penetrationDepth)
        {
            this.a = a;
            this.b = b;
            this.point = point;
            this.normal = normal;
            this.penetrationDepth = penetrationDepth;
            this.friction = (a.material.staticFriction + b.material.staticFriction) * f32.half;
            this.restitution = (a.material.restitution + b.material.restitution) * f32.half;

            // Initialize cached impulses to zero
            this.cachedNormalImpulse = f32.zero;

            // Compute initial local offsets, these are invalid if point is not set
            this.localOffsetA = this.point - a.transform.position; // Local offset from body A's position
            this.localOffsetB = this.point - b.transform.position; // Local offset from body B's position
        }

        // Recalculate local offsets based on the current world position of the colliders
        public void ComputeInternals()
        {
            this.localOffsetA = this.point - a.transform.position; // Local offset from body A's position
            this.localOffsetB = this.point - b.transform.position; // Local offset from body B's position
        }

        public void ResolveContact(f32 deltaTime, int maxIterations = 1)
        {
            RigidbodyS bodyA = (RigidbodyS)a;
            RigidbodyS bodyB = b.isDynamic ? (RigidbodyS)b : null;

            // Iteratively solve the contact
            f32 accumulatedImpulse = this.cachedNormalImpulse;

            for (int iter = 0; iter < maxIterations; iter++)
            {

                f32 oldImpulse = accumulatedImpulse;

                // Use local offsets directly since rotation hasn't been applied yet
                Vector3S ra = this.localOffsetA; // Local offset is directly used as ra
                Vector3S rb = this.localOffsetB; // Local offset is directly used as rb
                // Calculate relative velocity at contact point
                Vector3S relativeVelocityAtContact = CalculateRelativeVelocityAtContact(bodyA, bodyB, ra, rb);

                // Calculate velocity along normal
                f32 velocityAlongNormal = Vector3S.Dot(relativeVelocityAtContact, this.normal);
                if (velocityAlongNormal > f32.zero) break; // Moving away, no impulse required

                // Calculate effective mass
                f32 effectiveMass = CalculateEffectiveMass(bodyA, bodyB, ra, rb, this.normal);

                // Compute the current contact separation for a sub-step
                Vector3S worldPointA = a.transform.position + ra;
                Vector3S worldPointB = b.transform.position + rb;
                f32 separation = Vector3S.Dot(worldPointB - worldPointA, normal) + this.penetrationDepth;

                // Baumgarte stabilization factor
                f32 baumFactor = BaumgarteFactor * separation / deltaTime;

                // Calculate impulse
                f32 impulse = -(f32.one + this.restitution) * velocityAlongNormal / effectiveMass + baumFactor;
                accumulatedImpulse = MathS.Max(f32.zero, oldImpulse + impulse); // Project impulse to be non-negative
                f32 appliedImpulse = accumulatedImpulse - oldImpulse;

                // Apply impulse
                ApplyImpulse(bodyA, bodyB, ra, rb, this.normal, appliedImpulse);

                // Check for convergence
                if (MathS.Abs(appliedImpulse) < Tolerance) break;
            }

            // Update cached normal impulse
            this.cachedNormalImpulse = accumulatedImpulse;

            // Handle friction after resolving normal impulses
            ResolveFriction(bodyA, bodyB);

            // Correct positions to reduce penetration
            CorrectPositions(bodyA, bodyB);
        }

        private static Vector3S CalculateRelativeVelocityAtContact(RigidbodyS bodyA, RigidbodyS bodyB, Vector3S ra, Vector3S rb)
        {
            Vector3S relativeVelocity = bodyA.velocity + Vector3S.Cross(bodyA.angularVelocity, ra);
            if (bodyB != null) relativeVelocity -= bodyB.velocity + Vector3S.Cross(bodyB.angularVelocity, rb);
            return relativeVelocity;
        }

        private static f32 CalculateEffectiveMass(RigidbodyS bodyA, RigidbodyS bodyB, Vector3S ra, Vector3S rb, Vector3S normal)
        {
            f32 invMassA = bodyA.inverseMass;
            f32 invMassB = bodyB != null ? bodyB.inverseMass : f32.zero;

            f32 effectiveMass = invMassA + Vector3S.Dot(Vector3S.Cross(bodyA.tensor.inertiaWorld * Vector3S.Cross(ra, normal), ra), normal);
            if (bodyB != null) effectiveMass += invMassB + Vector3S.Dot(Vector3S.Cross(bodyB.tensor.inertiaWorld * Vector3S.Cross(rb, normal), rb), normal);

            return effectiveMass;
        }

        private static void ApplyImpulse(RigidbodyS bodyA, RigidbodyS bodyB, Vector3S ra, Vector3S rb, Vector3S normal, f32 impulse)
        {
            Vector3S normalImpulse = normal * impulse;

            // Apply linear impulse
            bodyA.velocity += normalImpulse * bodyA.inverseMass;
            if (bodyB != null) bodyB.velocity -= normalImpulse * bodyB.inverseMass;

            // Apply angular impulse
            bodyA.angularVelocity += bodyA.tensor.inertiaWorld * Vector3S.Cross(ra, normalImpulse);
            if (bodyB != null) bodyB.angularVelocity -= bodyB.tensor.inertiaWorld * Vector3S.Cross(rb, normalImpulse);
        }

        private void ResolveFriction(RigidbodyS bodyA, RigidbodyS bodyB)
        {
            // Use local offsets directly since rotation hasn't been applied yet
            Vector3S ra = this.localOffsetA;
            Vector3S rb = this.localOffsetB;

            // Calculate the relative velocity at the contact point
            Vector3S relativeVelocityAtContact = CalculateRelativeVelocityAtContact(bodyA, bodyB, ra, rb);

            // Calculate the velocity along the normal
            Vector3S normalVelocity = this.normal * Vector3S.Dot(relativeVelocityAtContact, this.normal);

            // Calculate the tangential velocity (relative velocity minus the normal component)
            Vector3S tangentialVelocity = relativeVelocityAtContact - normalVelocity;

            // Early exit if tangential velocity is near zero
            if (tangentialVelocity.Magnitude() < Tolerance)
                return;

            // Normalize the tangential velocity to get the friction direction (tangent)
            Vector3S tangent = tangentialVelocity.Normalize();

            // Calculate the friction denominator (based on the tangent direction, not the normal)
            f32 frictionDenominator = CalculateFrictionDenominator(bodyA, bodyB, ra, rb, tangent);

            // Calculate the friction impulse scalar
            f32 frictionImpulseScalar = Vector3S.Dot(tangentialVelocity, tangent) / frictionDenominator;
            frictionImpulseScalar = -frictionImpulseScalar;

            // Calculate the friction impulse vector
            Vector3S frictionImpulse = tangent * frictionImpulseScalar;

            // Limit the friction impulse based on Coulomb's law (maximum friction = normal impulse * friction coefficient)
            f32 maxFrictionImpulse = this.cachedNormalImpulse * this.friction;
            if (frictionImpulse.Magnitude() > maxFrictionImpulse)
            {
                frictionImpulse = frictionImpulse.Normalize() * maxFrictionImpulse;
            }

            // Apply the linear friction impulse
            bodyA.velocity += frictionImpulse * bodyA.inverseMass;
            if (bodyB != null) bodyB.velocity -= frictionImpulse * bodyB.inverseMass;

            // Apply the angular friction impulse
            bodyA.angularVelocity += bodyA.tensor.inertiaWorld * Vector3S.Cross(ra, frictionImpulse);
            if (bodyB != null) bodyB.angularVelocity -= bodyB.tensor.inertiaWorld * Vector3S.Cross(rb, frictionImpulse);
        }
        private static f32 CalculateFrictionDenominator(RigidbodyS bodyA, RigidbodyS bodyB, Vector3S ra, Vector3S rb, Vector3S tangent)
        {
            f32 invMassA = bodyA.inverseMass;
            f32 invMassB = bodyB != null ? bodyB.inverseMass : f32.zero;

            f32 frictionDenominator = invMassA + Vector3S.Dot(Vector3S.Cross(bodyA.tensor.inertiaWorld * Vector3S.Cross(ra, tangent), ra), tangent);
            if (bodyB != null)
            {
                frictionDenominator += invMassB + Vector3S.Dot(Vector3S.Cross(bodyB.tensor.inertiaWorld * Vector3S.Cross(rb, tangent), rb), tangent);
            }

            return frictionDenominator;
        }

        private void CorrectPositions(RigidbodyS bodyA, RigidbodyS bodyB)
        {
            // Calculate relative positions using local points
            Vector3S ra = this.localOffsetA;
            Vector3S rb = this.localOffsetB;

            // Compute the current contact separation for position correction
            Vector3S worldPointA = a.transform.position + ra;
            Vector3S worldPointB = b.transform.position + rb;
            f32 separation = Vector3S.Dot(worldPointB - worldPointA, normal) + this.penetrationDepth;

            // Apply position correction to reduce interpenetration
            Vector3S correctionVector = this.normal * separation * PositionCorrectionFactor;

            bodyA.transform.position += correctionVector * bodyA.inverseMass;
            if (bodyB != null) bodyB.transform.position -= correctionVector * bodyB.inverseMass;

        }
    }
}
