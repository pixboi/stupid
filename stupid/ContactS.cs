using System;
using stupid.Maths;

namespace stupid.Colliders
{
    public struct ContactS
    {
        public Collidable a, b;
        public Vector3S point, normal, ra, rb;
        public f32 penetrationDepth, effectiveMass;
        public readonly f32 friction, restitution;

        // Cached impulses for warm starting
        public f32 accumulatedImpulse;

        private static readonly f32 BaumgarteFactor = f32.zero;
        private static readonly f32 PositionCorrectionFactor = (f32)0.5; // Position correction factor

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
            this.accumulatedImpulse = f32.zero;

            // Compute initial local offsets, these are invalid if point is not set
            this.ra = this.point - a.transform.position; // Local offset from body A's position
            this.rb = this.point - b.transform.position; // Local offset from body B's position
            this.effectiveMass = f32.zero;
        }

        private static f32 CalculateEffectiveMass(RigidbodyS bodyA, RigidbodyS bodyB, Vector3S ra, Vector3S rb, Vector3S normal)
        {
            f32 invMassA = bodyA.inverseMass;
            f32 invMassB = bodyB != null ? bodyB.inverseMass : f32.zero;

            f32 effectiveMass = invMassA + Vector3S.Dot(Vector3S.Cross(bodyA.tensor.inertiaWorld * Vector3S.Cross(ra, normal), ra), normal);
            if (bodyB != null) effectiveMass += invMassB + Vector3S.Dot(Vector3S.Cross(bodyB.tensor.inertiaWorld * Vector3S.Cross(rb, normal), rb), normal);

            return effectiveMass;
        }

        // This is per one physics step, this data is reused between substeps
        public void PreStep()
        {
            this.ra = this.point - a.transform.position; // Local offset from body A's position
            this.rb = this.point - b.transform.position; // Local offset from body B's position

            RigidbodyS bodyA = (RigidbodyS)a;
            RigidbodyS bodyB = b.isDynamic ? (RigidbodyS)b : null;

            this.effectiveMass = CalculateEffectiveMass(bodyA, bodyB, this.ra, this.rb, this.normal);
        }


        public void ResolveContact(f32 deltaTime, in WorldSettings settings)
        {
            RigidbodyS bodyA = (RigidbodyS)a;
            RigidbodyS bodyB = b.isDynamic ? (RigidbodyS)b : null;

            // Calculate relative velocity at contact point
            Vector3S contactVelocity = CalculateContactVelocity(bodyA, bodyB, ra, rb);

            // Calculate velocity along normal
            f32 vn = Vector3S.Dot(contactVelocity, this.normal);
            if (vn > f32.zero) return;

            // Compute the current contact separation for a sub-step
            Vector3S worldPointA = a.transform.position + this.ra;
            Vector3S worldPointB = b.transform.position + this.rb;
            f32 separation = Vector3S.Dot(worldPointB - worldPointA, normal) + this.penetrationDepth;
            separation = MathS.Max(separation - settings.DefaultContactOffset, f32.zero);

            // Baumgarte stabilization factor for position correction
            f32 baumFactor = BaumgarteFactor * separation / deltaTime;

            // Calculate impulse only affecting linear velocity
            f32 impulse = -(f32.one + this.restitution) * vn / effectiveMass + baumFactor;
            f32 appliedImpulse = MathS.Max(f32.zero, this.accumulatedImpulse + impulse) - this.accumulatedImpulse;

            // Update cached normal impulse
            this.accumulatedImpulse += appliedImpulse;

            Vector3S normalImpulse = normal * appliedImpulse;

            bodyA.velocity += normalImpulse * bodyA.inverseMass;
            if (bodyB != null) bodyB.velocity -= normalImpulse * bodyB.inverseMass;

            bodyA.angularVelocity += bodyA.tensor.inertiaWorld * Vector3S.Cross(ra, normalImpulse);
            if (bodyB != null) bodyB.angularVelocity -= bodyB.tensor.inertiaWorld * Vector3S.Cross(rb, normalImpulse);

            // Handle friction after resolving normal impulses
            ResolveFriction(bodyA, bodyB);

            // Apply position correction to reduce interpenetration
            Vector3S finalCorrectionVector = this.normal * separation * PositionCorrectionFactor;

            bodyA.transform.position += finalCorrectionVector;
            if (bodyB != null) bodyB.transform.position -= finalCorrectionVector;
        }


        private static Vector3S CalculateContactVelocity(RigidbodyS bodyA, RigidbodyS bodyB, Vector3S ra, Vector3S rb)
        {
            Vector3S relativeVelocity = bodyA.velocity + Vector3S.Cross(bodyA.angularVelocity, ra);
            if (bodyB != null) relativeVelocity -= bodyB.velocity + Vector3S.Cross(bodyB.angularVelocity, rb);
            return relativeVelocity;
        }

        private void ResolveFriction(RigidbodyS bodyA, RigidbodyS bodyB)
        {
            // Calculate the relative velocity at the contact point
            Vector3S relativeVelocityAtContact = CalculateContactVelocity(bodyA, bodyB, ra, rb);

            // Calculate the velocity along the normal
            Vector3S normalVelocity = this.normal * Vector3S.Dot(relativeVelocityAtContact, this.normal);

            // Calculate the tangential velocity (relative velocity minus the normal component)
            Vector3S tangentialVelocity = relativeVelocityAtContact - normalVelocity;

            // Normalize the tangential velocity to get the friction direction (tangent)
            Vector3S tangent = tangentialVelocity.Normalize();

            f32 invMassA = bodyA.inverseMass;
            f32 invMassB = bodyB != null ? bodyB.inverseMass : f32.zero;
            f32 frictionDenominator = invMassA + Vector3S.Dot(Vector3S.Cross(bodyA.tensor.inertiaWorld * Vector3S.Cross(ra, tangent), ra), tangent);
            if (bodyB != null) frictionDenominator += invMassB + Vector3S.Dot(Vector3S.Cross(bodyB.tensor.inertiaWorld * Vector3S.Cross(rb, tangent), rb), tangent);

            // Calculate the friction impulse scalar
            f32 frictionImpulseScalar = Vector3S.Dot(tangentialVelocity, tangent) / frictionDenominator;
            frictionImpulseScalar = -frictionImpulseScalar;

            // Calculate the friction impulse vector
            Vector3S frictionImpulse = tangent * frictionImpulseScalar;

            // Limit the friction impulse based on Coulomb's law (maximum friction = normal impulse * friction coefficient)
            f32 maxFrictionImpulse = this.accumulatedImpulse * this.friction;
            if (frictionImpulse.Magnitude() > maxFrictionImpulse)
            {
                frictionImpulse = frictionImpulse.Normalize() * maxFrictionImpulse;
            }

            // Apply the linear friction impulse to body A
            bodyA.velocity += frictionImpulse * bodyA.inverseMass;
            // Apply the linear friction impulse to body B if dynamic
            if (bodyB != null) bodyB.velocity -= frictionImpulse * bodyB.inverseMass;

            // Apply the angular friction impulse to body A
            bodyA.angularVelocity += bodyA.tensor.inertiaWorld * Vector3S.Cross(ra, frictionImpulse);
            // Apply the angular friction impulse to body B if dynamic
            if (bodyB != null) bodyB.angularVelocity -= bodyB.tensor.inertiaWorld * Vector3S.Cross(rb, frictionImpulse);
        }

    }
}
