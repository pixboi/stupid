using System;
using stupid.Maths;

namespace stupid.Colliders
{
    public struct ContactS
    {
        public Collidable a, b;
        public Vector3S point, normal, ra, rb;
        public f32 penetrationDepth, effectiveMass, friction, restitution;

        // Cached impulses for warm starting
        public f32 accumulatedImpulse, accumulatedFriction;

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
            this.accumulatedFriction = f32.zero;

            // Compute initial local offsets, these are invalid if point is not set
            this.ra = this.point - a.transform.position; // Local offset from body A's position
            this.rb = this.point - b.transform.position; // Local offset from body B's position
            this.effectiveMass = f32.zero;
        }

        private static f32 CalculateEffectiveMass(RigidbodyS bodyA, RigidbodyS bodyB, Vector3S ra, Vector3S rb, Vector3S normal)
        {
            f32 invMassA = bodyA.inverseMass;
            f32 invMassB = bodyB != null ? bodyB.inverseMass : f32.zero;

            // Linear effective mass
            f32 effectiveMass = invMassA + invMassB;

            // Angular effective mass for body A
            Vector3S raCrossNormal = Vector3S.Cross(ra, normal);
            f32 angularMassA = Vector3S.Dot(raCrossNormal, bodyA.tensor.inertiaWorld * raCrossNormal);
            effectiveMass += angularMassA;

            // Angular effective mass for body B
            if (bodyB != null)
            {
                Vector3S rbCrossNormal = Vector3S.Cross(rb, normal);
                f32 angularMassB = Vector3S.Dot(rbCrossNormal, bodyB.tensor.inertiaWorld * rbCrossNormal);
                effectiveMass += angularMassB;
            }

            // Invert to get effective mass
            return effectiveMass > f32.zero ? f32.one / effectiveMass : f32.zero;
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

        public void ResolveContact(f32 deltaTime, in WorldSettings settings, bool bias = true)
        {
            RigidbodyS bodyA = (RigidbodyS)a;
            RigidbodyS bodyB = b.isDynamic ? (RigidbodyS)b : null;

            // Calculate relative velocity at contact point
            Vector3S contactVelocity = CalculateContactVelocity(bodyA, bodyB);

            // Calculate velocity along normal
            f32 vn = Vector3S.Dot(contactVelocity, this.normal);
            if (vn > f32.zero) return;

            // Compute the current contact separation for a sub-step
            Vector3S worldPointA = a.transform.position + this.ra;
            Vector3S worldPointB = b.transform.position + this.rb;
            f32 separation = Vector3S.Dot(worldPointB - worldPointA, normal) + this.penetrationDepth;
            separation = MathS.Max(separation - settings.DefaultContactOffset, f32.zero);

            f32 incrementalImpulse = -effectiveMass;
            if (bias)
            {
                f32 baum = -settings.Baumgartner * separation / deltaTime;
                incrementalImpulse *= (vn + baum);
            }
            else incrementalImpulse *= vn;

            f32 newAccumulatedImpulse = MathS.Max(f32.zero, accumulatedImpulse + incrementalImpulse);
            f32 appliedImpulse = newAccumulatedImpulse - accumulatedImpulse;
            accumulatedImpulse = newAccumulatedImpulse;

            Vector3S normalImpulse = normal * appliedImpulse;
            bodyA.velocity += normalImpulse * bodyA.inverseMass;
            if (bodyB != null) bodyB.velocity -= normalImpulse * bodyB.inverseMass;

            bodyA.angularVelocity += bodyA.tensor.inertiaWorld * Vector3S.Cross(ra, normalImpulse);
            if (bodyB != null) bodyB.angularVelocity -= bodyB.tensor.inertiaWorld * Vector3S.Cross(rb, normalImpulse);

            // Handle friction after resolving normal impulses
            ResolveFriction(bodyA, bodyB);

            // Apply position correction to reduce interpenetration
            if (bias)
            {
                Vector3S posCorrect = this.normal * separation * settings.PositionCorrection;
                bodyA.transform.position += posCorrect;
                if (bodyB != null) bodyB.transform.position -= posCorrect;
            }
        }

        private Vector3S CalculateContactVelocity(RigidbodyS a, RigidbodyS b)
        {
            Vector3S relativeVelocity = a.velocity + Vector3S.Cross(a.angularVelocity, ra);
            if (b != null)
            {
                relativeVelocity -= b.velocity + Vector3S.Cross(b.angularVelocity, rb);
            }
            return relativeVelocity;
        }

        private void ResolveFriction(RigidbodyS bodyA, RigidbodyS bodyB)
        {
            // Calculate the relative velocity at the contact point
            Vector3S relativeVelocityAtContact = CalculateContactVelocity(bodyA, bodyB);

            // Calculate the velocity along the normal
            Vector3S normalVelocity = this.normal * Vector3S.Dot(relativeVelocityAtContact, this.normal);

            // Calculate the tangential velocity (relative velocity minus the normal component)
            Vector3S tangentialVelocity = relativeVelocityAtContact - normalVelocity;

            // Check if tangential velocity is significant to avoid unnecessary calculations
            if (tangentialVelocity.Magnitude() <= f32.epsilon) return;

            // Normalize the tangential velocity to get the friction direction (tangent)
            Vector3S tangent = tangentialVelocity.Normalize();

            // Calculate the friction denominator
            f32 invMassA = bodyA.inverseMass;
            f32 invMassB = bodyB != null ? bodyB.inverseMass : f32.zero;
            f32 frictionDenominator = invMassA + Vector3S.Dot(Vector3S.Cross(bodyA.tensor.inertiaWorld * Vector3S.Cross(ra, tangent), ra), tangent);
            if (bodyB != null)
            {
                frictionDenominator += invMassB + Vector3S.Dot(Vector3S.Cross(bodyB.tensor.inertiaWorld * Vector3S.Cross(rb, tangent), rb), tangent);
            }

            // Calculate the friction impulse scalar
            f32 frictionImpulseScalar = Vector3S.Dot(tangentialVelocity, tangent) / frictionDenominator;
            frictionImpulseScalar = -frictionImpulseScalar;

            // Compute the maximum friction impulse (Coulomb's law)
            f32 maxFrictionImpulse = this.accumulatedImpulse * this.friction;

            // Calculate the new friction impulse with clamping
            f32 oldAccumulatedFriction = this.accumulatedFriction;
            this.accumulatedFriction = MathS.Clamp(this.accumulatedFriction + frictionImpulseScalar, -maxFrictionImpulse, maxFrictionImpulse);

            // Calculate the actual friction impulse to apply
            f32 appliedFrictionImpulse = this.accumulatedFriction - oldAccumulatedFriction;
            Vector3S frictionImpulse = tangent * appliedFrictionImpulse;

            // Apply the linear friction impulse to body A
            bodyA.velocity += frictionImpulse * bodyA.inverseMass;

            // Apply the linear friction impulse to body B if dynamic
            if (bodyB != null)
            {
                bodyB.velocity -= frictionImpulse * bodyB.inverseMass;
            }

            // Apply the angular friction impulse to body A
            bodyA.angularVelocity += bodyA.tensor.inertiaWorld * Vector3S.Cross(ra, frictionImpulse);

            // Apply the angular friction impulse to body B if dynamic
            if (bodyB != null)
            {
                bodyB.angularVelocity -= bodyB.tensor.inertiaWorld * Vector3S.Cross(rb, frictionImpulse);
            }
        }
    }
}
