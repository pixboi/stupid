using System;
using stupid.Maths;

namespace stupid.Colliders
{
    public readonly struct ContactVectorS
    {
        public readonly Vector3S point, normal;
        public readonly f32 penetrationDepth;

        public ContactVectorS(Vector3S point, Vector3S normal, f32 penetrationDepth)
        {
            this.point = point;
            this.normal = normal;
            this.penetrationDepth = penetrationDepth;
        }
    }

    public struct ContactS
    {
        public Collidable a, b;
        public RigidbodyS AB, BB;
        public ContactVectorS contact;
        public Vector3S ra, rb;
        public f32 effectiveMass, friction, restitution;

        // Cached impulses for warm starting
        public f32 accumulatedImpulse, accumulatedFriction;

        //Correction that are actuacted later, Only first contact should apply position correction?
        public Vector3S aVelocity, bVelocity, aAngular, bAngular;


        public ContactS(Collidable a, Collidable b, ContactVectorS contact)
        {
            this.a = a;
            this.b = b;
            this.AB = (RigidbodyS)a;
            this.BB = b.isDynamic ? (RigidbodyS)b : null;

            this.contact = contact;
            this.friction = (a.material.staticFriction + b.material.staticFriction) * f32.half;
            this.restitution = (a.material.restitution + b.material.restitution) * f32.half;

            // Initialize cached impulses to zero
            this.accumulatedImpulse = f32.zero;
            this.accumulatedFriction = f32.zero;

            this.ra = Vector3S.zero;
            this.rb = Vector3S.zero;
            this.effectiveMass = f32.zero;

            this.aVelocity = Vector3S.zero;
            this.bVelocity = Vector3S.zero;
            this.aAngular = Vector3S.zero;
            this.bAngular = Vector3S.zero;
        }

        public f32 CalculateEffectiveMass()
        {
            f32 invMassA = AB.inverseMass;
            f32 invMassB = BB != null ? BB.inverseMass : f32.zero;

            // Linear effective mass
            f32 effectiveMass = invMassA + invMassB;

            // Angular effective mass for body A
            Vector3S raCrossNormal = Vector3S.Cross(ra, this.contact.normal);
            f32 angularMassA = Vector3S.Dot(raCrossNormal, AB.tensor.inertiaWorld * raCrossNormal);
            effectiveMass += angularMassA;

            // Angular effective mass for body B
            if (BB != null)
            {
                Vector3S rbCrossNormal = Vector3S.Cross(rb, this.contact.normal);
                f32 angularMassB = Vector3S.Dot(rbCrossNormal, BB.tensor.inertiaWorld * rbCrossNormal);
                effectiveMass += angularMassB;
            }

            // Invert to get effective mass
            return effectiveMass > f32.zero ? f32.one / effectiveMass : f32.zero;
        }

        // This is per one physics step, this data is reused between substeps
        public void PreStep()
        {
            this.ra = this.contact.point - a.transform.position; // Local offset from body A's position
            this.rb = this.contact.point - b.transform.position; // Local offset from body B's position
            this.effectiveMass = CalculateEffectiveMass();
        }

        public void ResolveContact(f32 deltaTime, in WorldSettings settings, bool bias = true)
        {
            // Calculate relative velocity at contact point
            Vector3S contactVelocity = CalculateContactVelocity();

            // Calculate velocity along normal
            f32 vn = Vector3S.Dot(contactVelocity, this.contact.normal);
            if (vn > f32.zero) return;

            // Compute the current contact separation for a sub-step
            Vector3S worldPointA = a.transform.position + this.ra;
            Vector3S worldPointB = b.transform.position + this.rb;
            f32 separation = Vector3S.Dot(worldPointB - worldPointA, this.contact.normal) + this.contact.penetrationDepth;
            separation = MathS.Max(separation - settings.DefaultContactOffset, f32.zero);

            f32 incrementalImpulse = -effectiveMass;

            if (bias)
            {
                f32 baum = -settings.Baumgartner * separation / deltaTime;
                incrementalImpulse *= (vn + baum);
            }
            else
            {
                incrementalImpulse *= vn;
            }

            f32 newAccumulatedImpulse = MathS.Max(f32.zero, accumulatedImpulse + incrementalImpulse);
            f32 appliedImpulse = newAccumulatedImpulse - accumulatedImpulse;
            accumulatedImpulse = newAccumulatedImpulse;

            Vector3S normalImpulse = this.contact.normal * appliedImpulse;
            AB.velocity += normalImpulse * AB.inverseMass;
            if (BB != null) BB.velocity -= normalImpulse * BB.inverseMass;

            AB.angularVelocity += AB.tensor.inertiaWorld * Vector3S.Cross(ra, normalImpulse);
            if (BB != null) BB.angularVelocity -= BB.tensor.inertiaWorld * Vector3S.Cross(rb, normalImpulse);

            // Handle friction after resolving normal impulses
            //contactVelocity = CalculateContactVelocity();
            //vn = Vector3S.Dot(contactVelocity, this.contact.normal);
            ResolveFriction(contactVelocity, vn);

            // Apply position correction to reduce interpenetration
            if (bias)
            {
                Vector3S posCorrect = this.contact.normal * separation * settings.PositionCorrection;
                a.transform.position += posCorrect;
                if (BB != null) b.transform.position -= posCorrect;
            }
        }

        private Vector3S CalculateContactVelocity()
        {
            Vector3S relativeVelocity = AB.velocity + Vector3S.Cross(AB.angularVelocity, ra);
            if (BB != null) relativeVelocity -= BB.velocity + Vector3S.Cross(BB.angularVelocity, rb);
            return relativeVelocity;
        }

        private void ResolveFriction(Vector3S contactVelocity, f32 vn)
        {
            // Calculate the velocity along the normal
            Vector3S normalVelocity = this.contact.normal * vn;

            // Calculate the tangential velocity (relative velocity minus the normal component)
            Vector3S tangentialVelocity = contactVelocity - normalVelocity;

            // Check if tangential velocity is significant to avoid unnecessary calculations
            if (tangentialVelocity.Magnitude() <= f32.epsilon) return;

            // Normalize the tangential velocity to get the friction direction (tangent)
            Vector3S tangent = tangentialVelocity.Normalize();

            // Calculate the friction denominator
            f32 invMassA = AB.inverseMass;
            f32 invMassB = BB != null ? BB.inverseMass : f32.zero;
            f32 frictionDenominator = invMassA + Vector3S.Dot(Vector3S.Cross(AB.tensor.inertiaWorld * Vector3S.Cross(ra, tangent), ra), tangent);
            if (BB != null)
            {
                frictionDenominator += invMassB + Vector3S.Dot(Vector3S.Cross(BB.tensor.inertiaWorld * Vector3S.Cross(rb, tangent), rb), tangent);
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

            AB.velocity += frictionImpulse * AB.inverseMass;
            if (BB != null) BB.velocity -= frictionImpulse * BB.inverseMass;

            AB.angularVelocity += AB.tensor.inertiaWorld * Vector3S.Cross(ra, frictionImpulse);
            if (BB != null) BB.angularVelocity -= BB.tensor.inertiaWorld * Vector3S.Cross(rb, frictionImpulse);
        }

        public void Actuate()
        {
            AB.velocity += aVelocity;
            AB.angularVelocity += aAngular;

            if (BB != null)
            {
                BB.velocity += bVelocity;
                BB.angularVelocity += bAngular;
            }

            aVelocity = Vector3S.zero;
            aAngular = Vector3S.zero;
            bVelocity = Vector3S.zero;
            bAngular = Vector3S.zero;
        }
    }
}
