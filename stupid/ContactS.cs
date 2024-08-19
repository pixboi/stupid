﻿using System;
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
        public RigidbodyS bodyA, bodyB;
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
            this.bodyA = (RigidbodyS)a;
            this.bodyB = b.isDynamic ? (RigidbodyS)b : null;

            this.contact = contact;
            this.friction = (a.material.staticFriction + b.material.staticFriction) * f32.half;
            this.restitution = (a.material.restitution + b.material.restitution) * f32.half;

            // Initialize cached impulses to zero
            this.accumulatedImpulse = f32.zero;
            this.accumulatedFriction = f32.zero;

            // Compute initial local offsets, these are invalid if point is not set
            this.ra = Vector3S.zero; // Local offset from body A's position
            this.rb = Vector3S.zero; // Local offset from body B's position
            this.effectiveMass = f32.zero;

            this.aVelocity = Vector3S.zero;
            this.bVelocity = Vector3S.zero;
            this.aAngular = Vector3S.zero;
            this.bAngular = Vector3S.zero;
        }

        public f32 CalculateEffectiveMass()
        {
            f32 invMassA = bodyA.inverseMass;
            f32 invMassB = bodyB != null ? bodyB.inverseMass : f32.zero;

            // Linear effective mass
            f32 effectiveMass = invMassA + invMassB;

            // Angular effective mass for body A
            Vector3S raCrossNormal = Vector3S.Cross(ra, this.contact.normal);
            f32 angularMassA = Vector3S.Dot(raCrossNormal, bodyA.tensor.inertiaWorld * raCrossNormal);
            effectiveMass += angularMassA;

            // Angular effective mass for body B
            if (bodyB != null)
            {
                Vector3S rbCrossNormal = Vector3S.Cross(rb, this.contact.normal);
                f32 angularMassB = Vector3S.Dot(rbCrossNormal, bodyB.tensor.inertiaWorld * rbCrossNormal);
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
            bodyA.velocity += normalImpulse * bodyA.inverseMass;
            if (bodyB != null) bodyB.velocity -= normalImpulse * bodyB.inverseMass;

            bodyA.angularVelocity += bodyA.tensor.inertiaWorld * Vector3S.Cross(ra, normalImpulse);
            if (bodyB != null) bodyB.angularVelocity -= bodyB.tensor.inertiaWorld * Vector3S.Cross(rb, normalImpulse);

            // Handle friction after resolving normal impulses
            //contactVelocity = CalculateContactVelocity();
            //vn = Vector3S.Dot(contactVelocity, this.contact.normal);
            ResolveFriction(contactVelocity, vn);

            // Apply position correction to reduce interpenetration
            if (bias)
            {
                Vector3S posCorrect = this.contact.normal * separation * settings.PositionCorrection;
                a.transform.position += posCorrect;
                if (bodyB != null) b.transform.position -= posCorrect;
            }
        }

        private Vector3S CalculateContactVelocity()
        {
            Vector3S relativeVelocity = bodyA.velocity + Vector3S.Cross(bodyA.angularVelocity, ra);
            if (bodyB != null) relativeVelocity -= bodyB.velocity + Vector3S.Cross(bodyB.angularVelocity, rb);
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

            bodyA.velocity += frictionImpulse * bodyA.inverseMass;
            if (bodyB != null) bodyB.velocity -= frictionImpulse * bodyB.inverseMass;

            bodyA.angularVelocity += bodyA.tensor.inertiaWorld * Vector3S.Cross(ra, frictionImpulse);
            if (bodyB != null) bodyB.angularVelocity -= bodyB.tensor.inertiaWorld * Vector3S.Cross(rb, frictionImpulse);
        }

        public void Actuate()
        {
            bodyA.velocity += aVelocity;
            bodyA.angularVelocity += aAngular;

            if (bodyB != null)
            {
                bodyB.velocity += bVelocity;
                bodyB.angularVelocity += bAngular;
            }

            aVelocity = Vector3S.zero;
            aAngular = Vector3S.zero;
            bVelocity = Vector3S.zero;
            bAngular = Vector3S.zero;
        }
    }
}
