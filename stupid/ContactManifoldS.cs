using System;
using System.Diagnostics.Contracts;
using System.Runtime;
using stupid.Maths;

namespace stupid.Colliders
{

    public struct ContactManifoldS
    {
        public Collidable a, b;
        public RigidbodyS AB, BB;
        public ContactS[] contacts;
        public f32 friction, restitution;

        public ContactManifoldS(Collidable a, Collidable b, ContactS[] contacts)
        {
            this.a = a;
            this.b = b;
            this.AB = (RigidbodyS)a;
            this.BB = b.isDynamic ? (RigidbodyS)b : null;

            this.contacts = contacts;
            this.friction = (a.material.staticFriction + b.material.staticFriction) * f32.half;
            this.restitution = (a.material.restitution + b.material.restitution) * f32.half;
        }

        private Vector3S CalculateContactVelocity(ContactS contact)
        {
            Vector3S relativeVelocity = AB.velocity + Vector3S.Cross(AB.angularVelocity, contact.ra);
            if (BB != null) relativeVelocity -= BB.velocity + Vector3S.Cross(BB.angularVelocity, contact.rb);
            return relativeVelocity;
        }

        public void Resolve(f32 deltaTime, in WorldSettings settings, bool bias = true)
        {
            for (int i = 0; i < contacts.Length; i++)
            {
                var c = contacts[i];


                Vector3S contactVelocity = CalculateContactVelocity(c);
                f32 vn = Vector3S.Dot(contactVelocity, c.normal);
                if (vn > f32.zero) continue;

                SolveImpulse(ref c, vn, deltaTime, settings, bias);

                // Calculate relative velocity at contact point
                contactVelocity = CalculateContactVelocity(c);

                // Calculate velocity along normal
                vn = Vector3S.Dot(contactVelocity, c.normal);
                SolveFriction(ref c, contactVelocity, vn);

                contacts[i] = c;
            }

            var first = contacts[0];
            SolvePosition(ref first, settings);
        }

        void SolveImpulse(ref ContactS contact, f32 vn, f32 deltaTime, in WorldSettings settings, bool bias = true)
        {
            // Compute the current contact separation for a sub-step
            Vector3S worldPointA = a.transform.position + contact.ra;
            Vector3S worldPointB = b.transform.position + contact.rb;
            f32 separation = Vector3S.Dot(worldPointB - worldPointA, contact.normal) + contact.penetrationDepth;
            separation = MathS.Max(separation - settings.DefaultContactOffset, f32.zero);

            f32 incrementalImpulse = -contact.effectiveMass;

            if (bias)
            {
                f32 baum = -settings.Baumgartner * separation / deltaTime;
                incrementalImpulse *= (vn + baum);
            }
            else
            {
                incrementalImpulse *= vn;
            }

            f32 newAccumulatedImpulse = MathS.Max(f32.zero, contact.accumulatedImpulse + incrementalImpulse);
            f32 appliedImpulse = newAccumulatedImpulse - contact.accumulatedImpulse;
            contact.accumulatedImpulse = newAccumulatedImpulse;

            Vector3S normalImpulse = contact.normal * appliedImpulse;
            AB.velocity += normalImpulse * AB.inverseMass;
            if (BB != null) BB.velocity -= normalImpulse * BB.inverseMass;

            AB.angularVelocity += AB.tensor.inertiaWorld * Vector3S.Cross(contact.ra, normalImpulse);
            if (BB != null) BB.angularVelocity -= BB.tensor.inertiaWorld * Vector3S.Cross(contact.rb, normalImpulse);
        }

        void SolveFriction(ref ContactS contact, Vector3S contactVelocity, f32 vn)
        {
            // Calculate the velocity along the normal
            Vector3S normalVelocity = contact.normal * vn;

            // Calculate the tangential velocity (relative velocity minus the normal component)
            Vector3S tangentialVelocity = contactVelocity - normalVelocity;

            // Check if tangential velocity is significant to avoid unnecessary calculations
            if (tangentialVelocity.Magnitude() <= f32.epsilon) return;

            // Normalize the tangential velocity to get the friction direction (tangent)
            Vector3S tangent = tangentialVelocity.Normalize();

            // Calculate the friction denominator
            f32 invMassA = AB.inverseMass;
            f32 invMassB = BB != null ? BB.inverseMass : f32.zero;
            f32 frictionDenominator = invMassA + Vector3S.Dot(Vector3S.Cross(AB.tensor.inertiaWorld * Vector3S.Cross(contact.ra, tangent), contact.ra), tangent);
            if (BB != null)
            {
                frictionDenominator += invMassB + Vector3S.Dot(Vector3S.Cross(BB.tensor.inertiaWorld * Vector3S.Cross(contact.rb, tangent), contact.rb), tangent);
            }

            // Calculate the friction impulse scalar
            f32 frictionImpulseScalar = Vector3S.Dot(tangentialVelocity, tangent) / frictionDenominator;
            frictionImpulseScalar = -frictionImpulseScalar;

            // Compute the maximum friction impulse (Coulomb's law)
            f32 maxFrictionImpulse = contact.accumulatedImpulse * this.friction;

            // Calculate the new friction impulse with clamping
            f32 oldAccumulatedFriction = contact.accumulatedFriction;
            contact.accumulatedFriction = MathS.Clamp(contact.accumulatedFriction + frictionImpulseScalar, -maxFrictionImpulse, maxFrictionImpulse);

            // Calculate the actual friction impulse to apply
            f32 appliedFrictionImpulse = contact.accumulatedFriction - oldAccumulatedFriction;
            Vector3S frictionImpulse = tangent * appliedFrictionImpulse;

            AB.velocity += frictionImpulse * AB.inverseMass;
            if (BB != null) BB.velocity -= frictionImpulse * BB.inverseMass;

            AB.angularVelocity += AB.tensor.inertiaWorld * Vector3S.Cross(contact.ra, frictionImpulse);
            if (BB != null) BB.angularVelocity -= BB.tensor.inertiaWorld * Vector3S.Cross(contact.rb, frictionImpulse);
        }

        public void SolvePosition(ref ContactS contact, WorldSettings settings)
        {
            // Compute the current contact separation for a sub-step
            Vector3S worldPointA = a.transform.position + contact.ra;
            Vector3S worldPointB = b.transform.position + contact.rb;
            f32 separation = Vector3S.Dot(worldPointB - worldPointA, contact.normal) + contact.penetrationDepth;
            separation = MathS.Max(separation - settings.DefaultContactOffset, f32.zero);

            Vector3S posCorrect = contact.normal * separation * settings.PositionCorrection;
            a.transform.position += posCorrect;
            if (BB != null) b.transform.position -= posCorrect;
        }


    }
}
