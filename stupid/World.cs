﻿
using System.Collections.Generic;
using stupid.Colliders;
using stupid.Maths;
using System;

namespace stupid
{
    public class World
    {
        public WorldSettings Settings { get; private set; }
        public SortAndSweepBroadphase Broadphase { get; set; }
        public DumbList<Collidable> Collidables { get; private set; }
        public uint SimulationFrame { get; private set; }

        private int counter;
        public readonly Vector3S[] positionBuffer, velocityBuffer, angularBuffer;

        public World(WorldSettings worldSettings, int startSize = 1000)
        {
            this.Settings = worldSettings;

            Collidables = new DumbList<Collidable>(startSize);
            Broadphase = new SortAndSweepBroadphase(startSize);

            positionBuffer = new Vector3S[startSize * 2];
            velocityBuffer = new Vector3S[startSize * 2];
            angularBuffer = new Vector3S[startSize * 2];

            counter = 0;
            SimulationFrame = 0;
        }

        public Collidable AddCollidable(Collidable c)
        {
            c.Register(counter++);
            Collidables.Add(c);
            return c;
        }

        public Collidable AddCollidable(IShape collider, bool isDynamic = false, Vector3S position = default, QuaternionS rotation = default, Vector3S localScale = default)
        {
            var c = new Collidable(counter++, collider, isDynamic, new TransformS(position, rotation, localScale));
            Collidables.Add(c);
            return c;

        }

        public RigidbodyS AddRigidbody(RigidbodyS rb)
        {
            rb.Register(counter++);
            Collidables.Add(rb);
            return rb;
        }

        public RigidbodyS AddRigidbody(IShape collider, Vector3S position = default, Vector3S velocity = default, Vector3S angularVelocity = default, f32 mass = default)
        {
            var rb = new RigidbodyS(counter++, collider, true, new TransformS(position, QuaternionS.identity, Vector3S.one),
                velocity,
                angularVelocity,
                mass,
                true,
                false
                );
            Collidables.Add(rb);
            return rb;
        }


        public static f32 DeltaTime;

        public void Simulate(f32 deltaTime)
        {
            DeltaTime = deltaTime;

            foreach (var c in Collidables)
            {
                if (c is RigidbodyS rb) rb.Integrate(deltaTime, Settings);
            }

            //Recalc things
            foreach (var c in Collidables)
            {
                if (c.isDynamic)
                {
                    if (c.collider.NeedsRotationUpdate)
                    {
                        c.transform.UpdateRotationMatrix();
                        c.collider.OnRotationUpdate();
                    }

                    c.CalculateBounds();
                }

                if (c is RigidbodyS rb)
                {
                    if (rb.angularVelocity.SqrMagnitude > f32.zero)
                        rb.tensor.CalculateInverseInertiaTensor(rb.transform.rotation);
                }
            }

            var pairs = Broadphase.ComputePairs(Collidables);
            NarrowPhase(pairs);

            ApplyBuffers();

            SimulationFrame++;
        }

        public event Action<ContactManifoldS> OnContact;
        private void NarrowPhase(HashSet<BodyPair> pairs)
        {
            foreach (var pair in pairs)
            {
                var a = Collidables[pair.aIndex];
                var b = Collidables[pair.bIndex];

                // Dynamic vs Dynamic
                if (a is RigidbodyS ab && b is RigidbodyS bb)
                {
                    if (a.collider.Intersects(b, out var contact))
                    {
                        OnContact?.Invoke(new ContactManifoldS { a = a, b = b, contact = contact });
                        ResolveCollision(ab, bb, contact);
                    }
                    continue;
                }

                // Dynamic vs Static
                if (a.isDynamic && !b.isDynamic)
                {
                    if (a.collider.Intersects(b, out var contact))
                    {
                        OnContact?.Invoke(new ContactManifoldS { a = a, b = b, contact = contact });
                        ResolveCollisionWithStatic((RigidbodyS)a, contact);
                    }
                    continue;
                }

                // Static vs Dynamic
                if (!a.isDynamic && b.isDynamic)
                {
                    if (b.collider.Intersects(a, out var contact))
                    {
                        OnContact?.Invoke(new ContactManifoldS { a = a, b = b, contact = contact });
                        ResolveCollisionWithStatic((RigidbodyS)b, contact);
                    }
                    continue;
                }
            }
        }

        public void ApplyBuffers()
        {
            foreach (var c in Collidables)
            {
                if (c.isDynamic)
                {
                    c.transform.position += positionBuffer[c.index];
                    positionBuffer[c.index] = Vector3S.zero;
                }

                if (c is RigidbodyS rb)
                {
                    rb.velocity += velocityBuffer[c.index];
                    rb.angularVelocity += angularBuffer[c.index];
                    velocityBuffer[c.index] = Vector3S.zero;
                    angularBuffer[c.index] = Vector3S.zero;
                }
            }
        }

        private static readonly f32 POSITION_PERCENT = (f32)0.8;
        private void ResolveCollisionWithStatic(RigidbodyS body, ContactS contact)
        {
            // Ensure the normal always points from body to the contact point
            Vector3S normal = contact.normal;
            if (Vector3S.Dot(contact.normal, body.transform.position - contact.point) < f32.zero)
            {
                normal = -contact.normal;
            }

            // Calculate relative position from the center of mass to the contact point
            Vector3S rb = contact.point - body.transform.position;

            // Calculate the relative velocity at the contact point
            Vector3S relativeVelocityAtContact = body.velocity + Vector3S.Cross(body.angularVelocity, rb);

            // Calculate inverse mass
            f32 invMass = body.mass > f32.zero ? f32.one / body.mass : f32.zero;

            // Compute the effective mass along the normal direction
            f32 effectiveMass = invMass + Vector3S.Dot(Vector3S.Cross(body.tensor.inertiaWorld * Vector3S.Cross(rb, normal), rb), normal);

            // Positional correction to prevent sinking
            f32 slop = Settings.DefaultContactOffset;
            f32 penetrationDepth = MathS.Max(contact.penetrationDepth - slop, f32.zero);
            Vector3S correction = (penetrationDepth / effectiveMass) * POSITION_PERCENT * normal;
            positionBuffer[body.index] += invMass * correction;

            // Calculate the velocity along the normal
            f32 velocityAlongNormal = Vector3S.Dot(relativeVelocityAtContact, normal);

            // Do not resolve if velocities are separating
            if (velocityAlongNormal > f32.zero) return;

            // Restitution (coefficient of restitution)
            f32 restitution = relativeVelocityAtContact.Magnitude() >= Settings.BounceThreshold ? body.material.bounciness : f32.zero;

            // Calculate the normal impulse scalar
            f32 impulseScalar = -(f32.one + restitution) * velocityAlongNormal / effectiveMass;

            // Apply linear impulse
            Vector3S impulse = impulseScalar * normal;
            velocityBuffer[body.index] += invMass * impulse;
            angularBuffer[body.index] += body.tensor.inertiaWorld * Vector3S.Cross(rb, impulse);

            // Calculate relative tangential velocity
            Vector3S relativeTangentialVelocity = relativeVelocityAtContact - (velocityAlongNormal * normal);
            Vector3S tangent = relativeTangentialVelocity.Magnitude() > f32.zero ? relativeTangentialVelocity.Normalize() : Vector3S.zero;

            // Calculate the magnitude of the friction impulse
            f32 frictionDenominator = invMass + Vector3S.Dot(Vector3S.Cross(body.tensor.inertiaWorld * Vector3S.Cross(rb, tangent), rb), tangent);
            f32 frictionImpulseScalar = -Vector3S.Dot(relativeTangentialVelocity, tangent) / frictionDenominator;

            // Use the maximum of the static and dynamic friction coefficients
            f32 effectiveFriction = MathS.Max(body.material.staticFriction, body.material.dynamicFriction);

            // Limit the friction impulse to prevent excessive angular velocities
            Vector3S frictionImpulse = frictionImpulseScalar * tangent;
            if (frictionImpulse.Magnitude() > impulseScalar * effectiveFriction)
            {
                frictionImpulse = frictionImpulse.Normalize() * (impulseScalar * effectiveFriction);
            }

            // Apply friction impulse
            velocityBuffer[body.index] += invMass * frictionImpulse;
            angularBuffer[body.index] += body.tensor.inertiaWorld * Vector3S.Cross(rb, frictionImpulse);
        }

        private void ResolveCollision(RigidbodyS a, RigidbodyS b, ContactS contact)
        {
            // Ensure the normal always points from b to a
            Vector3S normal = contact.normal;

            // Calculate relative positions from the centers of mass to the contact point
            Vector3S ra = contact.point - a.transform.position;
            Vector3S rb = contact.point - b.transform.position;

            Vector3S va = a.velocity + Vector3S.Cross(a.angularVelocity, ra);
            Vector3S vb = b.velocity + Vector3S.Cross(b.angularVelocity, rb);

            // Calculate relative velocity at the contact point
            Vector3S relativeVelocityAtContact = va - vb;

            // Calculate the velocity along the normal
            f32 velocityAlongNormal = Vector3S.Dot(relativeVelocityAtContact, normal);

            // Do not resolve if velocities are separating
            if (velocityAlongNormal > f32.zero) return;

            // Calculate inverse masses
            f32 invMassA = a.mass > f32.zero ? f32.one / a.mass : f32.zero;
            f32 invMassB = b.mass > f32.zero ? f32.one / b.mass : f32.zero;

            // Compute the effective mass along the normal direction
            f32 effectiveMassA = invMassA + Vector3S.Dot(Vector3S.Cross(a.tensor.inertiaWorld * Vector3S.Cross(ra, normal), ra), normal);
            f32 effectiveMassB = invMassB + Vector3S.Dot(Vector3S.Cross(b.tensor.inertiaWorld * Vector3S.Cross(rb, normal), rb), normal);
            f32 effectiveMassSum = effectiveMassA + effectiveMassB;

            // Positional correction to prevent sinking
            f32 slop = Settings.DefaultContactOffset;
            f32 penetrationDepth = MathS.Max(contact.penetrationDepth - slop, f32.zero);
            Vector3S correction = (penetrationDepth / effectiveMassSum) * POSITION_PERCENT * normal;
            positionBuffer[a.index] += invMassA * correction;
            positionBuffer[b.index] -= invMassB * correction;

            // Restitution (coefficient of restitution)
            f32 bounce = relativeVelocityAtContact.Magnitude() >= Settings.BounceThreshold
                ? (a.material.bounciness + b.material.bounciness) * f32.half
                : f32.zero;

            // Calculate the normal impulse scalar
            f32 j = -(f32.one + bounce) * velocityAlongNormal / effectiveMassSum;

            // Apply linear impulse
            Vector3S impulse = j * normal;
            velocityBuffer[a.index] += invMassA * impulse;
            velocityBuffer[b.index] -= invMassB * impulse;
            angularBuffer[a.index] += a.tensor.inertiaWorld * Vector3S.Cross(ra, impulse);
            angularBuffer[b.index] -= b.tensor.inertiaWorld * Vector3S.Cross(rb, impulse);

            // Calculate relative tangential velocity
            Vector3S relativeTangentialVelocity = relativeVelocityAtContact - (velocityAlongNormal * normal);
            Vector3S tangent = relativeTangentialVelocity.Magnitude() > f32.zero ? relativeTangentialVelocity.Normalize() : Vector3S.zero;

            // Calculate the magnitude of the friction impulse
            f32 jt = -Vector3S.Dot(relativeTangentialVelocity, tangent) / effectiveMassSum;

            // Use the average of the static and dynamic friction coefficients
            f32 staticFriction = (a.material.staticFriction + b.material.staticFriction) * f32.half;
            f32 dynamicFriction = (a.material.dynamicFriction + b.material.dynamicFriction) * f32.half;

            // Calculate the friction impulse
            Vector3S frictionImpulse;
            if (MathS.Abs(jt) < j * staticFriction)
            {
                frictionImpulse = jt * tangent;
            }
            else
            {
                frictionImpulse = -j * dynamicFriction * tangent;
            }

            // Apply friction impulse
            velocityBuffer[a.index] += invMassA * frictionImpulse;
            velocityBuffer[b.index] -= invMassB * frictionImpulse;
            angularBuffer[a.index] += a.tensor.inertiaWorld * Vector3S.Cross(ra, frictionImpulse);
            angularBuffer[b.index] -= b.tensor.inertiaWorld * Vector3S.Cross(rb, frictionImpulse);
        }




    }
}
