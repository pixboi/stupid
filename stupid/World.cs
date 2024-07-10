
using System.Collections.Generic;
using stupid.Colliders;
using stupid.Maths;
using stupid.Collections;
using System;

namespace stupid
{
    public class World
    {
        public WorldSettings Settings { get; private set; }
        public SortAndSweepBroadphase Broadphase { get; set; }
        public List<Collidable> Collidables { get; private set; }
        public uint SimulationFrame { get; private set; }
        public DumbGrid<int> DumbGrid { get; private set; }

        private int counter;
        public readonly Vector3S[] positionBuffer, velocityBuffer, angularBuffer;

        public World(WorldSettings worldSettings, int startSize = 1000)
        {
            this.Settings = worldSettings;

            Collidables = new List<Collidable>(startSize);
            DumbGrid = new DumbGrid<int>(32, 32, 32, (f32)4);
            Broadphase = new SortAndSweepBroadphase(startSize);

            positionBuffer = new Vector3S[startSize * 2];
            velocityBuffer = new Vector3S[startSize * 2];
            angularBuffer = new Vector3S[startSize * 2];

            counter = 0;
            SimulationFrame = 0;
        }


        public Collidable AddCollidable(IShape collider, bool isDynamic = false, Vector3S position = default, QuaternionS rotation = default, Vector3S localScale = default)
        {
            var c = new Collidable(counter++, collider, isDynamic, new TransformS(position, rotation, localScale));
            Collidables.Add(c);
            return c;

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

            //Integrate
            foreach (var c in Collidables)
            {
                if (c is RigidbodyS rb) rb.Integrate(deltaTime, Settings);
            }

            //Recalc things
            foreach (var c in Collidables)
            {
                c.CalculateBounds();

                if (c is RigidbodyS rb)
                {
                    rb.tensor.CalculateInverseInertiaTensor(rb.transform.rotation);
                }
            }

            var pairs = Broadphase.ComputePairs(Collidables);
            NarrowPhase(pairs);

            ApplyBuffers();

            var worldCollision = new InsideAABBColliderS(Settings.WorldBounds.min, Settings.WorldBounds.max);
            foreach (var c in Collidables)
            {
                if (c is RigidbodyS rb)
                {
                    if (worldCollision.Intersects(c, out var contact))
                    {
                        OnContact?.Invoke(new ContactManifoldS { a = null, b = null, contact = contact });
                        ResolveCollisionWithStatic(rb, contact);
                    }
                }
            }

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

                //DYN DYN
                if (a is RigidbodyS ab && b is RigidbodyS bb)
                {
                    if (a.collider.Intersects(b, out var contact))
                    {
                        OnContact?.Invoke(new ContactManifoldS { a = a, b = b, contact = contact });
                        ResolveCollision(ab, bb, contact);
                    }
                }
            }
        }

        public void ApplyBuffers()
        {
            foreach (var c in Collidables)
            {
                c.transform.position += positionBuffer[c.index];
                positionBuffer[c.index] = Vector3S.zero;

                if (c is RigidbodyS rb)
                {
                    rb.velocity += velocityBuffer[c.index];
                    rb.angularVelocity += angularBuffer[c.index];
                    velocityBuffer[c.index] = Vector3S.zero;
                    angularBuffer[c.index] = Vector3S.zero;
                }
            }
        }

        f32 POSITION_PERCENT = (f32)0.8;
        private void ResolveCollisionWithStatic(RigidbodyS body, ContactS contact)
        {
            // Calculate relative position from the center of mass to the contact point
            Vector3S rb = contact.point - body.transform.position;

            // Calculate the relative velocity at the contact point (only the dynamic body's velocity matters)
            Vector3S relativeVelocityAtContact = body.velocity + Vector3S.Cross(body.angularVelocity, rb);

            // Calculate the velocity along the normal
            f32 velocityAlongNormal = Vector3S.Dot(relativeVelocityAtContact, contact.normal);

            // Do not resolve if velocities are separating
            if (velocityAlongNormal > f32.zero) return;

            // Restitution (coefficient of restitution)
            f32 bounce = relativeVelocityAtContact.Magnitude() >= Settings.BounceTreshold ? body.material.bounciness : f32.zero;

            // Calculate inverse mass
            f32 invMassB = body.mass > f32.zero ? f32.one / body.mass : f32.zero;

            // Compute the effective mass along the normal direction
            f32 invEffectiveMassB = invMassB + Vector3S.Dot(Vector3S.Cross(body.tensor.inertiaWorld * Vector3S.Cross(rb, contact.normal), rb), contact.normal);

            // Calculate the normal impulse scalar
            f32 j = -(f32.one + bounce) * velocityAlongNormal / invEffectiveMassB;

            // Apply linear impulse
            Vector3S impulse = j * contact.normal;
            velocityBuffer[body.index] += invMassB * impulse;
            angularBuffer[body.index] += body.tensor.inertiaWorld * Vector3S.Cross(rb, impulse);

            // Positional correction to prevent sinking
            f32 slop = Settings.DefaultContactOffset;
            f32 penetrationDepth = MathS.Max(contact.penetrationDepth - slop, f32.zero);
            Vector3S correction = (penetrationDepth / invEffectiveMassB) * POSITION_PERCENT * contact.normal;
            positionBuffer[body.index] += invMassB * correction;

            // Calculate relative tangential velocity
            Vector3S relativeTangentialVelocity = relativeVelocityAtContact - (velocityAlongNormal * contact.normal);
            Vector3S tangent = relativeTangentialVelocity.Magnitude() > f32.zero ? relativeTangentialVelocity.Normalize() : Vector3S.zero;

            // Calculate the magnitude of the friction impulse
            f32 denominator = invMassB + Vector3S.Dot(Vector3S.Cross(body.tensor.inertiaWorld * Vector3S.Cross(rb, tangent), rb), tangent);
            f32 jt = -Vector3S.Dot(relativeTangentialVelocity, tangent) / denominator;

            // Use the maximum of the static and dynamic friction coefficients
            f32 effectiveFriction = MathS.Max(body.material.staticFriction, body.material.dynamicFriction);

            // Limit the friction impulse to prevent excessive angular velocities
            Vector3S frictionImpulse = MathS.Abs(jt) < j * effectiveFriction ? jt * tangent : -j * effectiveFriction * tangent;
            if (frictionImpulse.Magnitude() > j * effectiveFriction)
            {
                frictionImpulse = frictionImpulse.Normalize() * (j * effectiveFriction);
            }

            // Apply friction impulse
            velocityBuffer[body.index] += invMassB * frictionImpulse;
            angularBuffer[body.index] += body.tensor.inertiaWorld * Vector3S.Cross(rb, frictionImpulse);
        }


        private void ResolveCollision(RigidbodyS a, RigidbodyS b, ContactS contact)
        {
            // Calculate relative positions from the centers of mass to the contact point
            Vector3S ra = contact.point - a.transform.position;
            Vector3S rb = contact.point - b.transform.position;

            // Calculate relative velocity at the contact point
            Vector3S relativeVelocityAtContact = b.velocity - a.velocity
                                                 + Vector3S.Cross(a.angularVelocity, ra)
                                                 - Vector3S.Cross(b.angularVelocity, rb);

            // Calculate the velocity along the normal
            f32 velocityAlongNormal = Vector3S.Dot(relativeVelocityAtContact, contact.normal);

            // Do not resolve if velocities are separating
            if (velocityAlongNormal > f32.zero) return;

            // Restitution (coefficient of restitution)
            f32 bounce = relativeVelocityAtContact.Magnitude() >= Settings.BounceTreshold ? (a.material.bounciness + b.material.bounciness) * f32.half : f32.zero;

            // Calculate inverse masses
            f32 invMassA = a.mass > f32.zero ? f32.one / a.mass : f32.zero;
            f32 invMassB = b.mass > f32.zero ? f32.one / b.mass : f32.zero;

            // Compute the effective mass along the normal direction
            f32 invEffectiveMassA = invMassA + Vector3S.Dot(Vector3S.Cross(a.tensor.inertiaWorld * Vector3S.Cross(ra, contact.normal), ra), contact.normal);
            f32 invEffectiveMassB = invMassB + Vector3S.Dot(Vector3S.Cross(b.tensor.inertiaWorld * Vector3S.Cross(rb, contact.normal), rb), contact.normal);
            f32 invMassSum = invEffectiveMassA + invEffectiveMassB;

            // Calculate the normal impulse scalar
            f32 j = -(f32.one + bounce) * velocityAlongNormal / invMassSum;

            // Apply linear impulse
            Vector3S impulse = j * contact.normal;
            velocityBuffer[a.index] -= invMassA * impulse;
            velocityBuffer[b.index] += invMassB * impulse;
            angularBuffer[a.index] -= a.tensor.inertiaWorld * Vector3S.Cross(ra, impulse);
            angularBuffer[b.index] += b.tensor.inertiaWorld * Vector3S.Cross(rb, impulse);

            // Positional correction to prevent sinking
            f32 slop = Settings.DefaultContactOffset;
            f32 penetrationDepth = MathS.Max(contact.penetrationDepth - slop, f32.zero);
            Vector3S correction = (penetrationDepth / invMassSum) * POSITION_PERCENT * contact.normal;
            positionBuffer[a.index] -= invMassA * correction;
            positionBuffer[b.index] += invMassB * correction;

            // Calculate relative tangential velocity
            Vector3S relativeTangentialVelocity = relativeVelocityAtContact - (velocityAlongNormal * contact.normal);
            Vector3S tangent = relativeTangentialVelocity.Magnitude() > f32.zero ? relativeTangentialVelocity.Normalize() : Vector3S.zero;

            // Calculate the magnitude of the friction impulse
            f32 denominatorA = invMassA + Vector3S.Dot(Vector3S.Cross(a.tensor.inertiaWorld * Vector3S.Cross(ra, tangent), ra), tangent);
            f32 denominatorB = invMassB + Vector3S.Dot(Vector3S.Cross(b.tensor.inertiaWorld * Vector3S.Cross(rb, tangent), rb), tangent);
            f32 denominator = denominatorA + denominatorB;
            f32 jt = -Vector3S.Dot(relativeTangentialVelocity, tangent) / denominator;

            // Use the maximum of the static and dynamic friction coefficients
            f32 effectiveFriction = MathS.Max(MathS.Max(a.material.staticFriction, a.material.dynamicFriction), MathS.Max(b.material.staticFriction, b.material.dynamicFriction));

            // Limit the friction impulse to prevent excessive angular velocities
            Vector3S frictionImpulse = MathS.Abs(jt) < j * effectiveFriction ? jt * tangent : -j * effectiveFriction * tangent;
            if (frictionImpulse.Magnitude() > j * effectiveFriction)
            {
                frictionImpulse = frictionImpulse.Normalize() * (j * effectiveFriction);
            }

            // Apply friction impulse
            velocityBuffer[a.index] -= invMassA * frictionImpulse;
            velocityBuffer[b.index] += invMassB * frictionImpulse;
            angularBuffer[a.index] -= a.tensor.inertiaWorld * Vector3S.Cross(ra, frictionImpulse);
            angularBuffer[b.index] += b.tensor.inertiaWorld * Vector3S.Cross(rb, frictionImpulse);
        }

    }
}
