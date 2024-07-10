
using System.Collections.Generic;
using stupid.Colliders;
using stupid.Maths;
using stupid.Collections;
using System;
using System.Diagnostics;

namespace stupid
{
    public class World
    {
        public SortAndSweepBroadphase Broadphase { get; set; }
        public BoundsS WorldBounds { get; private set; }
        public List<Collidable> Collidables { get; private set; }
        public Vector3S Gravity { get; private set; }
        public uint SimulationFrame { get; private set; }
        public DumbGrid<int> DumbGrid { get; private set; }

        private int counter;
        public readonly Vector3S[] positionBuffer, velocityBuffer, angularBuffer;

        public World(BoundsS worldBounds, SortAndSweepBroadphase broadphase, Vector3S gravity, int startSize = 1000)
        {
            counter = 0;
            SimulationFrame = 0;
            WorldBounds = worldBounds;
            Gravity = gravity;
            Collidables = new List<Collidable>(startSize);
            DumbGrid = new DumbGrid<int>(32, 32, 32, (f32)4);

            Broadphase = broadphase;

            positionBuffer = new Vector3S[startSize * 2];
            velocityBuffer = new Vector3S[startSize * 2];
            angularBuffer = new Vector3S[startSize * 2];
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

            Integrate(deltaTime);

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

            var worldCollision = new InsideAABBColliderS(WorldBounds.min, WorldBounds.max);
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

        public void ApplyBuffers()
        {
            foreach (var c in Collidables)
            {
                c.transform.position += positionBuffer[c.index];
                if (c is RigidbodyS rb)
                {
                    rb.velocity += velocityBuffer[c.index];
                    rb.angularVelocity += angularBuffer[c.index];
                }

                positionBuffer[c.index] = Vector3S.zero;
                velocityBuffer[c.index] = Vector3S.zero;
                angularBuffer[c.index] = Vector3S.zero;
            }

            //ClearBuffer();
        }

        public void ClearBuffer()
        {
            Array.Clear(angularBuffer, 0, angularBuffer.Length);
            Array.Clear(velocityBuffer, 0, velocityBuffer.Length);
            Array.Clear(positionBuffer, 0, velocityBuffer.Length);
        }

        f32 MAX_ANGULAR = (f32)7;
        private void Integrate(f32 deltaTime)
        {
            foreach (RigidbodyS rb in Collidables)
            {
                if (rb.isKinematic) continue;

                // Apply accumulated forces
                if (rb.useGravity)
                {
                    rb.AddForce(Gravity, ForceModeS.Acceleration); // Apply gravity as acceleration
                }

                // Update linear velocity with accumulated forces
                if (rb.forceBucket != Vector3S.zero)
                    rb.velocity += rb.forceBucket / rb.mass;

                // Apply linear drag
                if (rb.drag != f32.zero)
                    rb.velocity *= MathS.Max((f32)1.0f - rb.drag * deltaTime, (f32)0.0f);

                // Update position
                rb.transform.position += rb.velocity * deltaTime;

                // Update angular velocity with accumulated torques
                if (rb.torqueBucket != Vector3S.zero)
                    rb.angularVelocity += rb.tensor.inertiaWorld * rb.torqueBucket / rb.mass;

                // Apply angular drag
                rb.angularVelocity *= MathS.Max((f32)1.0f - rb.angularDrag * deltaTime, (f32)0.0f);
                //rb.angularVelocity = Vector3S.Clamp(rb.angularVelocity, -MAX_ANGULAR, MAX_ANGULAR);

                // Update rotation
                if (rb.angularVelocity.SqrMagnitude > f32.epsilon)
                {
                    Vector3S angDelta = rb.angularVelocity * deltaTime;
                    var nrmAng = angDelta.NormalizeWithMagnitude(out var mag);
                    QuaternionS deltaRot = QuaternionS.FromAxisAngle(nrmAng, mag);
                    rb.transform.rotation = (deltaRot * rb.transform.rotation).Normalize();
                }

                // Clear accumulated forces and torques
                rb.ClearBuckets();
            }
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

                //Dyn v stat?
            }
        }

        public struct CollisionInfo
        {
            public RigidbodyS a;

            public RigidbodyS b;

            public ContactS contact;

            public Vector3S tangent;

            public Vector3S frictionImpulse;

            public Vector3S impulse;
        }

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
            f32 bounce = body.material.bounciness;

            // Calculate inverse mass
            f32 invMassB = body.mass > f32.zero ? f32.one / body.mass : f32.zero;

            // Compute the effective mass along the normal direction
            f32 invEffectiveMassB = invMassB + Vector3S.Dot(Vector3S.Cross(body.tensor.inertiaWorld * Vector3S.Cross(rb, contact.normal), rb), contact.normal);

            // Calculate the normal impulse scalar
            f32 j = -(f32.one + bounce) * velocityAlongNormal / invEffectiveMassB;

            // Apply linear impulse
            Vector3S impulse = j * contact.normal;
            velocityBuffer[body.index] += invMassB * impulse;

            // Apply angular impulse
            angularBuffer[body.index] += body.tensor.inertiaWorld * Vector3S.Cross(rb, impulse);

            // Positional correction to prevent sinking
            f32 percent = (f32)0.8f;
            f32 slop = (f32)0.01f;
            f32 penetrationDepth = MathS.Max(contact.penetrationDepth - slop, f32.zero);
            Vector3S correction = (penetrationDepth / invEffectiveMassB) * percent * contact.normal;
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
            f32 bounce = MathS.Max(a.material.bounciness, b.material.bounciness);

            // Calculate inverse masses
            f32 invMassA = a.mass > f32.zero ? f32.one / a.mass : f32.zero;
            f32 invMassB = b.mass > f32.zero ? f32.one / b.mass : f32.zero;

            // Compute the effective mass along the normal direction
            f32 invEffectiveMassA = invMassA + Vector3S.Dot(Vector3S.Cross(a.tensor.inertiaWorld * Vector3S.Cross(ra, contact.normal), ra), contact.normal);
            f32 invEffectiveMassB = invMassB + Vector3S.Dot(Vector3S.Cross(b.tensor.inertiaWorld * Vector3S.Cross(rb, contact.normal), rb), contact.normal);
            f32 invMassSum = invEffectiveMassA + invEffectiveMassB;

            // Calculate the impulse scalar
            f32 j = -(f32.one + bounce) * velocityAlongNormal / invMassSum;

            // Apply linear and angular impulse
            Vector3S impulse = j * contact.normal;
            velocityBuffer[a.index] -= invMassA * impulse;
            velocityBuffer[b.index] += invMassB * impulse;
            angularBuffer[a.index] -= a.tensor.inertiaWorld * Vector3S.Cross(ra, impulse);
            angularBuffer[b.index] += b.tensor.inertiaWorld * Vector3S.Cross(rb, impulse);

            // Positional correction to prevent sinking
            f32 percent = (f32)0.8f; // usually between 0.2 and 0.8
            f32 slop = (f32)0.01f; // usually a small value
            f32 penetrationDepth = MathS.Max(contact.penetrationDepth - slop, f32.zero);
            Vector3S correction = (penetrationDepth / invMassSum) * percent * contact.normal;
            positionBuffer[a.index] -= invMassA * correction;
            positionBuffer[b.index] += invMassB * correction;

            // Calculate relative tangential velocity
            Vector3S relativeTangentialVelocity = relativeVelocityAtContact - (velocityAlongNormal * contact.normal);

            // Define the tangent vector
            Vector3S tangent = relativeTangentialVelocity.Magnitude() > f32.zero ? relativeTangentialVelocity.Normalize() : Vector3S.zero;

            // Calculate the magnitude of the friction impulse
            f32 denominatorA = invMassA + Vector3S.Dot(Vector3S.Cross(a.tensor.inertiaWorld * Vector3S.Cross(ra, tangent), ra), tangent);
            f32 denominatorB = invMassB + Vector3S.Dot(Vector3S.Cross(b.tensor.inertiaWorld * Vector3S.Cross(rb, tangent), rb), tangent);
            f32 denominator = denominatorA + denominatorB;

            f32 jt = -Vector3S.Dot(relativeTangentialVelocity, tangent) / denominator;

            // Coulomb's law of friction
            f32 effectiveFriction = a.material.staticFriction;
            Vector3S frictionImpulse;
            if (MathS.Abs(jt) < j * effectiveFriction)
            {
                frictionImpulse = jt * tangent;
            }
            else
            {
                frictionImpulse = -j * effectiveFriction * tangent;
            }

            // Apply friction impulse
            velocityBuffer[a.index] -= invMassA * frictionImpulse;
            velocityBuffer[b.index] += invMassB * frictionImpulse;
            angularBuffer[a.index] -= a.tensor.inertiaWorld * Vector3S.Cross(ra, frictionImpulse);
            angularBuffer[b.index] += b.tensor.inertiaWorld * Vector3S.Cross(rb, frictionImpulse);
        }


    }
}
