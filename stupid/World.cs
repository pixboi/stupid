
using System.Collections.Generic;
using stupid.Colliders;
using stupid.Maths;
using stupid.Collections;

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
        private readonly Vector3S[] velocityBuffer;
        private readonly Vector3S[] angularVelocityBuffer;
        private readonly Vector3S[] positionBuffer;

        public World(BoundsS worldBounds, SortAndSweepBroadphase broadphase, Vector3S gravity, int startSize = 1000)
        {
            counter = 0;
            SimulationFrame = 0;
            WorldBounds = worldBounds;
            Gravity = gravity;
            Collidables = new List<Collidable>(startSize);
            DumbGrid = new DumbGrid<int>(32, 32, 32, (f32)4);

            Broadphase = broadphase;

            //These need to resize on rigid body adds
            velocityBuffer = new Vector3S[startSize * 2];
            angularVelocityBuffer = new Vector3S[startSize * 2];
            positionBuffer = new Vector3S[startSize * 2];
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
                true, false
                );
            Collidables.Add(rb);
            return rb;
        }

        public void Simulate(f32 deltaTime)
        {
            Integrate(deltaTime);

            //DumbGrid.Invalidate(-1);

            foreach (var body in Collidables)
            {
                if (body is RigidbodyS rb)
                {
                    rb.tensor.CalculateInverseInertiaTensor(rb.transform.rotation);
                }

                var bounds = body.CalculateBounds();
                // DumbGrid.Add(bounds, body.index);
            }

            var pairs = Broadphase.ComputePairs(Collidables);
            NarrowPhase(pairs);
            WorldCollision();

            ApplyBuffers();

            SimulationFrame++;
        }

        private void Integrate(f32 deltaTime)
        {
            foreach (RigidbodyS rb in Collidables)
            {
                if (rb.isKinematic) continue;

                if (rb.useGravity)
                {
                    rb.velocity += Gravity * deltaTime;
                }

                rb.transform.position += rb.velocity * deltaTime;

                // Angular integration
                if (rb.angularVelocity.SqrMagnitude > f32.zero)
                {
                    Vector3S angDelta = rb.angularVelocity * deltaTime;

                    var nrmAng = angDelta.NormalizeWithMagnitude(out var mag);
                    QuaternionS deltaRot = QuaternionS.FromAxisAngle(nrmAng, mag);

                    rb.transform.rotation *= deltaRot;
                    rb.angularVelocity *= (f32)0.95f;
                }
            }
        }


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
                        ResolveCollision(ab, bb, contact);
                    }
                }

                //Dyn v stat?
            }

        }
        private void ResolveCollisionStatic(RigidbodyS a, ContactS contact)
        {
            // Full positional correction to prevent sinking
            Vector3S correction = contact.penetrationDepth * contact.normal;
            positionBuffer[a.index] += correction;

            // Reflect velocity off the collision normal
            Vector3S velocity = velocityBuffer[a.index];
            Vector3S reflectedVelocity = velocity - (f32.two * Vector3S.Dot(velocity, contact.normal) * contact.normal);

            // Apply the reflected velocity with restitution
            velocityBuffer[a.index] = reflectedVelocity * a.material.bounciness;

            // Apply angular velocity correction
            Vector3S ra = contact.point - a.transform.position;
            Vector3S angularImpulse = a.tensor.inertiaWorld * Vector3S.Cross(ra, reflectedVelocity * a.material.bounciness);
            angularVelocityBuffer[a.index] += angularImpulse;

            // Small position shift to ensure no sticking to the border
            positionBuffer[a.index] += contact.normal * f32.epsilon;
        }



        private void ResolveCollision(RigidbodyS a, RigidbodyS b, ContactS contact)
        {
            // Calculate relative velocity at contact point
            Vector3S relativeVelocity = b.velocity - a.velocity;
            Vector3S ra = contact.point - a.transform.position;
            Vector3S rb = contact.point - b.transform.position;

            // Include the angular velocities in the relative velocity calculation
            Vector3S relativeVelocityAtContact = relativeVelocity
                + Vector3S.Cross(a.angularVelocity, ra)
                - Vector3S.Cross(b.angularVelocity, rb);

            f32 velocityAlongNormal = Vector3S.Dot(relativeVelocityAtContact, contact.normal);

            // Do not resolve if velocities are separating
            if (velocityAlongNormal > f32.zero) return;

            // Restitution (coefficient of restitution)
            f32 bounce = (a.material.bounciness + b.material.bounciness) * f32.half; // This can be parameterized for different materials

            // Calculate inverse masses
            f32 invMassA = f32.one / a.mass;
            f32 invMassB = f32.one / b.mass;

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
            angularVelocityBuffer[a.index] -= a.tensor.inertiaWorld * Vector3S.Cross(ra, impulse);
            angularVelocityBuffer[b.index] += b.tensor.inertiaWorld * Vector3S.Cross(rb, impulse);

            // Positional correction to prevent sinking
            f32 percent = (f32)0.5f; // Percentage of penetration to correct
            f32 slop = (f32)0.01f; // Allowable penetration slop
            f32 penetrationDepth = MathS.Max(contact.penetrationDepth - slop, f32.zero);
            Vector3S correction = (penetrationDepth / invMassSum) * percent * contact.normal;
            positionBuffer[a.index] -= invMassA * correction;
            positionBuffer[b.index] += invMassB * correction;

            // Calculate relative tangential velocity
            Vector3S relativeTangentialVelocity = relativeVelocityAtContact - (velocityAlongNormal * contact.normal);

            // Calculate friction impulse
            Vector3S tangent = relativeTangentialVelocity.Normalize();
            f32 jt = -Vector3S.Dot(relativeTangentialVelocity, tangent) / invMassSum;
            f32 effectiveFriction = (a.material.friction + b.material.friction) * f32.half; // This can be parameterized for different materials
            Vector3S frictionImpulse = MathS.Abs(jt) < j * effectiveFriction ? jt * tangent : -j * effectiveFriction * tangent;

            // Apply friction impulse
            velocityBuffer[a.index] -= invMassA * frictionImpulse;
            velocityBuffer[b.index] += invMassB * frictionImpulse;
            angularVelocityBuffer[a.index] -= a.tensor.inertiaWorld * Vector3S.Cross(ra, frictionImpulse);
            angularVelocityBuffer[b.index] += b.tensor.inertiaWorld * Vector3S.Cross(rb, frictionImpulse);
        }



        private void ApplyBuffers()
        {
            foreach (var c in Collidables)
            {
                if (c is RigidbodyS body)
                {
                    body.velocity += velocityBuffer[c.index];
                    body.angularVelocity += angularVelocityBuffer[c.index];
                    body.transform.position += positionBuffer[c.index];

                    velocityBuffer[c.index] = Vector3S.zero;
                    positionBuffer[c.index] = Vector3S.zero;
                    angularVelocityBuffer[c.index] = Vector3S.zero;
                }

            }
        }

        private void WorldCollision()
        {
            foreach (var col in Collidables)
            {
                if (col is RigidbodyS rb)
                {
                    var bounds = rb.GetBounds();
                    var position = rb.transform.position;
                    var velocity = rb.velocity;

                    // Check for collisions along the x-axis
                    if (bounds.min.x < WorldBounds.min.x)
                    {
                        var newX = WorldBounds.min.x + bounds.halfSize.x;
                        var newPos = new Vector3S(newX, position.y, position.z);
                        var nrm = newPos - position;
                        positionBuffer[rb.index] += nrm;

                        var newXVel = velocity.x * -f32.half;
                        var newVel = new Vector3S(newXVel, velocity.y, velocity.z);
                        var Vn = newVel - velocity;
                        velocityBuffer[rb.index] += Vn;
                    }
                    else if (bounds.max.x > WorldBounds.max.x)
                    {
                        var newX = WorldBounds.max.x - bounds.halfSize.x;
                        var newPos = new Vector3S(newX, position.y, position.z);
                        var nrm = newPos - position;
                        positionBuffer[rb.index] += nrm;

                        var newXVel = velocity.x * -f32.half;
                        var newVel = new Vector3S(newXVel, velocity.y, velocity.z);
                        var Vn = newVel - velocity;
                        velocityBuffer[rb.index] += Vn;
                    }

                    // Check for collisions along the y-axis
                    if (bounds.min.y < WorldBounds.min.y)
                    {
                        var newY = WorldBounds.min.y + bounds.halfSize.y;
                        var newPos = new Vector3S(position.x, newY, position.z);
                        var nrm = newPos - position;
                        positionBuffer[rb.index] += nrm;

                        var newYVel = velocity.y * -f32.half;
                        var newVel = new Vector3S(velocity.x, newYVel, velocity.z);
                        var Vn = newVel - velocity;
                        velocityBuffer[rb.index] += Vn;
                    }
                    else if (bounds.max.y > WorldBounds.max.y)
                    {
                        var newY = WorldBounds.max.y - bounds.halfSize.y;
                        var newPos = new Vector3S(position.x, newY, position.z);
                        var nrm = newPos - position;
                        positionBuffer[rb.index] += nrm;

                        var newYVel = velocity.y * -f32.half;
                        var newVel = new Vector3S(velocity.x, newYVel, velocity.z);
                        var Vn = newVel - velocity;
                        velocityBuffer[rb.index] += Vn;
                    }

                    // Check for collisions along the z-axis
                    if (bounds.min.z < WorldBounds.min.z)
                    {
                        var newZ = WorldBounds.min.z + bounds.halfSize.z;
                        var newPos = new Vector3S(position.x, position.y, newZ);
                        var nrm = newPos - position;
                        positionBuffer[rb.index] += nrm;

                        var newZVel = velocity.z * -f32.half;
                        var newVel = new Vector3S(velocity.x, velocity.y, newZVel);
                        var Vn = newVel - velocity;
                        velocityBuffer[rb.index] += Vn;
                    }
                    else if (bounds.max.z > WorldBounds.max.z)
                    {
                        var newZ = WorldBounds.max.z - bounds.halfSize.z;
                        var newPos = new Vector3S(position.x, position.y, newZ);
                        var nrm = newPos - position;
                        positionBuffer[rb.index] += nrm;

                        var newZVel = velocity.z * -f32.half;
                        var newVel = new Vector3S(velocity.x, velocity.y, newZVel);
                        var Vn = newVel - velocity;
                        velocityBuffer[rb.index] += Vn;
                    }
                }
            }
        }





    }
}
