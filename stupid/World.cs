
using System.Collections.Generic;
using stupid.Colliders;
using stupid.Maths;
using stupid.Collections;

namespace stupid
{
    public class World
    {
        public SortAndSweepBroadphase Broadphase { get; set; }
        public SBounds WorldBounds { get; private set; }
        public List<SRigidbody> Rigidbodies { get; private set; }
        public Vector3S Gravity { get; private set; }
        public uint SimulationFrame { get; private set; }
        public DumbGrid<int> DumbGrid { get; private set; }

        private int counter;
        private readonly bool multiThread;
        private readonly Vector3S[] velocityBuffer;
        private readonly Vector3S[] angularVelocityBuffer;
        private readonly Vector3S[] positionBuffer;

        public World(SBounds worldBounds, SortAndSweepBroadphase broadphase, Vector3S gravity, int gridSize = 4, int gridDim = 32, int startSize = 1000, bool multiThread = false)
        {
            counter = 0;
            SimulationFrame = 0;
            WorldBounds = worldBounds;
            this.multiThread = multiThread;
            Gravity = gravity;
            Rigidbodies = new List<SRigidbody>(startSize);
            Broadphase = broadphase;
            velocityBuffer = new Vector3S[startSize * 2];
            angularVelocityBuffer = new Vector3S[startSize * 2];
            positionBuffer = new Vector3S[startSize * 2];
            DumbGrid = new DumbGrid<int>(gridDim, gridDim, gridDim, (f32)gridSize);
        }

        public SRigidbody AddRigidbody(Vector3S position = default, Vector3S velocity = default, Vector3S angularVelocity = default)
        {
            var rb = new SRigidbody(counter++, position, velocity, angularVelocity);
            Rigidbodies.Add(rb);
            return rb;
        }

        public void Simulate(f32 deltaTime)
        {
            Integrate(deltaTime);

            DumbGrid.Invalidate(-1);

            foreach (var body in Rigidbodies)
            {
                body.CalculateInverseInertiaTensor();

                var bounds = body.collider.CalculateBounds(body.position);
                DumbGrid.Add(bounds, body.index);
            }

            var pairs = Broadphase.ComputePairs(Rigidbodies);
            NaiveNarrowPhase(pairs);
            WorldCollision();

            SimulationFrame++;
        }

        private void Integrate(f32 deltaTime)
        {
            foreach (var rb in Rigidbodies)
            {
                if (rb.isKinematic) continue;

                if (rb.useGravity)
                {
                    rb.velocity += Gravity * deltaTime;
                }

                rb.position += rb.velocity * deltaTime;

                // Angular integration
                if (rb.angularVelocity.MagnitudeSquared() > f32.zero)
                {
                    Vector3S angDelta = rb.angularVelocity * deltaTime;

                    var nrmAng = angDelta.NormalizeWithMagnitude(out var mag);
                    SQuaternion deltaRot = SQuaternion.FromAxisAngle(nrmAng, mag);

                    rb.rotation *= deltaRot;
                    rb.angularVelocity *= (f32)0.9f;
                }
            }
        }


        private void NaiveNarrowPhase(HashSet<BodyPair> pairs)
        {
            foreach (var pair in pairs)
            {
                var a = Rigidbodies[pair.aIndex];
                var b = Rigidbodies[pair.bIndex];

                if (a.collider.Intersects(a.position, b.position, b.collider, out var contact))
                {
                    ResolveCollision(a, b, contact);
                }
            }

            ApplyBuffers();
        }

        private void ResolveCollision(SRigidbody a, SRigidbody b, Contact contact)
        {
            // Calculate relative velocity at contact point
            Vector3S relativeVelocity = b.velocity - a.velocity;
            f32 velocityAlongNormal = Vector3S.Dot(relativeVelocity, contact.normal);

            // Do not resolve if velocities are separating
            if (velocityAlongNormal > f32.zero) return;

            // Restitution (coefficient of restitution)
            f32 e = f32.half;

            // Calculate inverse masses
            f32 invMassA = f32.one / a.mass;
            f32 invMassB = f32.one / b.mass;
            f32 invMassSum = invMassA + invMassB;

            // Calculate the impulse scalar
            f32 j = -(f32.one + e) * velocityAlongNormal / invMassSum;

            // Apply linear impulse
            Vector3S impulse = j * contact.normal;
            Vector3S invMassImpulseA = invMassA * impulse;
            Vector3S invMassImpulseB = invMassB * impulse;
            velocityBuffer[a.index] -= invMassImpulseA;
            velocityBuffer[b.index] += invMassImpulseB;

            // Positional correction to prevent sinking
            f32 percent = (f32)0.2f; // Percentage of penetration to correct
            f32 slop = (f32)0.01f; // Allowable penetration slop
            f32 penetrationDepth = MathS.Max(contact.penetrationDepth - slop, f32.zero);
            Vector3S correction = (penetrationDepth / invMassSum) * percent * contact.normal;
            positionBuffer[a.index] -= invMassA * correction;
            positionBuffer[b.index] += invMassB * correction;

            // Angular velocity handling
            Vector3S ra = contact.point - a.position;
            Vector3S rb = contact.point - b.position;
            Matrix3S inverseInertiaTensorA = a.inertiaTensorInverse;
            Matrix3S inverseInertiaTensorB = b.inertiaTensorInverse;

            Vector3S raCrossImpulse = Vector3S.Cross(ra, impulse);
            Vector3S rbCrossImpulse = Vector3S.Cross(rb, impulse);
            angularVelocityBuffer[a.index] -= inverseInertiaTensorA * raCrossImpulse;
            angularVelocityBuffer[b.index] += inverseInertiaTensorB * rbCrossImpulse;

            // Calculate relative tangential velocity
            Vector3S relativeTangentialVelocity = relativeVelocity + Vector3S.Cross(a.angularVelocity, ra) - Vector3S.Cross(b.angularVelocity, rb);

            // Calculate friction impulse
            Vector3S tangent = (relativeTangentialVelocity - Vector3S.Dot(relativeTangentialVelocity, contact.normal) * contact.normal).Normalize();
            f32 jt = -Vector3S.Dot(relativeTangentialVelocity, tangent) / invMassSum;
            f32 mu = f32.half;
            Vector3S frictionImpulse = MathS.Abs(jt) < j * mu ? jt * tangent : -j * mu * tangent;

            // Apply friction impulse
            Vector3S invMassFrictionImpulseA = invMassA * frictionImpulse;
            Vector3S invMassFrictionImpulseB = invMassB * frictionImpulse;
            velocityBuffer[a.index] -= invMassFrictionImpulseA;
            velocityBuffer[b.index] += invMassFrictionImpulseB;

            raCrossImpulse = Vector3S.Cross(ra, frictionImpulse);
            rbCrossImpulse = Vector3S.Cross(rb, frictionImpulse);
            angularVelocityBuffer[a.index] -= inverseInertiaTensorA * raCrossImpulse;
            angularVelocityBuffer[b.index] += inverseInertiaTensorB * rbCrossImpulse;
        }

        private void ApplyBuffers()
        {
            foreach (var a in Rigidbodies)
            {
                a.velocity += velocityBuffer[a.index];
                a.angularVelocity += angularVelocityBuffer[a.index];
                a.position += positionBuffer[a.index];

                velocityBuffer[a.index] = Vector3S.zero;
                positionBuffer[a.index] = Vector3S.zero;
                angularVelocityBuffer[a.index] = Vector3S.zero;
            }
        }

        private void WorldCollision()
        {
            foreach (var rb in Rigidbodies)
            {
                var bounds = rb.collider.GetBounds();
                if (WorldBounds.ContainsBounds(bounds)) continue;

                var sc = (SSphereCollider)rb.collider;
                CheckAxisCollision(ref rb.position.x, ref rb.velocity.x, bounds.min.x, bounds.max.x, WorldBounds.min.x, WorldBounds.max.x, sc.Radius);
                CheckAxisCollision(ref rb.position.y, ref rb.velocity.y, bounds.min.y, bounds.max.y, WorldBounds.min.y, WorldBounds.max.y, sc.Radius);
                CheckAxisCollision(ref rb.position.z, ref rb.velocity.z, bounds.min.z, bounds.max.z, WorldBounds.min.z, WorldBounds.max.z, sc.Radius);
            }
        }

        private void CheckAxisCollision(ref f32 position, ref f32 velocity, f32 minBound, f32 maxBound, f32 worldMin, f32 worldMax, f32 radius)
        {
            if (minBound < worldMin)
            {
                velocity *= f32.negativeOne * f32.half;
                position = worldMin + radius;
            }
            else if (maxBound > worldMax)
            {
                velocity *= f32.negativeOne * f32.half;
                position = worldMax - radius;
            }
        }
    }
}
