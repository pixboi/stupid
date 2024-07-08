
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

        public AABBTree AABBTree { get; private set; }
        public DumbGrid<int> DumbGrid { get; private set; }

        private int counter;
        private readonly Vector3S[] velocityBuffer;
        private readonly Vector3S[] angularVelocityBuffer;
        private readonly Vector3S[] positionBuffer;

        public World(SBounds worldBounds, SortAndSweepBroadphase broadphase, Vector3S gravity, int startSize = 1000)
        {
            counter = 0;
            SimulationFrame = 0;
            WorldBounds = worldBounds;
            Gravity = gravity;
            Rigidbodies = new List<SRigidbody>(startSize);

            Broadphase = broadphase;
            AABBTree = new AABBTree(Rigidbodies);

            //These need to resize on rigid body adds
            velocityBuffer = new Vector3S[startSize * 2];
            angularVelocityBuffer = new Vector3S[startSize * 2];
            positionBuffer = new Vector3S[startSize * 2];
        }

        public SRigidbody AddRigidbody(ICollider collider, Vector3S position = default, Vector3S velocity = default, Vector3S angularVelocity = default)
        {
            var rb = new SRigidbody(counter++, position, velocity, angularVelocity);

            rb.Attach(collider);
            AABBTree.Insert(rb);

            Rigidbodies.Add(rb);
            return rb;
        }


        List<RaycastHit> _hits = new List<RaycastHit>();
        void RayTest()
        {
            var ray = new Ray(Vector3S.zero, Vector3S.one * (f32)32);
            var _hits = AABBTree.QueryRay(ray);
            BruteRay(ray);
        }

        void BruteRay(Ray ray)
        {
            foreach (var body in Rigidbodies)
            {
                if (body.collider.GetBounds().IntersectRay(ray))
                {

                }
            }
        }

        public void Simulate(f32 deltaTime)
        {
            Integrate(deltaTime);

            foreach (var body in Rigidbodies)
            {
                body.CalculateInverseInertiaTensor();
                var bounds = body.collider.CalculateBounds(body.position);
            }

            AABBTree.Rebuild(Rigidbodies);

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
                if (rb.angularVelocity.SqrMagnitude > f32.zero)
                {
                    Vector3S angDelta = rb.angularVelocity * deltaTime;

                    var nrmAng = angDelta.NormalizeWithMagnitude(out var mag);
                    SQuaternion deltaRot = SQuaternion.FromAxisAngle(nrmAng, mag);

                    rb.rotation = deltaRot * rb.rotation;
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
            Vector3S ra = contact.point - a.position;
            Vector3S rb = contact.point - b.position;

            // Include the angular velocities in the relative velocity calculation
            Vector3S relativeVelocityAtContact = relativeVelocity
                + Vector3S.Cross(a.angularVelocity, ra)
                - Vector3S.Cross(b.angularVelocity, rb);

            f32 velocityAlongNormal = Vector3S.Dot(relativeVelocityAtContact, contact.normal);

            // Do not resolve if velocities are separating
            if (velocityAlongNormal > f32.zero) return;

            // Restitution (coefficient of restitution)
            f32 e = f32.half; // This can be parameterized for different materials

            // Calculate inverse masses
            f32 invMassA = f32.one / a.mass;
            f32 invMassB = f32.one / b.mass;

            // Compute the effective mass along the normal direction
            f32 invEffectiveMassA = invMassA + Vector3S.Dot(Vector3S.Cross(a.inertiaTensorInverse * Vector3S.Cross(ra, contact.normal), ra), contact.normal);
            f32 invEffectiveMassB = invMassB + Vector3S.Dot(Vector3S.Cross(b.inertiaTensorInverse * Vector3S.Cross(rb, contact.normal), rb), contact.normal);
            f32 invMassSum = invEffectiveMassA + invEffectiveMassB;

            // Calculate the impulse scalar
            f32 j = -(f32.one + e) * velocityAlongNormal / invMassSum;

            // Apply linear and angular impulse
            Vector3S impulse = j * contact.normal;
            velocityBuffer[a.index] -= invMassA * impulse;
            velocityBuffer[b.index] += invMassB * impulse;
            angularVelocityBuffer[a.index] -= a.inertiaTensorInverse * Vector3S.Cross(ra, impulse);
            angularVelocityBuffer[b.index] += b.inertiaTensorInverse * Vector3S.Cross(rb, impulse);

            // Positional correction to prevent sinking
            f32 percent = (f32)1f; // Percentage of penetration to correct
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
            f32 mu = f32.half; // This can be parameterized for different materials
            Vector3S frictionImpulse = MathS.Abs(jt) < j * mu ? jt * tangent : -j * mu * tangent;

            // Apply friction impulse
            velocityBuffer[a.index] -= invMassA * frictionImpulse;
            velocityBuffer[b.index] += invMassB * frictionImpulse;
            angularVelocityBuffer[a.index] -= a.inertiaTensorInverse * Vector3S.Cross(ra, frictionImpulse);
            angularVelocityBuffer[b.index] += b.inertiaTensorInverse * Vector3S.Cross(rb, frictionImpulse);
        }



        private void ApplyBuffers()
        {
            foreach (var body in Rigidbodies)
            {
                body.velocity += velocityBuffer[body.index];
                body.angularVelocity += angularVelocityBuffer[body.index];
                body.position += positionBuffer[body.index];

                velocityBuffer[body.index] = Vector3S.zero;
                positionBuffer[body.index] = Vector3S.zero;
                angularVelocityBuffer[body.index] = Vector3S.zero;
            }
        }

        private void WorldCollision()
        {
            foreach (var rb in Rigidbodies)
            {
                var bounds = rb.collider.GetBounds();
                if (WorldBounds.Contains(bounds)) continue;

                var sc = (SSphereCollider)rb.collider;
                CheckAxisCollision(ref rb.position.x, ref rb.velocity.x, bounds.min.x, bounds.max.x, WorldBounds.min.x, WorldBounds.max.x, sc.Radius);
                CheckAxisCollision(ref rb.position.y, ref rb.velocity.y, bounds.min.y, bounds.max.y, WorldBounds.min.y, WorldBounds.max.y, sc.Radius);
                CheckAxisCollision(ref rb.position.z, ref rb.velocity.z, bounds.min.z, bounds.max.z, WorldBounds.min.z, WorldBounds.max.z, sc.Radius);

                rb.velocity.x *= (f32)0.95;
                rb.velocity.z *= (f32)0.95;

                rb.angularVelocity *= (f32)0.95;
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
