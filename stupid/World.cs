using System;
using System.Collections.Generic;
using System.Security.Cryptography;
using System.Text;
using System.Threading.Tasks;
using SoftFloat;
using stupid.Colliders;
using stupid.Maths;

namespace stupid
{
    public class World
    {
        public IBroadphase Broadphase { get; set; }
        public SBounds WorldBounds { get; private set; }
        public List<SRigidbody> Rigidbodies { get; private set; }
        public Vector3S Gravity { get; private set; }
        public uint SimulationFrame { get; private set; }

        private int counter;
        private readonly bool multiThread;
        private readonly Vector3S[] velocityBuffer;
        private readonly Vector3S[] positionBuffer;

        public World(SBounds worldBounds, IBroadphase broadphase, Vector3S gravity, int startSize = 1000, bool multiThread = false)
        {
            counter = 0;
            SimulationFrame = 0;
            WorldBounds = worldBounds;
            this.multiThread = multiThread;
            Gravity = gravity;
            Rigidbodies = new List<SRigidbody>(startSize);
            Broadphase = broadphase;
            velocityBuffer = new Vector3S[startSize * 2];
            positionBuffer = new Vector3S[startSize * 2];
        }

        public SRigidbody AddRigidbody(Vector3S position = default, Vector3S velocity = default)
        {
            var rb = new SRigidbody(counter++, position, velocity);
            Rigidbodies.Add(rb);
            return rb;
        }

        public void Simulate(sfloat deltaTime)
        {
            Integrate(deltaTime);

            foreach (var body in Rigidbodies)
            {
                body.collider.CalculateBounds(body.position);
            }

            var pairs = Broadphase.ComputePairs(Rigidbodies);
            NaiveNarrowPhase(pairs);
            WorldCollision();


            SimulationFrame++;
        }

        private void Integrate(sfloat deltaTime)
        {
            foreach (var rb in Rigidbodies)
            {
                if (rb.isKinematic) continue;

                if (rb.useGravity)
                {
                    rb.velocity += Gravity * deltaTime;
                }

                rb.position += rb.velocity * deltaTime;
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

            foreach (var pair in pairs)
            {
                var a = Rigidbodies[pair.aIndex];
                var b = Rigidbodies[pair.bIndex];

                a.velocity += velocityBuffer[a.index];
                a.position += positionBuffer[a.index];

                b.velocity += velocityBuffer[b.index];
                b.position += positionBuffer[b.index];

                velocityBuffer[a.index] = Vector3S.zero;
                positionBuffer[a.index] = Vector3S.zero;

                velocityBuffer[b.index] = Vector3S.zero;
                positionBuffer[b.index] = Vector3S.zero;
            }
        }

        private void ResolveCollision(SRigidbody a, SRigidbody b, Contact contact)
        {
            Vector3S relativeVelocity = b.velocity - a.velocity;
            sfloat velocityAlongNormal = Vector3S.Dot(relativeVelocity, contact.normal);

            if (velocityAlongNormal > sfloat.zero) return;

            sfloat e = (sfloat)0.5f;
            sfloat invMassA = (sfloat.one / a.mass);
            sfloat invMassB = (sfloat.one / b.mass);
            sfloat j = -(sfloat.one + e) * velocityAlongNormal / (invMassA + invMassB);

            Vector3S impulse = j * contact.normal;
            AddToBuffer(a, b, impulse, invMassA, invMassB);

            Vector3S tangent = (relativeVelocity - (velocityAlongNormal * contact.normal)).Normalize();
            sfloat jt = -Vector3S.Dot(relativeVelocity, tangent) / (invMassA + invMassB);

            sfloat mu = (sfloat)0.5f;
            Vector3S frictionImpulse = MathS.Abs(jt) < j * mu ? jt * tangent : -j * mu * tangent;
            AddToBuffer(a, b, frictionImpulse, invMassA, invMassB);

            CorrectPositions(a, b, contact, invMassA, invMassB);
        }

        private void AddToBuffer(SRigidbody a, SRigidbody b, Vector3S impulse, sfloat invMassA, sfloat invMassB)
        {
            velocityBuffer[a.index] -= invMassA * impulse;
            velocityBuffer[b.index] += invMassB * impulse;
        }

        private void CorrectPositions(SRigidbody a, SRigidbody b, Contact contact, sfloat invMassA, sfloat invMassB)
        {
            sfloat percent = (sfloat)0.2f;
            sfloat slop = (sfloat)0.01f;
            Vector3S correction = MathS.Max(contact.penetrationDepth - slop, sfloat.zero) / (invMassA + invMassB) * percent * contact.normal;

            positionBuffer[a.index] -= invMassA * correction;
            positionBuffer[b.index] += invMassB * correction;
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

                rb.velocity.x *= (sfloat)0.9f;
                rb.velocity.z *= (sfloat)0.9f;
            }
        }

        private void CheckAxisCollision(ref sfloat position, ref sfloat velocity, sfloat minBound, sfloat maxBound, sfloat worldMin, sfloat worldMax, sfloat radius)
        {
            if (minBound < worldMin)
            {
                velocity *= sfloat.MinusOne * (sfloat)0.5f;
                position = worldMin + radius;
            }
            else if (maxBound > worldMax)
            {
                velocity *= sfloat.MinusOne * (sfloat)0.5f;
                position = worldMax - radius;
            }
        }

    }
}
