using System;
using System.Collections.Generic;
using System.Security.Cryptography;
using System.Text;
using System.Threading.Tasks;
using SoftFloat;

namespace stupid
{
    public class World
    {
        public IBroadphase Broadphase { get; set; }
        public Bounds WorldBounds { get; private set; }
        public List<Rigidbody> Rigidbodies { get; private set; }
        public Vector3S Gravity { get; private set; }

        private int counter;
        private readonly bool multiThread;

        public string CalculateStateHash()
        {
            using (var sha256 = SHA256.Create())
            {
                var sb = new StringBuilder();
                foreach (var rb in Rigidbodies)
                {
                    sb.Append(rb.position.x.ToString());
                    sb.Append(rb.position.y.ToString());
                    sb.Append(rb.position.z.ToString());
                    sb.Append(rb.velocity.x.ToString());
                    sb.Append(rb.velocity.y.ToString());
                    sb.Append(rb.velocity.z.ToString());
                }

                var hashBytes = sha256.ComputeHash(Encoding.UTF8.GetBytes(sb.ToString()));
                return BitConverter.ToString(hashBytes).Replace("-", "").ToLower();
            }
        }

        public World(Bounds worldBounds, IBroadphase broadphase, Vector3S gravity, int startSize = 1000, bool multiThread = false)
        {
            this.counter = 0;
            this.SimulationFrame = 0;
            this.WorldBounds = worldBounds;
            this.multiThread = multiThread;
            this.Gravity = gravity;
            this.Rigidbodies = new List<Rigidbody>(startSize);
            this.Broadphase = broadphase;
            this.velocityBuffer = new Vector3S[startSize * 2];
            this.positionBuffer = new Vector3S[startSize * 2];
        }

        public Rigidbody AddRigidbody(Vector3S position, Vector3S velocity)
        {
            var rb = new Rigidbody(counter++, position, velocity);
            Rigidbodies.Add(rb);
            return rb;
        }

        public void WorldCollision()
        {
            foreach (var rb in Rigidbodies)
            {
                if (rb.isSleeping) continue;
                
                var bounds = rb.collider.GetBounds();
                if (WorldBounds.ContainsBounds(bounds)) continue;

                var sc = (SphereCollider)rb.collider;
                CheckAxisCollision(ref rb.position.x, ref rb.velocity.x, bounds.Min.x, bounds.Max.x, WorldBounds.Min.x, WorldBounds.Max.x, sc.radius);
                CheckAxisCollision(ref rb.position.y, ref rb.velocity.y, bounds.Min.y, bounds.Max.y, WorldBounds.Min.y, WorldBounds.Max.y, sc.radius);
                CheckAxisCollision(ref rb.position.z, ref rb.velocity.z, bounds.Min.z, bounds.Max.z, WorldBounds.Min.z, WorldBounds.Max.z, sc.radius);

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

        private Vector3S[] velocityBuffer;
        private Vector3S[] positionBuffer;

        private void NaiveNarrowPhase(HashSet<BodyPair> pairs)
        {
            foreach (var pair in pairs)
            {
                var a = Rigidbodies[pair.aIndex];
                var b = Rigidbodies[pair.bIndex];

                if (a.isSleeping && b.isSleeping) { continue; }

                if (a.collider.Intersects(a.position, b.position, b.collider, out var contact))
                {
                    ResolveCollision(a, b, contact);
                }
            }

            foreach (var rb in Rigidbodies)
            {
                rb.velocity += velocityBuffer[rb.index];
                rb.position += positionBuffer[rb.index];

                velocityBuffer[rb.index] = Vector3S.zero;
                positionBuffer[rb.index] = Vector3S.zero;
            }

        }

        private void ResolveCollision(Rigidbody a, Rigidbody b, Contact contact)
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

        private void AddToBuffer(Rigidbody a, Rigidbody b, Vector3S impulse, sfloat invMassA, sfloat invMassB)
        {
            velocityBuffer[a.index] -= invMassA * impulse;
            velocityBuffer[b.index] += invMassB * impulse;
        }

        private void CorrectPositions(Rigidbody a, Rigidbody b, Contact contact, sfloat invMassA, sfloat invMassB)
        {
            sfloat percent = (sfloat)0.2f;
            sfloat slop = (sfloat)0.01f;
            Vector3S correction = MathS.Max(contact.penetrationDepth - slop, sfloat.zero) / (invMassA + invMassB) * percent * contact.normal;

            positionBuffer[a.index] -= invMassA * correction;
            positionBuffer[b.index] += invMassB * correction;
        }

        public uint SimulationFrame { get; private set; }
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

            foreach (var body in Rigidbodies)
            {
                CheckSleep(body);
            }

            SimulationFrame++;
        }

        private void Integrate(sfloat deltaTime)
        {
            foreach (var rb in Rigidbodies)
            {
                if (rb.isSleeping) continue;

                if (rb.useGravity)
                {
                    rb.velocity += Gravity * deltaTime;
                }

                rb.position += rb.velocity * deltaTime;
            }
        }

        void CheckSleep(Rigidbody body)
        {
            var v = body.velocity.MagnitudeSquared();
            if (body.isSleeping)
            {
                if (v > body.sleepThreshold)
                {
                    body.WakeUp();
                }
            }
            else
            {
                if (v <= body.sleepThreshold)
                {
                    body.Sleep();
                }
            }
        }
    }
}
