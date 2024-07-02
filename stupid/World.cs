using System;
using System.Collections.Generic;
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

        public World(Bounds worldBounds, IBroadphase broadphase, Vector3S gravity, int startSize = 1000, bool multiThread = false)
        {
            this.counter = 0;
            this.WorldBounds = worldBounds;
            this.multiThread = multiThread;
            this.Gravity = gravity;
            this.Rigidbodies = new List<Rigidbody>(startSize);
            this.Broadphase = broadphase;
        }

        public Rigidbody AddRigidbody(Vector3S position, Vector3S velocity)
        {
            var rb = new Rigidbody(counter++, position, velocity);
            Rigidbodies.Add(rb);
            return rb;
        }

        private void AddGravity(sfloat deltaTime)
        {
            foreach (var rb in Rigidbodies)
            {
                if (rb.isSleeping || !rb.useGravity) continue;
                rb.velocity += Gravity * deltaTime;
            }
        }

        public void WorldCollision()
        {
            foreach (var rb in Rigidbodies)
            {
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

        private Dictionary<Rigidbody, Vector3S> velocityBuffer = new Dictionary<Rigidbody, Vector3S>();
        private Dictionary<Rigidbody, Vector3S> positionBuffer = new Dictionary<Rigidbody, Vector3S>();

        private void NaiveNarrowPhase(HashSet<BodyPair> pairs)
        {
            velocityBuffer.Clear();
            positionBuffer.Clear();

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
            if (!velocityBuffer.ContainsKey(a)) velocityBuffer[a] = Vector3S.zero;
            if (!velocityBuffer.ContainsKey(b)) velocityBuffer[b] = Vector3S.zero;

            velocityBuffer[a] -= invMassA * impulse;
            velocityBuffer[b] += invMassB * impulse;
        }

        private void CorrectPositions(Rigidbody a, Rigidbody b, Contact contact, sfloat invMassA, sfloat invMassB)
        {
            sfloat percent = (sfloat)0.2f;
            sfloat slop = (sfloat)0.01f;
            Vector3S correction = MathS.Max(contact.penetrationDepth - slop, sfloat.zero) / (invMassA + invMassB) * percent * contact.normal;

            if (!positionBuffer.ContainsKey(a)) positionBuffer[a] = Vector3S.zero;
            if (!positionBuffer.ContainsKey(b)) positionBuffer[b] = Vector3S.zero;

            positionBuffer[a] -= invMassA * correction;
            positionBuffer[b] += invMassB * correction;
        }

        private void ApplyBuffers()
        {
            foreach (var change in velocityBuffer)
            {
                change.Key.velocity += change.Value;
            }

            foreach (var change in positionBuffer)
            {
                change.Key.position += change.Value;
            }
        }

        private void Integrate(sfloat deltaTime)
        {
            foreach (var rb in Rigidbodies)
            {
                if (rb.isSleeping) continue;
                rb.position += rb.velocity * deltaTime;
            }
        }

        public void CheckSleep(Rigidbody body)
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

        public void Simulate(sfloat deltaTime)
        {
            AddGravity(deltaTime);
            Integrate(deltaTime);

            if (multiThread)
            {
                // Calculating bounds can be parallelized
                Parallel.ForEach(Rigidbodies, body =>
                {
                    body.collider.CalculateBounds(body.position);
                });
            }
            else
            {
                foreach (var body in Rigidbodies)
                {
                    body.collider.CalculateBounds(body.position);
                }
            }

            var pairs = Broadphase.ComputePairs(Rigidbodies);
            NaiveNarrowPhase(pairs);
            WorldCollision();

            if (multiThread)
            {
                // Checking sleep can be parallelized
                Parallel.ForEach(Rigidbodies, body =>
                {
                    CheckSleep(body);
                });
            }
            else
            {
                foreach (var body in Rigidbodies)
                {
                    CheckSleep(body);
                }
            }
        }
    }
}
