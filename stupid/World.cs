using System;
using System.Collections.Generic;
using System.Runtime.CompilerServices;
using System.Threading.Tasks;
using SoftFloat;

namespace stupid
{
    public class World
    {
        public IBroadphase Broadphase { get; set; }
        public Bounds worldBounds { get; private set; }
        public List<Rigidbody> Rigidbodies { get; private set; }

        private int counter;
        public Rigidbody AddRigidbody(Vector3S position, Vector3S velocity)
        {
            var rb = new Rigidbody(counter++, position, velocity);
            Rigidbodies.Add(rb);
            return rb;
        }

        bool _multiThread;
        public World(Bounds worldBounds, IBroadphase broadphase, int startSize = 1000, bool multiThread = false)
        {
            counter = 0;
            this.worldBounds = worldBounds;
            this._multiThread = multiThread;
            Rigidbodies = new List<Rigidbody>(startSize);

            Broadphase = broadphase;
        }

        void AddGravity(sfloat deltaTime)
        {
            foreach (var rb in Rigidbodies)
            {
                if (rb.isSleeping) continue;

                if (rb.useGravity)
                {
                    rb.velocity += new Vector3S(0f, -20f, 0f) * deltaTime;
                }
            }
        }

        public void WorldCollision()
        {
            foreach (var rb in Rigidbodies)
            {
                var bounds = rb.collider.GetBounds();
                if (worldBounds.ContainsBounds(bounds)) continue;

                var sc = (SphereCollider)rb.collider;

                CheckAxisCollision(ref rb.position.x, ref rb.velocity.x, bounds.Min.x, bounds.Max.x, worldBounds.Min.x, worldBounds.Max.x, sc.radius);
                CheckAxisCollision(ref rb.position.y, ref rb.velocity.y, bounds.Min.y, bounds.Max.y, worldBounds.Min.y, worldBounds.Max.y, sc.radius);
                CheckAxisCollision(ref rb.position.z, ref rb.velocity.z, bounds.Min.z, bounds.Max.z, worldBounds.Min.z, worldBounds.Max.z, sc.radius);

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

        List<ContactPair> BruteForceBroadphase()
        {
            var pairs = new List<ContactPair>();
            //The integration must be halved for like each iteration count
            for (int i = 0; i < Rigidbodies.Count; i++)
            {
                for (int j = i + 1; j < Rigidbodies.Count; j++)
                {
                    var a = Rigidbodies[i];
                    var b = Rigidbodies[j];

                    var aBounds = a.collider.GetBounds();
                    var bBounds = b.collider.GetBounds();

                    if (aBounds.Intersects(bBounds))
                    {
                        var cp = new ContactPair { bodyA = a, bodyB = b };
                        pairs.Add(cp);
                    }
                }
            }

            return pairs;
        }

        public Dictionary<Rigidbody, Vector3S> VelocityBuffer = new Dictionary<Rigidbody, Vector3S>();
        public Dictionary<Rigidbody, Vector3S> PositionBuffer = new Dictionary<Rigidbody, Vector3S>();


        void NaiveNarrowPhase(HashSet<BodyPair> pairs)
        {
            // Ensure buffers are clear
            VelocityBuffer.Clear();
            PositionBuffer.Clear();

            foreach (var pair in pairs)
            {
                var a = Rigidbodies[pair.BodyA];
                var b = Rigidbodies[pair.BodyB];

                if (a.isSleeping && b.isSleeping) continue;

                if (a.collider.Intersects(a.position, b.position, b.collider, out var contact))
                {
                    // Calculate relative velocity
                    Vector3S relativeVelocity = b.velocity - a.velocity;

                    // Calculate relative velocity in terms of the normal direction
                    sfloat velocityAlongNormal = Vector3S.Dot(relativeVelocity, contact.normal);

                    // If velocities are separating, do nothing
                    if (velocityAlongNormal > sfloat.zero)
                        continue;

                    // Calculate restitution (elasticity), reduce for less bounce
                    sfloat e = (sfloat)0.5f;

                    // Calculate impulse scalar
                    sfloat invMassA = (sfloat.one / a.mass);
                    sfloat invMassB = (sfloat.one / b.mass);
                    sfloat j = -(sfloat.one + e) * velocityAlongNormal;
                    j /= invMassA + invMassB;

                    // Apply impulse
                    Vector3S impulse = j * contact.normal;

                    if (!VelocityBuffer.ContainsKey(a))
                        VelocityBuffer[a] = Vector3S.zero;
                    if (!VelocityBuffer.ContainsKey(b))
                        VelocityBuffer[b] = Vector3S.zero;

                    VelocityBuffer[a] -= invMassA * impulse;
                    VelocityBuffer[b] += invMassB * impulse;

                    // Friction impulse
                    Vector3S tangent = (relativeVelocity - (velocityAlongNormal * contact.normal)).Normalize();
                    sfloat jt = -Vector3S.Dot(relativeVelocity, tangent);
                    jt /= invMassA + invMassB;

                    // Coulomb's law of friction
                    sfloat mu = (sfloat)0.5f; // coefficient of friction
                    Vector3S frictionImpulse;
                    if (MathS.Abs(jt) < j * mu)
                    {
                        frictionImpulse = jt * tangent;
                    }
                    else
                    {
                        frictionImpulse = -j * mu * tangent;
                    }

                    VelocityBuffer[a] -= invMassA * frictionImpulse;
                    VelocityBuffer[b] += invMassB * frictionImpulse;

                    // Positional correction to avoid sinking
                    sfloat percent = (sfloat)0.2f; // usually 20% to 80%
                    sfloat slop = (sfloat)0.01f; // usually 0.01 to 0.1

                    Vector3S correction = MathS.Max(contact.penetrationDepth - slop, sfloat.zero) / (invMassA + invMassB) * percent * contact.normal;

                    if (!PositionBuffer.ContainsKey(a))
                        PositionBuffer[a] = Vector3S.zero;
                    if (!PositionBuffer.ContainsKey(b))
                        PositionBuffer[b] = Vector3S.zero;

                    PositionBuffer[a] -= invMassA * correction;
                    PositionBuffer[b] += invMassB * correction;
                }
            }

            // Apply changes to velocities and positions
            foreach (var change in VelocityBuffer)
            {
                change.Key.velocity += change.Value;
            }

            foreach (var change in PositionBuffer)
            {
                change.Key.position += change.Value;
            }
        }


        void Integrate(sfloat deltaTime)
        {
            //The integration must be halved for like each iteration count
            for (int i = 0; i < Rigidbodies.Count; i++)
            {
                var rb = Rigidbodies[i];
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

            if (_multiThread)
            {
                Parallel.For(0, Rigidbodies.Count, i =>
                {
                    Rigidbodies[i].collider.CalculateBounds(Rigidbodies[i].position);
                });
            }
            else
            {
                foreach (var body in Rigidbodies) body.collider.CalculateBounds(body.position);
            }

            var pairs = Broadphase.ComputePairs(this.Rigidbodies);
            NaiveNarrowPhase(pairs);

            WorldCollision();

            if (_multiThread)
            {
                Parallel.For(0, Rigidbodies.Count, i =>
                {
                    CheckSleep(Rigidbodies[i]);
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