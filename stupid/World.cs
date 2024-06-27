using System;
using System.Collections.Generic;
using System.Runtime.CompilerServices;
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

        public World(Bounds worldBounds, IBroadphase broadphase)
        {
            counter = 0;
            this.worldBounds = worldBounds;
            Rigidbodies = new List<Rigidbody>(128);

            Broadphase = broadphase;
        }

        void AddGravity(sfloat deltaTime)
        {
            foreach (var rb in Rigidbodies)
            {
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
                var bounds = rb.collider.GetBounds(rb.position);
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

                    var aBounds = a.collider.GetBounds(a.position);
                    var bBounds = b.collider.GetBounds(b.position);

                    if (aBounds.Intersects(bBounds))
                    {
                        var cp = new ContactPair { bodyA = a, bodyB = b };
                        pairs.Add(cp);
                    }
                }
            }

            return pairs;
        }

        void NaiveNarrowPhase(List<ContactPair> pairs)
        {
            var velocityChanges = new Dictionary<Rigidbody, Vector3S>();
            var positionChanges = new Dictionary<Rigidbody, Vector3S>();

            foreach (var pair in pairs)
            {
                var a = pair.bodyA;
                var b = pair.bodyB;

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
                    sfloat j = -(sfloat.one + e) * velocityAlongNormal;
                    j /= (sfloat.one / a.mass) + (sfloat.one / b.mass);

                    // Apply impulse
                    Vector3S impulse = j * contact.normal;

                    if (!velocityChanges.ContainsKey(a))
                        velocityChanges[a] = Vector3S.zero;
                    if (!velocityChanges.ContainsKey(b))
                        velocityChanges[b] = Vector3S.zero;

                    velocityChanges[a] -= (sfloat.one / a.mass) * impulse;
                    velocityChanges[b] += (sfloat.one / b.mass) * impulse;

                    // Friction impulse
                    Vector3S tangent = (relativeVelocity - (velocityAlongNormal * contact.normal)).Normalize();
                    sfloat jt = -Vector3S.Dot(relativeVelocity, tangent);
                    jt /= (sfloat.one / a.mass) + (sfloat.one / b.mass);

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

                    velocityChanges[a] -= (sfloat.one / a.mass) * frictionImpulse;
                    velocityChanges[b] += (sfloat.one / b.mass) * frictionImpulse;

                    // Positional correction to avoid sinking
                    sfloat percent = (sfloat)0.2f; // usually 20% to 80%
                    sfloat slop = (sfloat)0.01f; // usually 0.01 to 0.1

                    Vector3S correction = MathS.Max(contact.penetrationDepth - slop, sfloat.zero) / ((sfloat.one / a.mass) + (sfloat.one / b.mass)) * percent * contact.normal;

                    if (!positionChanges.ContainsKey(a))
                        positionChanges[a] = Vector3S.zero;
                    if (!positionChanges.ContainsKey(b))
                        positionChanges[b] = Vector3S.zero;

                    positionChanges[a] -= (sfloat.one / a.mass) * correction;
                    positionChanges[b] += (sfloat.one / b.mass) * correction;
                }
            }

            // Apply changes to velocities
            foreach (var change in velocityChanges)
            {
                change.Key.velocity += change.Value;
            }

            // Apply changes to positions
            foreach (var change in positionChanges)
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
                rb.position += rb.velocity * deltaTime;
            }
        }

        public void Simulate(sfloat deltaTime)
        {
            AddGravity(deltaTime);
            Integrate(deltaTime);

            var pairs = Broadphase.ComputePairs(this.Rigidbodies);
            NaiveNarrowPhase(pairs);

            WorldCollision();
        }
    }
}