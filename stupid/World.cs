﻿using System;
using System.Collections.Generic;
using SoftFloat;
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
            DumbGrid = new DumbGrid<int>(gridDim, gridDim, gridDim, (sfloat)gridSize);
        }

        public SRigidbody AddRigidbody(Vector3S position = default, Vector3S velocity = default, Vector3S angularVelocity = default)
        {
            var rb = new SRigidbody(counter++, position, velocity, angularVelocity);
            Rigidbodies.Add(rb);
            return rb;
        }

        public void Simulate(sfloat deltaTime)
        {
            Integrate(deltaTime);

            DumbGrid.Invalidate(-1);

            foreach (var body in Rigidbodies)
            {
                var bounds = body.collider.CalculateBounds(body.position);
                DumbGrid.Add(bounds, body.index);
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

                // Angular integration
                if (rb.angularVelocity.Magnitude() > sfloat.Epsilon)
                {
                    Vector3S angularVelocityDelta = rb.angularVelocity * deltaTime;
                    SQuaternion deltaRotation = SQuaternion.FromAxisAngle(angularVelocityDelta.Normalize(), angularVelocityDelta.Magnitude());
                    rb.rotation = (deltaRotation * rb.rotation).Normalize();
                    rb.angularVelocity *= (sfloat)0.99f;
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

            ApplyBuffers(pairs);
        }

        private void ResolveCollision(SRigidbody a, SRigidbody b, Contact contact)
        {
            Vector3S ra = contact.point - a.position;
            Vector3S rb = contact.point - b.position;

            Vector3S raCrossN = Vector3S.Cross(ra, contact.normal);
            Vector3S rbCrossN = Vector3S.Cross(rb, contact.normal);

            Vector3S angularVelocityA = a.angularVelocity;
            Vector3S angularVelocityB = b.angularVelocity;

            Vector3S velocityAtPointA = a.velocity + Vector3S.Cross(angularVelocityA, ra);
            Vector3S velocityAtPointB = b.velocity + Vector3S.Cross(angularVelocityB, rb);

            Vector3S relativeVelocity = velocityAtPointB - velocityAtPointA;
            sfloat velocityAlongNormal = Vector3S.Dot(relativeVelocity, contact.normal);

            if (velocityAlongNormal > sfloat.zero) return;

            sfloat e = (sfloat)0.5f;
            sfloat invMassA = sfloat.one / a.mass;
            sfloat invMassB = sfloat.one / b.mass;

            Matrix3S inverseInertiaTensorA = a.GetInverseInertiaTensorWorld();
            Matrix3S inverseInertiaTensorB = b.GetInverseInertiaTensorWorld();

            sfloat raCrossNDot = Vector3S.Dot(raCrossN, inverseInertiaTensorA * raCrossN);
            sfloat rbCrossNDot = Vector3S.Dot(rbCrossN, inverseInertiaTensorB * rbCrossN);

            sfloat denom = invMassA + invMassB + raCrossNDot + rbCrossNDot;

            sfloat j = -(sfloat.one + e) * velocityAlongNormal / denom;

            Vector3S impulse = j * contact.normal;

            // Buffer the linear velocity changes
            velocityBuffer[a.index] -= invMassA * impulse;
            velocityBuffer[b.index] += invMassB * impulse;

            // Buffer the angular velocity changes
            angularVelocityBuffer[a.index] -= inverseInertiaTensorA * Vector3S.Cross(ra, impulse);
            angularVelocityBuffer[b.index] += inverseInertiaTensorB * Vector3S.Cross(rb, impulse);

            // Friction impulse
            Vector3S tangent = (relativeVelocity - (velocityAlongNormal * contact.normal)).Normalize();
            sfloat jt = -Vector3S.Dot(relativeVelocity, tangent) / denom;

            sfloat mu = (sfloat)0.5f;
            Vector3S frictionImpulse = MathS.Abs(jt) < j * mu ? jt * tangent : -j * mu * tangent;

            // Buffer the friction impulse changes
            velocityBuffer[a.index] -= invMassA * frictionImpulse;
            velocityBuffer[b.index] += invMassB * frictionImpulse;

            angularVelocityBuffer[a.index] -= inverseInertiaTensorA * Vector3S.Cross(ra, frictionImpulse);
            angularVelocityBuffer[b.index] += inverseInertiaTensorB * Vector3S.Cross(rb, frictionImpulse);
        }

        private void ApplyBuffers(HashSet<BodyPair> pairs)
        {
            foreach (var pair in pairs)
            {
                var a = Rigidbodies[pair.aIndex];
                var b = Rigidbodies[pair.bIndex];

                a.velocity += velocityBuffer[a.index];
                a.angularVelocity += angularVelocityBuffer[a.index];

                b.velocity += velocityBuffer[b.index];
                b.angularVelocity += angularVelocityBuffer[b.index];

                velocityBuffer[a.index] = Vector3S.zero;
                angularVelocityBuffer[a.index] = Vector3S.zero;

                velocityBuffer[b.index] = Vector3S.zero;
                angularVelocityBuffer[b.index] = Vector3S.zero;
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
