using System;
using System.Collections.Generic;
using System.Security.Cryptography;
using System.Text;
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
        private readonly Vector3S[] angularVelocityBuffer;
        private readonly SQuaternion[] rotationBuffer;

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
            angularVelocityBuffer = new Vector3S[startSize * 2];
            rotationBuffer = new SQuaternion[startSize * 2];
        }

        public SRigidbody AddRigidbody()
        {
            var rb = new SRigidbody(counter++);
            Rigidbodies.Add(rb);
            return rb;
        }

        public SRigidbody AddRigidbody(Vector3S position, Vector3S velocity)
        {
            var rb = new SRigidbody(counter++, position, velocity);
            Rigidbodies.Add(rb);
            return rb;
        }

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
                    sb.Append(rb.angularVelocity.x.ToString());
                    sb.Append(rb.angularVelocity.y.ToString());
                    sb.Append(rb.angularVelocity.z.ToString());
                    sb.Append(rb.rotation.x.ToString());
                    sb.Append(rb.rotation.y.ToString());
                    sb.Append(rb.rotation.z.ToString());
                    sb.Append(rb.rotation.w.ToString());
                }

                var hashBytes = sha256.ComputeHash(Encoding.UTF8.GetBytes(sb.ToString()));
                return BitConverter.ToString(hashBytes).Replace("-", "").ToLower();
            }
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
                if (rb.isKinematic) continue;

                if (rb.useGravity)
                {
                    rb.velocity += Gravity * deltaTime;
                }

                rb.position += rb.velocity * deltaTime;

                SQuaternion deltaRotation = SQuaternion.FromAngularVelocity(rb.angularVelocity * deltaTime);
                rb.rotation = (deltaRotation * rb.rotation).Normalize();
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
                a.angularVelocity += angularVelocityBuffer[a.index];
                a.rotation = (rotationBuffer[a.index] * a.rotation).Normalize();

                b.velocity += velocityBuffer[b.index];
                b.position += positionBuffer[b.index];
                b.angularVelocity += angularVelocityBuffer[b.index];
                b.rotation = (rotationBuffer[b.index] * b.rotation).Normalize();

                velocityBuffer[a.index] = Vector3S.zero;
                positionBuffer[a.index] = Vector3S.zero;
                angularVelocityBuffer[a.index] = Vector3S.zero;
                rotationBuffer[a.index] = SQuaternion.Identity;

                velocityBuffer[b.index] = Vector3S.zero;
                positionBuffer[b.index] = Vector3S.zero;
                angularVelocityBuffer[b.index] = Vector3S.zero;
                rotationBuffer[b.index] = SQuaternion.Identity;
            }
        }

        private void ResolveCollision(SRigidbody a, SRigidbody b, Contact contact)
        {
            Vector3S relativeVelocity = b.velocity - a.velocity;
            sfloat velocityAlongNormal = Vector3S.Dot(relativeVelocity, contact.normal);

            //if (velocityAlongNormal > sfloat.zero) return;

            sfloat e = (sfloat)0.5f;
            sfloat invMassA = a.isKinematic ? sfloat.zero : (sfloat.one / a.mass);
            sfloat invMassB = b.isKinematic ? sfloat.zero : (sfloat.one / b.mass);

            // Linear impulse
            sfloat j = -(sfloat.one + e) * velocityAlongNormal / (invMassA + invMassB);
            Vector3S impulse = j * contact.normal;
            ApplyImpulse(a, b, impulse, invMassA, invMassB);

            // Angular impulse
            Vector3S rA = contact.point - a.position;
            Vector3S rB = contact.point - b.position;

            Vector3S torqueA = Vector3S.Cross(rA, impulse);
            Vector3S torqueB = Vector3S.Cross(rB, impulse);

            sfloat invInertiaA = a.isKinematic ? sfloat.zero : (sfloat.one / a.inertia);
            sfloat invInertiaB = b.isKinematic ? sfloat.zero : (sfloat.one / b.inertia);

            ApplyAngularImpulse(a, b, torqueA, torqueB, invInertiaA, invInertiaB);

            // Friction impulse
            Vector3S tangent = (relativeVelocity - (velocityAlongNormal * contact.normal)).Normalize();
            sfloat jt = -Vector3S.Dot(relativeVelocity, tangent) / (invMassA + invMassB);
            sfloat mu = (sfloat)0.5f;
            Vector3S frictionImpulse = MathS.Abs(jt) < j * mu ? jt * tangent : -j * mu * tangent;
            ApplyImpulse(a, b, frictionImpulse, invMassA, invMassB);

            torqueA = Vector3S.Cross(rA, frictionImpulse);
            torqueB = Vector3S.Cross(rB, frictionImpulse);
            ApplyAngularImpulse(a, b, torqueA, torqueB, invInertiaA, invInertiaB);

            CorrectPositions(a, b, contact, invMassA, invMassB);
        }

        private void ApplyImpulse(SRigidbody a, SRigidbody b, Vector3S impulse, sfloat invMassA, sfloat invMassB)
        {
            if (!a.isKinematic)
                velocityBuffer[a.index] -= invMassA * impulse;

            if (!b.isKinematic)
                velocityBuffer[b.index] += invMassB * impulse;
        }

        private void ApplyAngularImpulse(SRigidbody a, SRigidbody b, Vector3S torqueA, Vector3S torqueB, sfloat invInertiaA, sfloat invInertiaB)
        {
            if (!a.isKinematic)
                angularVelocityBuffer[a.index] += invInertiaA * torqueA;

            if (!b.isKinematic)
                angularVelocityBuffer[b.index] -= invInertiaB * torqueB;
        }

        private void CorrectPositions(SRigidbody a, SRigidbody b, Contact contact, sfloat invMassA, sfloat invMassB)
        {
            sfloat percent = (sfloat)0.2f;
            sfloat slop = (sfloat)0.01f;
            Vector3S correction = MathS.Max(contact.penetrationDepth - slop, sfloat.zero) / (invMassA + invMassB) * percent * contact.normal;

            if (!a.isKinematic)
                positionBuffer[a.index] -= invMassA * correction;

            if (!b.isKinematic)
                positionBuffer[b.index] += invMassB * correction;
        }

        private void CheckSleep(SRigidbody body)
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
