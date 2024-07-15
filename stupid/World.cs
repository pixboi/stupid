using System.Collections.Generic;
using stupid.Colliders;
using stupid.Maths;
using System;

namespace stupid
{
    public class World
    {
        public WorldSettings Settings { get; private set; }
        public SortAndSweepBroadphase Broadphase { get; set; }
        public DumbList<Collidable> Collidables { get; private set; }
        public uint SimulationFrame { get; private set; }

        private int counter;

        public World(WorldSettings worldSettings, int startSize = 1000)
        {
            this.Settings = worldSettings;

            Collidables = new DumbList<Collidable>(startSize);
            Broadphase = new SortAndSweepBroadphase(startSize);

            counter = 0;
            SimulationFrame = 0;
        }

        public Collidable AddCollidable(Collidable c)
        {
            c.Register(counter++);
            Collidables.Add(c);
            return c;
        }

        public static f32 DeltaTime;
        public void Simulate(f32 deltaTime)
        {
            DeltaTime = deltaTime;
            foreach (var c in Collidables) if (c is RigidbodyS rb) rb.Integrate(deltaTime, Settings);

            // Recalculate positions and bounds
            foreach (var c in Collidables)
            {
                if (c.isDynamic)
                {
                    if (c.collider.NeedsRotationUpdate)
                    {
                        c.transform.UpdateRotationMatrix();
                        c.collider.OnRotationUpdate();
                    }

                    c.CalculateBounds();
                }

                if (c is RigidbodyS rb)
                {
                    if (rb.angularVelocity.SqrMagnitude > f32.epsilon) rb.tensor.CalculateInverseInertiaTensor(rb.transform.rotation);
                }
            }

            var pairs = Broadphase.ComputePairs(Collidables);
            NarrowPhase(pairs);

            SimulationFrame++;
        }

        public event Action<ContactManifoldS> OnContact;
        public ContactS[] _contactCache = new ContactS[8];
        public Dictionary<BodyPair, ContactManifoldS> _manifolds = new Dictionary<BodyPair, ContactManifoldS>();

        void UpdateManifold(BodyPair pair)
        {
            var a = Collidables[pair.aIndex];
            var b = Collidables[pair.bIndex];

            // Ensure the dynamic body is 'a' for consistent processing
            if (b.isDynamic && !a.isDynamic)
            {
                var temp = a;
                a = b;
                b = temp;
            }

            var contactCount = a.collider.Intersects(b, ref _contactCache);

            if (contactCount > 0)
            {
                var freshManifold = new ContactManifoldS(a, b, _contactCache, contactCount);

                if (!_manifolds.TryGetValue(pair, out var oldManifold))
                {
                    // On ENTER: Add a new manifold
                    _manifolds[pair] = freshManifold;
                }
                else
                {
                    // On STAY: Update the manifold while preserving warm start data
                    freshManifold = new ContactManifoldS(freshManifold, oldManifold);
                    _manifolds[pair] = freshManifold;
                }

                // Trigger the contact event
                OnContact?.Invoke(freshManifold);
            }
            else
            {
                if (_manifolds.ContainsKey(pair))
                {
                    // On EXIT: Remove the manifold
                    _manifolds.Remove(pair);
                }
            }
        }

        List<BodyPair> _removeCache = new List<BodyPair>();
        private void NarrowPhase(HashSet<BodyPair> pairs)
        {
            // Collect keys that were not touched by the broadphase
            _removeCache.Clear();
            foreach (var key in _manifolds.Keys)
            {
                if (!pairs.Contains(key))
                {
                    _removeCache.Add(key);
                }
            }

            // Remove the old keys
            foreach (var key in _removeCache)
            {
                _manifolds.Remove(key);
            }

            // Update the manifolds for the current pairs
            foreach (var pair in pairs)
            {
                UpdateManifold(pair);
            }

            // Solve collisions
            for (int i = 0; i < Settings.DefaultSolverIterations; i++)
            {
                foreach (var kvp in _manifolds)
                {
                    var m = kvp.Value;

                    if (m.a.isDynamic && m.b.isDynamic)
                    {
                        ResolveCollision(m);
                    }
                    else
                    {
                        ResolveCollisionStatic(m);
                    }
                }
            }
        }

        private f32 CORRECTION => (f32)1 / (f32)Settings.DefaultSolverIterations;

        private void ResolveCollisionStatic(ContactManifoldS manifold)
        {
            // Assume always that 'a' is the dynamic body
            var body = (RigidbodyS)manifold.a;
            var stat = manifold.b;

            // Constants
            f32 slop = Settings.DefaultContactOffset;

            for (int i = 0; i < manifold.count; i++)
            {
                var contact = manifold.contacts[i];

                // Ensure the normal always points from the static object to the dynamic body
                Vector3S normal = contact.normal;

                // Calculate relative position from the center of mass to the contact point
                Vector3S rb = contact.point - body.transform.position;

                // Calculate the relative velocity at the contact point
                Vector3S relativeVelocityAtContact = body.velocity + Vector3S.Cross(body.angularVelocity, rb);

                // Calculate inverse mass
                f32 invMass = body.mass > f32.zero ? f32.one / body.mass : f32.zero;

                // Compute the effective mass along the normal direction
                f32 effectiveMass = invMass + Vector3S.Dot(Vector3S.Cross(body.tensor.inertiaWorld * Vector3S.Cross(rb, normal), rb), normal);

                // Positional correction to prevent sinking
                f32 penetrationDepth = MathS.Max(contact.penetrationDepth - slop, f32.zero);
                Vector3S correction = (penetrationDepth / effectiveMass) * normal * CORRECTION;
                body.transform.position += invMass * correction;

                // Calculate the velocity along the normal
                f32 velocityAlongNormal = Vector3S.Dot(relativeVelocityAtContact, normal);

                // Do not resolve if velocities are separating
                if (velocityAlongNormal > f32.zero) continue;

                // Restitution (coefficient of restitution)
                f32 restitution = relativeVelocityAtContact.Magnitude() >= Settings.BounceThreshold
                    ? (body.material.restitution + stat.material.restitution) * f32.half
                    : f32.zero;

                // Calculate the normal impulse scalar
                f32 incrementalImpulse = -(f32.one + restitution) * velocityAlongNormal / effectiveMass;
                f32 newAccumulatedImpulse = MathS.Max(contact.cachedNormalImpulse + incrementalImpulse, f32.zero);
                f32 appliedImpulse = newAccumulatedImpulse - contact.cachedNormalImpulse;
                contact.cachedNormalImpulse = newAccumulatedImpulse;

                // Apply linear impulse
                Vector3S normalImpulse = appliedImpulse * normal;
                body.velocity += invMass * normalImpulse;
                body.angularVelocity += body.tensor.inertiaWorld * Vector3S.Cross(rb, normalImpulse);

                // Calculate relative tangential velocity
                Vector3S relativeTangentialVelocity = relativeVelocityAtContact - (velocityAlongNormal * normal);
                Vector3S tangent = relativeTangentialVelocity.Magnitude() > f32.zero ? relativeTangentialVelocity.Normalize() : Vector3S.zero;

                // Calculate the magnitude of the friction impulse
                f32 frictionDenominator = invMass + Vector3S.Dot(Vector3S.Cross(body.tensor.inertiaWorld * Vector3S.Cross(rb, tangent), rb), tangent);
                f32 frictionImpulseScalar = -Vector3S.Dot(relativeTangentialVelocity, tangent) / frictionDenominator;

                // Effective friction
                f32 effectiveFriction = (body.material.staticFriction + stat.material.staticFriction) * f32.half;

                // Limit the friction impulse to prevent excessive angular velocities
                Vector3S frictionImpulse = frictionImpulseScalar * tangent;
                if (frictionImpulse.Magnitude() > appliedImpulse * effectiveFriction)
                {
                    frictionImpulse = frictionImpulse.Normalize() * (appliedImpulse * effectiveFriction);
                }

                // Apply friction impulse
                body.velocity += invMass * frictionImpulse;
                body.angularVelocity += body.tensor.inertiaWorld * Vector3S.Cross(rb, frictionImpulse);

                // Store the accumulated impulses
                contact.cachedImpulse = normalImpulse + frictionImpulse;
                contact.cachedNormalImpulse = newAccumulatedImpulse;
                contact.cachedFrictionImpulse = frictionImpulseScalar;

                // Update the contact in the manifold
                manifold.contacts[i] = contact;
            }
        }

        private void ResolveCollision(ContactManifoldS manifold)
        {
            var a = (RigidbodyS)manifold.a;
            var b = (RigidbodyS)manifold.b;

            // Constants
            f32 slop = Settings.DefaultContactOffset;

            for (int i = 0; i < manifold.count; i++)
            {
                var contact = manifold.contacts[i];

                // Ensure the normal always points from b to a
                Vector3S normal = contact.normal;

                // Calculate relative positions from the centers of mass to the contact point
                Vector3S ra = contact.point - a.transform.position;
                Vector3S rb = contact.point - b.transform.position;

                Vector3S va = a.velocity + Vector3S.Cross(a.angularVelocity, ra);
                Vector3S vb = b.velocity + Vector3S.Cross(b.angularVelocity, rb);

                // Calculate relative velocity at the contact point
                Vector3S relativeVelocityAtContact = va - vb;

                // Compute the effective mass along the normal direction
                f32 effectiveMassA = a.inverseMass + Vector3S.Dot(Vector3S.Cross(a.tensor.inertiaWorld * Vector3S.Cross(ra, normal), ra), normal);
                f32 effectiveMassB = b.inverseMass + Vector3S.Dot(Vector3S.Cross(b.tensor.inertiaWorld * Vector3S.Cross(rb, normal), rb), normal);
                f32 effectiveMassSum = effectiveMassA + effectiveMassB;

                // Positional correction to prevent sinking
                f32 penetrationDepth = MathS.Max(contact.penetrationDepth - slop, f32.zero);
                Vector3S correction = (penetrationDepth / effectiveMassSum) * normal * CORRECTION;
                Vector3S ca = a.inverseMass * correction;
                Vector3S cb = b.inverseMass * correction;
                a.transform.position += ca;
                b.transform.position -= cb;

                // Calculate the velocity along the normal
                f32 velocityAlongNormal = Vector3S.Dot(relativeVelocityAtContact, normal);

                // Do not resolve if velocities are separating
                if (velocityAlongNormal > f32.zero) continue;

                // Restitution (coefficient of restitution)
                f32 restitution = relativeVelocityAtContact.Magnitude() >= Settings.BounceThreshold
                    ? (a.material.restitution + b.material.restitution) * f32.half
                    : f32.zero;

                // Calculate the normal impulse scalar
                f32 incrementalImpulse = -(f32.one + restitution) * velocityAlongNormal / effectiveMassSum;
                f32 newAccumulatedImpulse = MathS.Max(contact.cachedNormalImpulse + incrementalImpulse, f32.zero);
                f32 appliedImpulse = newAccumulatedImpulse - contact.cachedNormalImpulse;
                contact.cachedNormalImpulse = newAccumulatedImpulse;

                // Apply linear impulse
                Vector3S impulse = appliedImpulse * normal;
                a.velocity += a.inverseMass * impulse;
                b.velocity -= b.inverseMass * impulse;

                a.angularVelocity += a.tensor.inertiaWorld * Vector3S.Cross(ra, impulse);
                b.angularVelocity -= b.tensor.inertiaWorld * Vector3S.Cross(rb, impulse);

                // Calculate relative tangential velocity
                Vector3S relativeTangentialVelocity = relativeVelocityAtContact - (velocityAlongNormal * normal);
                Vector3S tangent = relativeTangentialVelocity.Magnitude() > f32.zero ? relativeTangentialVelocity.Normalize() : Vector3S.zero;

                // Calculate the magnitude of the friction impulse
                f32 frictionDenominatorA = a.inverseMass + Vector3S.Dot(Vector3S.Cross(a.tensor.inertiaWorld * Vector3S.Cross(ra, tangent), ra), tangent);
                f32 frictionDenominatorB = b.inverseMass + Vector3S.Dot(Vector3S.Cross(b.tensor.inertiaWorld * Vector3S.Cross(rb, tangent), rb), tangent);
                f32 frictionDenominator = frictionDenominatorA + frictionDenominatorB;

                f32 frictionImpulseScalar = -Vector3S.Dot(relativeTangentialVelocity, tangent) / frictionDenominator;

                // Just use static friction average
                f32 effectiveFriction = (a.material.staticFriction + b.material.staticFriction) * f32.half;

                // Limit the friction impulse to prevent excessive angular velocities
                Vector3S frictionImpulse = frictionImpulseScalar * tangent;
                if (frictionImpulse.Magnitude() > appliedImpulse * effectiveFriction)
                {
                    frictionImpulse = frictionImpulse.Normalize() * (appliedImpulse * effectiveFriction);
                }

                a.velocity += a.inverseMass * frictionImpulse;
                b.velocity -= b.inverseMass * frictionImpulse;

                a.angularVelocity += a.tensor.inertiaWorld * Vector3S.Cross(ra, frictionImpulse);
                b.angularVelocity -= b.tensor.inertiaWorld * Vector3S.Cross(rb, frictionImpulse);

                // Store the accumulated impulses
                contact.cachedImpulse = impulse + frictionImpulse;
                contact.cachedNormalImpulse = newAccumulatedImpulse;
                contact.cachedFrictionImpulse = frictionImpulseScalar;

                // Update the contact in the manifold
                manifold.contacts[i] = contact;
            }
        }
    }
}
