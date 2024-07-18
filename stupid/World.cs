using System;
using System.Collections.Generic;
using System.Linq;
using stupid.Maths;

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
            Settings = worldSettings;
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
            IntegrateBodies(deltaTime);
            RecalculatePositionsAndBounds();
            var pairs = Broadphase.ComputePairs(Collidables);
            NarrowPhase(pairs);
            SimulationFrame++;
        }

        private void IntegrateBodies(f32 deltaTime)
        {
            foreach (var c in Collidables)
            {
                if (c is RigidbodyS rb) rb.Integrate(deltaTime, Settings);
            }
        }

        private void RecalculatePositionsAndBounds()
        {
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

                if (c is RigidbodyS rb && rb.angularVelocity.SqrMagnitude > f32.epsilon)
                {
                    rb.tensor.CalculateInverseInertiaTensor(rb.transform.rotation);
                }
            }
        }

        public event Action<ContactManifoldS> OnContact;
        private ContactS[] _contactCache = new ContactS[8];
        private Dictionary<BodyPair, ContactManifoldS> _manifolds = new Dictionary<BodyPair, ContactManifoldS>();

        private void UpdateManifold(BodyPair pair)
        {
            var a = Collidables[pair.aIndex];
            var b = Collidables[pair.bIndex];

            // Ensure the dynamic body is 'a' for consistent processing
            if (b.isDynamic && !a.isDynamic)
            {
                (a, b) = (b, a);
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
            else if (_manifolds.ContainsKey(pair))
            {
                // On EXIT: Remove the manifold
                _manifolds.Remove(pair);
            }
        }

        private List<BodyPair> _removeCache = new List<BodyPair>();

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

            var sortedManifolds = _manifolds.Values.OrderByDescending(x => x.contacts[0].penetrationDepth);

            // Solve collisions
            for (int i = 0; i < Settings.DefaultSolverIterations; i++)
            {
                foreach (var manifold in sortedManifolds)
                {
                    ResolveCollision(manifold, i);
                }
            }
        }

        private void ResolveCollision(ContactManifoldS manifold, int iteration)
        {
            if (manifold.a.isDynamic && manifold.b.isDynamic)
            {
                ResolveDynamicCollision(manifold);
            }
            else
            {
                ResolveStaticCollision(manifold);
            }
        }


        private void ResolveDynamicCollision(ContactManifoldS manifold)
        {
            var a = (RigidbodyS)manifold.a;
            var b = (RigidbodyS)manifold.b;

            for (int i = 0; i < manifold.count; i++)
            {
                ResolveContact(a, b, manifold.contacts[i]);
            }
        }

        private void ResolveStaticCollision(ContactManifoldS manifold)
        {
            var body = (RigidbodyS)manifold.a;
            var stat = manifold.b;

            for (int i = 0; i < manifold.count; i++)
            {
                ResolveContact(body, stat, manifold.contacts[i], isStatic: true);
            }
        }

        private void ResolveContact(RigidbodyS a, Collidable b, ContactS contact, bool isStatic = false)
        {
            Vector3S normal = contact.normal;
            Vector3S ra = contact.point - a.transform.position;
            RigidbodyS? rbBody = isStatic ? null : b as RigidbodyS;
            Vector3S rb = isStatic ? Vector3S.zero : contact.point - rbBody.transform.position;

            Vector3S relativeVelocityAtContact = a.velocity + Vector3S.Cross(a.angularVelocity, ra) -
                (isStatic ? Vector3S.zero : rbBody.velocity + Vector3S.Cross(rbBody.angularVelocity, rb));

            f32 invMassA = a.inverseMass;
            f32 invMassB = isStatic ? f32.zero : rbBody.inverseMass;
            f32 effectiveMass = invMassA + Vector3S.Dot(Vector3S.Cross(a.tensor.inertiaWorld * Vector3S.Cross(ra, normal), ra), normal) +
                (isStatic ? f32.zero : invMassB + Vector3S.Dot(Vector3S.Cross(rbBody.tensor.inertiaWorld * Vector3S.Cross(rb, normal), rb), normal));

            f32 slop = Settings.DefaultContactOffset;
            f32 baumgarteFactor = f32.one / (f32)Settings.DefaultSolverIterations;
            f32 penetrationDepth = MathS.Max(contact.penetrationDepth - slop, f32.zero);
            Vector3S correction = (penetrationDepth / effectiveMass) * normal * baumgarteFactor;
            a.transform.position += invMassA * correction;
            if (!isStatic)
            {
                rbBody.transform.position -= invMassB * correction;
            }

            f32 velocityAlongNormal = Vector3S.Dot(relativeVelocityAtContact, normal);
            if (velocityAlongNormal > f32.zero) return;

            f32 restitution = relativeVelocityAtContact.Magnitude() >= Settings.BounceThreshold
                ? MathS.Min(a.material.restitution, b.material.restitution)
                : f32.zero;

            f32 incrementalImpulse = -(f32.one + restitution) * velocityAlongNormal / effectiveMass;
            f32 newAccumulatedImpulse = MathS.Max(contact.cachedNormalImpulse + incrementalImpulse, f32.zero);
            f32 appliedImpulse = newAccumulatedImpulse - contact.cachedNormalImpulse;
            contact.cachedNormalImpulse = newAccumulatedImpulse;

            Vector3S normalImpulse = appliedImpulse * normal;
            a.velocity += invMassA * normalImpulse;
            a.angularVelocity += a.tensor.inertiaWorld * Vector3S.Cross(ra, normalImpulse);
            if (!isStatic)
            {
                rbBody.velocity -= invMassB * normalImpulse;
                rbBody.angularVelocity -= rbBody.tensor.inertiaWorld * Vector3S.Cross(rb, normalImpulse);
            }

            Vector3S relativeTangentialVelocity = relativeVelocityAtContact - (velocityAlongNormal * normal);
            Vector3S tangent = relativeTangentialVelocity.Magnitude() > f32.zero ? relativeTangentialVelocity.Normalize() : Vector3S.zero;
            f32 frictionDenominator = invMassA + Vector3S.Dot(Vector3S.Cross(a.tensor.inertiaWorld * Vector3S.Cross(ra, tangent), ra), tangent) +
                (isStatic ? f32.zero : invMassB + Vector3S.Dot(Vector3S.Cross(rbBody.tensor.inertiaWorld * Vector3S.Cross(rb, tangent), rb), tangent));
            f32 frictionImpulseScalar = -Vector3S.Dot(relativeTangentialVelocity, tangent) / frictionDenominator;
            f32 effectiveFriction = (a.material.staticFriction + b.material.staticFriction) * f32.half;

            Vector3S frictionImpulse = frictionImpulseScalar * tangent;
            if (frictionImpulse.Magnitude() > appliedImpulse * effectiveFriction)
            {
                frictionImpulse = frictionImpulse.Normalize() * (appliedImpulse * effectiveFriction);
            }

            a.velocity += invMassA * frictionImpulse;
            a.angularVelocity += a.tensor.inertiaWorld * Vector3S.Cross(ra, frictionImpulse);
            if (!isStatic)
            {
                rbBody.velocity -= invMassB * frictionImpulse;
                rbBody.angularVelocity -= rbBody.tensor.inertiaWorld * Vector3S.Cross(rb, frictionImpulse);
            }

            contact.cachedImpulse = normalImpulse + frictionImpulse;
            contact.cachedNormalImpulse = newAccumulatedImpulse;
            contact.cachedFrictionImpulse = frictionImpulseScalar;
        }


    }
}
