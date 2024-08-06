using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.CompilerServices;
using stupid.Colliders;
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
                if (c.collider.NeedsRotationUpdate)
                {
                    c.transform.UpdateRotationMatrix();
                    c.collider.OnRotationUpdate();
                }

                c.CalculateBounds();

                if (c is RigidbodyS rb)
                {
                    rb.tensor.CalculateInverseInertiaTensor(rb.transform.rotation);
                }
            }
        }

        public event Action<ContactS> OnContact;
        private Dictionary<IntPair, ContactS> _contacts = new Dictionary<IntPair, ContactS>();

        private void UpdateManifold(IntPair pair)
        {
            var a = Collidables[pair.aIndex];
            var b = Collidables[pair.bIndex];

            // Ensure the dynamic body is 'a' for consistent processing
            if (b.isDynamic && !a.isDynamic)
            {
                (a, b) = (b, a);
            }

            var contact = new ContactS();
            contact.a = a;
            contact.b = b;

            var contactCount = a.collider.Intersects(b, ref contact);

            if (contactCount > 0)
            {
                contact.CalculateRelativePoints();

                if (_contacts.TryGetValue(pair, out var oldManifold))
                {
                    // On STAY: Update the manifold while preserving warm start data
                    // Copy old manifold's cached impulses to the new manifold
                    contact.cachedNormalImpulse = oldManifold.cachedNormalImpulse;
                    contact.cachedFrictionImpulse = oldManifold.cachedFrictionImpulse;
                    contact.cachedImpulse = oldManifold.cachedImpulse;
                }

                // On ENTER: Add a new manifold
                _contacts[pair] = contact;

                // Trigger the contact event
                OnContact?.Invoke(contact);
            }
            else if (_contacts.ContainsKey(pair))
            {
                // On EXIT: Remove the manifold
                _contacts.Remove(pair);
            }
        }

        List<IntPair> _removeCache = new List<IntPair>();
        private void NarrowPhase(HashSet<IntPair> pairs)
        {
            // Collect keys that were not touched by the broadphase
            _removeCache.Clear();
            foreach (var key in _contacts.Keys) if (!pairs.Contains(key)) _removeCache.Add(key);

            // Remove the old keys
            foreach (var key in _removeCache) _contacts.Remove(key);

            // Update the manifolds for the current pairs
            foreach (var pair in pairs) UpdateManifold(pair);

            pairs.RemoveWhere(x => !_contacts.ContainsKey(x));

            // Solve collisions
            for (int i = 0; i < Settings.DefaultSolverIterations; i++)
            {
                foreach (var pair in pairs)
                {
                    var contact = _contacts[pair];
                    ResolveCollision(ref contact);
                    _contacts[pair] = contact;
                }
            }
        }

        private void ResolveCollision(ref ContactS contact)
        {
            if (contact.a.isDynamic && contact.b.isDynamic)
            {
                ResolveDynamicCollision(ref contact);
            }
            else
            {
                ResolveStaticCollision(ref contact);
            }
        }

        private void ResolveDynamicCollision(ref ContactS contact)
        {
            var a = (RigidbodyS)contact.a;
            var b = (RigidbodyS)contact.b;
            ResolveContact(a, b, ref contact);
        }

        private void ResolveStaticCollision(ref ContactS contact)
        {
            var body = (RigidbodyS)contact.a;
            var stat = contact.b;
            ResolveContact(body, stat, ref contact, isStatic: true);
        }

        // Constants
        private static readonly f32 BAUM = f32.FromFloat(0.2f);

        // Property to get the default contact offset setting
        public f32 SLOP => Settings.DefaultContactOffset;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private void ResolveContact(RigidbodyS a, Collidable bStat, ref ContactS contact, bool isStatic = false)
        {
            // Determine the other rigidbody involved in the contact
            RigidbodyS? b = isStatic ? null : bStat as RigidbodyS;

            // Positions
            Vector3S aPosition = a.transform.position;
            Vector3S bPosition = bStat.transform.position;

            // Contact points
            Vector3S contactPointA = aPosition + contact.pA;
            Vector3S contactPointB = bPosition + contact.pB;
            Vector3S normal = contact.normal;

            // Calculate penetration depth
            f32 penetrationDepth = Vector3S.Dot(contactPointB - contactPointA, normal) + contact.penetrationDepth;
            penetrationDepth = MathS.Max(penetrationDepth - SLOP, f32.zero);
            if (penetrationDepth == f32.zero) return;

            // Relative positions from centers of mass
            Vector3S ra = contactPointA - aPosition;
            Vector3S rb = isStatic ? Vector3S.zero : contactPointB - b.transform.position;

            // Relative velocity at the contact point
            Vector3S relativeVelocityAtContact = a.velocity + Vector3S.Cross(a.angularVelocity, ra) -
                                                 (isStatic ? Vector3S.zero : b.velocity + Vector3S.Cross(b.angularVelocity, rb));

            // Effective mass calculation
            f32 invMassA = a.inverseMass;
            f32 invMassB = isStatic ? f32.zero : b.inverseMass;
            f32 effectiveMass = invMassA +
                                Vector3S.Dot(Vector3S.Cross(a.tensor.inertiaWorld * Vector3S.Cross(ra, normal), ra), normal) +
                                (isStatic ? f32.zero : invMassB +
                                Vector3S.Dot(Vector3S.Cross(b.tensor.inertiaWorld * Vector3S.Cross(rb, normal), rb), normal));

            // Correct positions based on penetration
            Vector3S correction = (penetrationDepth / effectiveMass) * normal;
            a.transform.position += invMassA * correction;
            if (!isStatic) b.transform.position -= invMassB * correction;

            // Relative velocity along the normal
            f32 velocityAlongNormal = Vector3S.Dot(relativeVelocityAtContact, normal);
            if (velocityAlongNormal > f32.zero) return;

            // Restitution (bounciness)
            f32 restitution = relativeVelocityAtContact.Magnitude() >= Settings.BounceThreshold
                ? MathS.Min(a.material.restitution, bStat.material.restitution)
                : f32.zero;

            // Baumgarte stabilization factor
            f32 baumFactor = (BAUM * penetrationDepth / DeltaTime);

            // Impulse calculation
            f32 incrementalImpulse = -(f32.one + restitution) * velocityAlongNormal / effectiveMass;
            f32 newAccumulatedImpulse = MathS.Max(contact.cachedNormalImpulse + incrementalImpulse, f32.zero);
            f32 appliedImpulse = newAccumulatedImpulse - contact.cachedNormalImpulse;
            contact.cachedNormalImpulse = newAccumulatedImpulse;

            // Apply impulses to velocities
            Vector3S normalImpulse = appliedImpulse * normal;
            Vector3S linearImpulseWithBaum = normalImpulse + (baumFactor * normal);
            a.velocity += invMassA * linearImpulseWithBaum;
            if (!isStatic) b.velocity -= invMassB * linearImpulseWithBaum;

            // Angular impulses
            a.angularVelocity += a.tensor.inertiaWorld * Vector3S.Cross(ra, normalImpulse);
            if (!isStatic) b.angularVelocity -= b.tensor.inertiaWorld * Vector3S.Cross(rb, normalImpulse);

            // Friction calculation
            Vector3S relativeTangentialVelocity = relativeVelocityAtContact - (velocityAlongNormal * normal);
            Vector3S tangent = relativeTangentialVelocity.Magnitude() > f32.zero ? relativeTangentialVelocity.Normalize() : Vector3S.zero;
            f32 frictionDenominator = invMassA +
                                      Vector3S.Dot(Vector3S.Cross(a.tensor.inertiaWorld * Vector3S.Cross(ra, tangent), ra), tangent) +
                                      (isStatic ? f32.zero : invMassB +
                                      Vector3S.Dot(Vector3S.Cross(b.tensor.inertiaWorld * Vector3S.Cross(rb, tangent), rb), tangent));
            f32 frictionImpulseScalar = -Vector3S.Dot(relativeTangentialVelocity, tangent) / frictionDenominator;

            // Apply static friction
            f32 effectiveStaticFriction = MathS.Max(a.material.staticFriction, bStat.material.staticFriction);
            if (relativeTangentialVelocity.Magnitude() < effectiveStaticFriction)
            {
                frictionImpulseScalar = f32.zero;
            }

            f32 effectiveFriction = (a.material.staticFriction + bStat.material.staticFriction) * f32.half;

            // Apply friction impulse
            Vector3S frictionImpulse = frictionImpulseScalar * tangent;
            if (frictionImpulse.Magnitude() > appliedImpulse * effectiveFriction)
            {
                frictionImpulse = frictionImpulse.Normalize() * (appliedImpulse * effectiveFriction);
            }

            a.velocity += invMassA * frictionImpulse;
            a.angularVelocity += a.tensor.inertiaWorld * Vector3S.Cross(ra, frictionImpulse);
            if (!isStatic)
            {
                b.velocity -= invMassB * frictionImpulse;
                b.angularVelocity -= b.tensor.inertiaWorld * Vector3S.Cross(rb, frictionImpulse);
            }

            // Cache impulses
            contact.cachedImpulse = normalImpulse + frictionImpulse;
            contact.cachedFrictionImpulse = frictionImpulseScalar;
        }

    }
}
