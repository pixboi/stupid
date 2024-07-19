using System;
using System.Collections.Generic;
using System.Linq;
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
                    rb.tensor.CalculateInverseInertiaTensor(rb.transform.rotation);
                }
            }
        }

        public event Action<ContactS> OnContact;
        private Dictionary<BodyPair, ContactS> _contacts = new Dictionary<BodyPair, ContactS>();

        private void UpdateManifold(BodyPair pair)
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

                if (!_contacts.TryGetValue(pair, out var oldManifold))
                {
                    // On ENTER: Add a new manifold
                    _contacts[pair] = contact;
                }
                else
                {
                    // On STAY: Update the manifold while preserving warm start data
                    // Copy old manifold's cached impulses to the new manifold
                    contact.cachedNormalImpulse = oldManifold.cachedNormalImpulse;
                    contact.cachedFrictionImpulse = oldManifold.cachedFrictionImpulse;
                    contact.cachedImpulse = oldManifold.cachedImpulse;
                    _contacts[pair] = contact;
                }

                // Trigger the contact event
                OnContact?.Invoke(contact);
            }
            else if (_contacts.ContainsKey(pair))
            {
                // On EXIT: Remove the manifold
                _contacts.Remove(pair);
            }
        }

        List<BodyPair> _removeCache = new List<BodyPair>();
        private void NarrowPhase(HashSet<BodyPair> pairs)
        {
            // Collect keys that were not touched by the broadphase
            _removeCache.Clear();
            foreach (var key in _contacts.Keys)
            {
                if (!pairs.Contains(key))
                {
                    _removeCache.Add(key);
                }
            }

            // Remove the old keys
            foreach (var key in _removeCache)
            {
                _contacts.Remove(key);
            }

            // Update the manifolds for the current pairs
            foreach (var pair in pairs)
            {
                UpdateManifold(pair);
            }

            pairs.RemoveWhere(x => !_contacts.ContainsKey(x));

            /// var sortedContacts = _contacts.Values.OrderByDescending(x => x.penetrationDepth);
            //var sorted = _contacts.Values.OrderByDescending(x => x.penetrationDepth);

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

        private void ApplyWarmStarting(ref ContactS contact)
        {
            var a = contact.a as RigidbodyS;
            var b = contact.b as RigidbodyS;
            // return;

            if (a != null)
            {
                a.velocity += a.inverseMass * contact.cachedImpulse;
                a.angularVelocity += a.tensor.inertiaWorld * Vector3S.Cross(contact.point - a.transform.position, contact.cachedImpulse);
            }

            if (b != null)
            {
                b.velocity -= b.inverseMass * contact.cachedImpulse;
                b.angularVelocity -= b.tensor.inertiaWorld * Vector3S.Cross(contact.point - b.transform.position, contact.cachedImpulse);
            }

            contact.ResetCachedImpulses();
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


        private static readonly f32 BAUM = (f32)0.2;
        private void ResolveContact(RigidbodyS a, Collidable b, ref ContactS contact, bool isStatic = false)
        {
            f32 slop = Settings.DefaultContactOffset;
            f32 normalSlop = (f32)0.1;

            var contactPointA = a.transform.position + contact.pA;
            var contactPointB = b.transform.position + contact.pB;

            Vector3S normal = contact.normal;
            if (Vector3S.DistanceSquared(contactPointA, contactPointB) > normalSlop) normal = ((contactPointA) - (contactPointB)).Normalize();

            // Compute the current contact separation for a sub-step
            f32 penetrationDepth = Vector3S.Dot((b.transform.position + contact.pB) - (a.transform.position + contact.pA), normal) + contact.penetrationDepth;
            penetrationDepth = MathS.Max(penetrationDepth - slop, f32.zero);
            if (penetrationDepth == f32.zero) return;

            Vector3S ra = contactPointA - a.transform.position;
            RigidbodyS? bb = isStatic ? null : b as RigidbodyS;
            Vector3S rb = isStatic ? Vector3S.zero : contactPointB - bb.transform.position;

            Vector3S relativeVelocityAtContact = a.velocity + Vector3S.Cross(a.angularVelocity, ra) -
                (isStatic ? Vector3S.zero : bb.velocity + Vector3S.Cross(bb.angularVelocity, rb));

            f32 invMassA = a.inverseMass;
            f32 invMassB = isStatic ? f32.zero : bb.inverseMass;
            f32 effectiveMass = invMassA + Vector3S.Dot(Vector3S.Cross(a.tensor.inertiaWorld * Vector3S.Cross(ra, normal), ra), normal) +
                (isStatic ? f32.zero : invMassB + Vector3S.Dot(Vector3S.Cross(bb.tensor.inertiaWorld * Vector3S.Cross(rb, normal), rb), normal));

            Vector3S correction = (penetrationDepth / effectiveMass) * normal;
            a.transform.position += invMassA * correction;
            if (!isStatic)
            {
                bb.transform.position -= invMassB * correction;
            }

            f32 velocityAlongNormal = Vector3S.Dot(relativeVelocityAtContact, normal);
            if (velocityAlongNormal > f32.zero) return;

            f32 restitution = relativeVelocityAtContact.Magnitude() >= Settings.BounceThreshold
                ? MathS.Min(a.material.restitution, b.material.restitution)
                : f32.zero;

            //Should baum be applied to the angular impulse as well? i dont think so
            f32 baumFactor = (BAUM * penetrationDepth / DeltaTime);
            f32 incrementalImpulse = -(f32.one + restitution) * velocityAlongNormal / effectiveMass + baumFactor;
            f32 newAccumulatedImpulse = MathS.Max(contact.cachedNormalImpulse + incrementalImpulse, f32.zero);
            f32 appliedImpulse = newAccumulatedImpulse - contact.cachedNormalImpulse;
            contact.cachedNormalImpulse = newAccumulatedImpulse;

            Vector3S normalImpulse = appliedImpulse * normal;
            a.velocity += invMassA * normalImpulse;
            a.angularVelocity += a.tensor.inertiaWorld * Vector3S.Cross(ra, normalImpulse);
            if (!isStatic)
            {
                bb.velocity -= invMassB * normalImpulse;
                bb.angularVelocity -= bb.tensor.inertiaWorld * Vector3S.Cross(rb, normalImpulse);
            }

            Vector3S relativeTangentialVelocity = relativeVelocityAtContact - (velocityAlongNormal * normal);
            Vector3S tangent = relativeTangentialVelocity.Magnitude() > f32.zero ? relativeTangentialVelocity.Normalize() : Vector3S.zero;
            f32 frictionDenominator = invMassA + Vector3S.Dot(Vector3S.Cross(a.tensor.inertiaWorld * Vector3S.Cross(ra, tangent), ra), tangent) +
                (isStatic ? f32.zero : invMassB + Vector3S.Dot(Vector3S.Cross(bb.tensor.inertiaWorld * Vector3S.Cross(rb, tangent), rb), tangent));
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
                bb.velocity -= invMassB * frictionImpulse;
                bb.angularVelocity -= bb.tensor.inertiaWorld * Vector3S.Cross(rb, frictionImpulse);
            }

            contact.cachedImpulse = normalImpulse + frictionImpulse;
            contact.cachedFrictionImpulse = frictionImpulseScalar;

        }
    }
}
