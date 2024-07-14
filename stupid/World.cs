
using System.Collections.Generic;
using stupid.Colliders;
using stupid.Maths;
using System;
using System.Linq;

namespace stupid
{
    public  class World
    {
        public WorldSettings Settings { get; private set; }
        public SortAndSweepBroadphase Broadphase { get; set; }
        public DumbList<Collidable> Collidables { get; private set; }
        public uint SimulationFrame { get; private set; }

        private int counter;
        public readonly Vector3S[] positionBuffer, velocityBuffer, angularBuffer;

        public World(WorldSettings worldSettings, int startSize = 1000)
        {
            this.Settings = worldSettings;

            Collidables = new DumbList<Collidable>(startSize);
            Broadphase = new SortAndSweepBroadphase(startSize);

            positionBuffer = new Vector3S[startSize * 2];
            velocityBuffer = new Vector3S[startSize * 2];
            angularBuffer = new Vector3S[startSize * 2];

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

            foreach (var c in Collidables)
            {
                if (c is RigidbodyS rb) rb.Integrate(deltaTime, Settings);
            }

            //Recalc things
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
                    if (rb.angularVelocity.SqrMagnitude > f32.zero)
                        rb.tensor.CalculateInverseInertiaTensor(rb.transform.rotation);
                }
            }

            var pairs = Broadphase.ComputePairs(Collidables);
            NarrowPhase(pairs);

            ApplyBuffers();

            SimulationFrame++;
        }

        public event Action<ContactManifoldS> OnContact;
        public ContactS[] _contactCache = new ContactS[8];
        public HashSet<int> _correctedObjects = new HashSet<int>();

        private void NarrowPhase(HashSet<BodyPair> pairs)
        {
            // Convert HashSet to List for sorting
            var pairList = pairs.ToList();
            _correctedObjects.Clear();
            // Sort pairs by Y-axis position of the lower object in each pair

            /*
            pairList.Sort((pair1, pair2) =>
            {
                var a1 = Collidables[pair1.aIndex];
                var b1 = Collidables[pair1.bIndex];
                var a2 = Collidables[pair2.aIndex];
                var b2 = Collidables[pair2.bIndex];

                f32 y1 = MathS.Min(a1.transform.position.y, b1.transform.position.y);
                f32 y2 = MathS.Min(a2.transform.position.y, b2.transform.position.y);

                return y1.CompareTo(y2);
            });
            */


            // Separate pairs into static and dynamic
            var staticPairs = new List<BodyPair>();
            var dynamicPairs = new List<BodyPair>();

            foreach (var pair in pairList)
            {
                var a = Collidables[pair.aIndex];
                var b = Collidables[pair.bIndex];

                if ((a.isDynamic && !b.isDynamic) || (!a.isDynamic && b.isDynamic))
                {
                    staticPairs.Add(pair);
                }
                else if (a.isDynamic && b.isDynamic)
                {
                    dynamicPairs.Add(pair);
                }
            }

            // Process static pairs first
            foreach (var pair in staticPairs)
            {
                var a = Collidables[pair.aIndex];
                var b = Collidables[pair.bIndex];

                // Dynamic vs Static
                if (a.isDynamic && !b.isDynamic)
                {
                    var count = a.collider.Intersects(b, ref _contactCache);

                    if (count > 0)
                    {
                        var cm = new ContactManifoldS(a, b, _contactCache, count);
                        OnContact?.Invoke(cm);
                        ResolveCollisionStatic(cm);
                    }
                    continue;
                }

                // Static vs Dynamic
                if (b.isDynamic && !a.isDynamic)
                {
                    var count = b.collider.Intersects(a, ref _contactCache);

                    if (count > 0)
                    {
                        var cm = new ContactManifoldS(b, a, _contactCache, count);
                        OnContact?.Invoke(cm);
                        ResolveCollisionStatic(cm);
                    }
                    continue;
                }
            }

            // Process dynamic pairs next
            foreach (var pair in dynamicPairs)
            {
                var a = Collidables[pair.aIndex];
                var b = Collidables[pair.bIndex];

                // Dynamic vs Dynamic
                if (a is RigidbodyS ab && b is RigidbodyS bb)
                {
                    var count = a.collider.Intersects(b, ref _contactCache);

                    if (count > 0)
                    {
                        var cm = new ContactManifoldS(a, b, _contactCache, count);
                        OnContact?.Invoke(cm);
                        ResolveCollision(cm);
                    }
                    continue;
                }
            }


        }


        public void ApplyBuffers()
        {
            foreach (var c in Collidables)
            {
                if (c.isDynamic)
                {
                    c.transform.position += positionBuffer[c.index];
                    positionBuffer[c.index] = Vector3S.zero;
                }

                if (c is RigidbodyS rb)
                {
                    rb.velocity += velocityBuffer[c.index];
                    rb.angularVelocity += angularBuffer[c.index];
                    velocityBuffer[c.index] = Vector3S.zero;
                    angularBuffer[c.index] = Vector3S.zero;
                }
            }
        }

        private static readonly f32 POSITION_PERCENT = (f32)1;
        private void ResolveCollisionStatic(ContactManifoldS manifold)
        {
            //Assume always that a is the dynamic body
            var body = (RigidbodyS)manifold.a;

            for (int i = 0; i < manifold.count; i++)
            {
                var contact = manifold.contacts[i];

                // Ensure the normal always points from contact to BODY!
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
                f32 slop = Settings.DefaultContactOffset;
                f32 penetrationDepth = MathS.Max(contact.penetrationDepth - slop, f32.zero);
                Vector3S correction = (penetrationDepth / effectiveMass) * POSITION_PERCENT * normal;
                //positionBuffer[body.index] += invMass * correction;
                body.transform.position += invMass * correction;


                // Calculate the velocity along the normal
                f32 velocityAlongNormal = Vector3S.Dot(relativeVelocityAtContact, normal);

                // Do not resolve if velocities are separating
                if (velocityAlongNormal > f32.zero) return;

                // Restitution (coefficient of restitution)
                f32 restitution = relativeVelocityAtContact.Magnitude() >= Settings.BounceThreshold ? body.material.restitution : f32.zero;

                // Calculate the normal impulse scalar
                f32 impulseScalar = -(f32.one + restitution) * velocityAlongNormal / effectiveMass;

                // Apply linear impulse
                Vector3S impulse = impulseScalar * normal;
                /*
                velocityBuffer[body.index] += invMass * impulse;
                angularBuffer[body.index] += body.tensor.inertiaWorld * Vector3S.Cross(rb, impulse);
                */
                body.velocity += invMass * impulse;
                body.angularVelocity += body.tensor.inertiaWorld * Vector3S.Cross(rb, impulse);

                // Calculate relative tangential velocity
                Vector3S relativeTangentialVelocity = relativeVelocityAtContact - (velocityAlongNormal * normal);
                Vector3S tangent = relativeTangentialVelocity.Magnitude() > f32.zero ? relativeTangentialVelocity.Normalize() : Vector3S.zero;

                // Calculate the magnitude of the friction impulse
                f32 frictionDenominator = invMass + Vector3S.Dot(Vector3S.Cross(body.tensor.inertiaWorld * Vector3S.Cross(rb, tangent), rb), tangent);
                f32 frictionImpulseScalar = -Vector3S.Dot(relativeTangentialVelocity, tangent) / frictionDenominator;

                //Just use static
                f32 effectiveFriction = (body.material.staticFriction);

                // Limit the friction impulse to prevent excessive angular velocities
                Vector3S frictionImpulse = frictionImpulseScalar * tangent;
                if (frictionImpulse.Magnitude() > impulseScalar * effectiveFriction)
                {
                    frictionImpulse = frictionImpulse.Normalize() * (impulseScalar * effectiveFriction);
                }

                // Apply friction impulse
                /*
                velocityBuffer[body.index] += invMass * frictionImpulse;
                angularBuffer[body.index] += body.tensor.inertiaWorld * Vector3S.Cross(rb, frictionImpulse);
                */

                body.velocity += invMass * frictionImpulse;
                body.angularVelocity += body.tensor.inertiaWorld * Vector3S.Cross(rb, frictionImpulse);
            }

        }

        private void ResolveCollision(ContactManifoldS manifold)
        {
            var a = (RigidbodyS)manifold.a;
            var b = (RigidbodyS)manifold.b;

            //The velocities probably need to be updated between contact runs?
            //Keep like a local cache for changes? Or average out the impulses? probably..
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
                f32 slop = Settings.DefaultContactOffset;
                f32 penetrationDepth = MathS.Max(contact.penetrationDepth - slop, f32.zero);
                Vector3S correction = (penetrationDepth / effectiveMassSum) * POSITION_PERCENT * normal;

                a.transform.position += a.inverseMass * correction;
                b.transform.position -= b.inverseMass * correction;


                // b.transform.position -= b.inverseMass * correction;
                //positionBuffer[a.index] += a.inverseMass * correction;
                //positionBuffer[b.index] -= b.inverseMass * correction;

                // Calculate the velocity along the normal
                f32 velocityAlongNormal = Vector3S.Dot(relativeVelocityAtContact, normal);

                // Do not resolve if velocities are separating
                if (velocityAlongNormal > f32.zero) return;

                // Restitution (coefficient of restitution)
                f32 restitution = relativeVelocityAtContact.Magnitude() >= Settings.BounceThreshold
                    ? (a.material.restitution + b.material.restitution) * f32.half
                    : f32.zero;

                // Calculate the normal impulse scalar
                f32 impulseScalar = -(f32.one + restitution) * velocityAlongNormal / effectiveMassSum;

                // Apply linear impulse
                Vector3S impulse = impulseScalar * normal;
                /*
                velocityBuffer[a.index] += a.inverseMass * impulse;
                velocityBuffer[b.index] -= b.inverseMass * impulse;
                angularBuffer[a.index] += a.tensor.inertiaWorld * Vector3S.Cross(ra, impulse);
                angularBuffer[b.index] -= b.tensor.inertiaWorld * Vector3S.Cross(rb, impulse);
                */

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

                //Just use static friction AVerage
                f32 effectiveFriction = (a.material.staticFriction + b.material.staticFriction) * f32.half;

                // Limit the friction impulse to prevent excessive angular velocities
                Vector3S frictionImpulse = frictionImpulseScalar * tangent;
                if (frictionImpulse.Magnitude() > impulseScalar * effectiveFriction)
                {
                    frictionImpulse = frictionImpulse.Normalize() * (impulseScalar * effectiveFriction);
                }

                // Apply friction impulse
                /*
                velocityBuffer[a.index] += a.inverseMass * frictionImpulse;
                velocityBuffer[b.index] -= b.inverseMass * frictionImpulse;
                angularBuffer[a.index] += a.tensor.inertiaWorld * Vector3S.Cross(ra, frictionImpulse);
                angularBuffer[b.index] -= b.tensor.inertiaWorld * Vector3S.Cross(rb, frictionImpulse);
                */
                a.velocity += a.inverseMass * frictionImpulse;
                b.velocity -= b.inverseMass * frictionImpulse;
                a.angularVelocity += a.tensor.inertiaWorld * Vector3S.Cross(ra, frictionImpulse);
                b.angularVelocity -= b.tensor.inertiaWorld * Vector3S.Cross(rb, frictionImpulse);
            }

        }


    }
}
