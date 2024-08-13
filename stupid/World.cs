using System;
using System.Collections.Generic;
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
            foreach (var c in Collidables)
            {
                if (c is RigidbodyS rb) rb.Integrate(deltaTime, Settings);
            }

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

            var pairs = Broadphase.ComputePairs(Collidables);
            NarrowPhase(pairs);
            SimulationFrame++;
        }


        public event Action<ContactS> OnContact;
        private Dictionary<IntPair, ContactS> _contacts = new Dictionary<IntPair, ContactS>();

        //Could there be a way of approximating just how penetrated things are on brtoadphase?
        private void UpdateManifold(IntPair pair)
        {
            var a = Collidables[pair.aIndex];
            var b = Collidables[pair.bIndex];

            f32 subDelta = DeltaTime / (f32)Settings.DefaultSolverIterations;

            // Ensure the dynamic body is 'a' for consistent processing
            if (b.isDynamic && !a.isDynamic)
            {
                (a, b) = (b, a);
            }

            var contact = new ContactS(a, b, Vector3S.zero, Vector3S.zero, f32.zero);
            var contactCount = a.collider.Intersects(b, ref contact);

            if (contactCount > 0)
            {

                if (_contacts.TryGetValue(pair, out var oldManifold))
                {
                    // On STAY: Update the manifold while preserving warm start data
                    // Copy old manifold's cached impulses to the new manifold
                    contact.accumulatedImpulse = oldManifold.accumulatedImpulse;
                }

                contact.PreStep();

                //Resolve straight a way? this way we get more accurate collision data on the proceeding pairs?
                //contact.ResolveContact(DeltaTime, Settings);

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

        private List<IntPair> _removeCache = new List<IntPair>();
        private void NarrowPhase(HashSet<IntPair> pairs)
        {
            // Collect keys that were not touched by the broadphase
            _removeCache.Clear();

            foreach (var key in _contacts.Keys)
                if (!pairs.Contains(key))
                    _removeCache.Add(key);

            // Remove the old keys
            foreach (var key in _removeCache)
                _contacts.Remove(key);

            // Update the manifolds for the current pairs
            foreach (var pair in pairs)
                UpdateManifold(pair);

            pairs.RemoveWhere(x => !_contacts.ContainsKey(x));

            // Solve collisions
            for (int i = 0; i < Settings.DefaultSolverIterations; i++)
            {
                foreach (var pair in pairs)
                {
                    var contact = _contacts[pair];
                    contact.ResolveContact(DeltaTime, Settings);
                    _contacts[pair] = contact;
                }
            }

            //Relax
            foreach (var pair in pairs)
            {
                var contact = _contacts[pair];
                //Relax should not apply position changes
                contact.ResolveContact(DeltaTime, Settings, false);
                _contacts[pair] = contact;
            }
        }
    }
}
