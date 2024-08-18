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
        public static f32 DeltaTime;

        private int _counter;
        private Dictionary<IntPair, ContactS> _contacts = new Dictionary<IntPair, ContactS>();
        private List<IntPair> _removeCache = new List<IntPair>();

        public event Action<ContactS> OnContact;

        public World(WorldSettings worldSettings, int startSize = 1000)
        {
            Settings = worldSettings;
            Collidables = new DumbList<Collidable>(startSize);
            Broadphase = new SortAndSweepBroadphase(startSize);
            _counter = 0;
            SimulationFrame = 0;
        }

        public Collidable AddCollidable(Collidable c)
        {
            c.Register(_counter++);
            Collidables.Add(c);
            return c;
        }

        public void Simulate(f32 deltaTime)
        {
            DeltaTime = deltaTime;

            IntegrateRigidbodies();
            UpdateCollidableTransforms();
            var pairs = Broadphase.ComputePairs(Collidables);
            NarrowPhase(pairs);

            SimulationFrame++;
        }

        private void IntegrateRigidbodies()
        {
            foreach (var c in Collidables)
            {
                if (c is RigidbodyS rb)
                {
                    rb.Integrate(DeltaTime, Settings);
                }
            }
        }

        private void UpdateCollidableTransforms()
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

        private void NarrowPhase(HashSet<IntPair> pairs)
        {
            RemoveOldContacts(pairs);
            UpdateManifolds(pairs);
            SolveCollisions(pairs);
        }

        private void RemoveOldContacts(HashSet<IntPair> pairs)
        {
            _removeCache.Clear();

            foreach (var key in _contacts.Keys)
            {
                if (!pairs.Contains(key))
                {
                    _removeCache.Add(key);
                }
            }

            foreach (var key in _removeCache)
            {
                _contacts.Remove(key);
            }
        }

        private void UpdateManifolds(HashSet<IntPair> pairs)
        {
            foreach (var pair in pairs)
            {
                UpdateManifold(pair);
            }

            pairs.RemoveWhere(x => !_contacts.ContainsKey(x));
        }

        private void UpdateManifold(IntPair pair)
        {
            var a = Collidables[pair.aIndex];
            var b = Collidables[pair.bIndex];

            // Ensure the dynamic body is 'a' for consistent processing
            if (b.isDynamic && !a.isDynamic)
            {
                (a, b) = (b, a);
            }

            var contact = new ContactS(a, b, Vector3S.zero, Vector3S.zero, f32.zero);
            var contactCount = a.collider.Intersects(b, ref contact);

            if (contactCount > 0)
            {
                if (_contacts.TryGetValue(pair, out var old))
                {
                    // On STAY: Update the manifold while preserving warm start data
                    contact.accumulatedImpulse = old.accumulatedImpulse;
                    contact.accumulatedFriction = old.accumulatedFriction;
                }

                contact.PreStep();
                _contacts[pair] = contact;
                OnContact?.Invoke(contact);
            }
            else
            {
                _contacts.Remove(pair);
            }
        }

        private void SolveCollisions(HashSet<IntPair> pairs)
        {
            for (int i = 0; i < Settings.DefaultSolverIterations; i++)
            {
                foreach (var pair in pairs)
                {
                    ResolveAndUpdateContact(pair, DeltaTime, true);
                }
            }

            if (Settings.Relaxation)
            {
                foreach (var pair in pairs)
                {
                    ResolveAndUpdateContact(pair, DeltaTime, false);
                }
            }
        }

        private void ResolveAndUpdateContact(IntPair pair, f32 delta, bool warmStart)
        {
            var contact = _contacts[pair];
            contact.ResolveContact(delta, Settings, warmStart);
            _contacts[pair] = contact;
        }
    }
}
