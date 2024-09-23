using stupid.Broadphase;
using stupid.Constraints;
using stupid.Maths;
using System;
using System.Collections.Generic;

namespace stupid
{
    public class World
    {
        public WorldSettings WorldSettings { get; private set; }
        public SortAndSweepBroadphase Broadphase { get; set; }
        public List<Collidable> Collidables { get; private set; }

        List<RigidbodyS> Bodies = new List<RigidbodyS>();
        public int SimulationFrame { get; private set; }

        public static f32 DeltaTime, InverseDeltaTime, SubDelta, InverseSubDelta;

        int _counter;

        Dictionary<IntPair, ContactManifoldSlim> ManifoldMap = new Dictionary<IntPair, ContactManifoldSlim>();

        List<IntPair> _removeCache = new List<IntPair>();

        ContactData[] _contactCache = new ContactData[4];

        public ContactSlim[] oldContacts = new ContactSlim[5000];
        public ContactSlim[] allContacts = new ContactSlim[5000];

        List<ContactManifoldSlim> _currentManifolds = new List<ContactManifoldSlim>(1000);

        public event Action<ContactManifoldSlim> OnContact;

        public World(WorldSettings worldSettings, int startSize = 1000)
        {
            WorldSettings = worldSettings;
            Collidables = new List<Collidable>(startSize);
            Broadphase = new SortAndSweepBroadphase(startSize);
            _counter = 0;
            SimulationFrame = 0;
        }

        public Collidable AddCollidable(Collidable c)
        {
            c.Register(_counter++);
            Collidables.Add(c);
            if (c is RigidbodyS rb) Bodies.Add(rb);
            return c;
        }

        public void Simulate(f32 deltaTime)
        {
            //Update delta time
            if (deltaTime != DeltaTime)
            {
                DeltaTime = deltaTime;
                InverseDeltaTime = f32.one / deltaTime;
                SubDelta = deltaTime / (f32)WorldSettings.DefaultSolverIterations;
                InverseSubDelta = InverseDeltaTime * (f32)WorldSettings.DefaultSolverIterations;
            }

            //Broadphase
            UpdateCollidableTransforms();
            var pairs = Broadphase.ComputePairs(Collidables);

            //Prepare contacts
            PrepareContacts(pairs);

            //Solve contacts + iterate?
            NarrowPhase(pairs);

            SimulationFrame++;
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

                if (c is RigidbodyS rb) rb.tensor.UpdateInertiaTensor(c.transform);
            }
        }

        void PrepareContacts(HashSet<IntPair> pairs)
        {
            _removeCache.Clear();

            int startIndex = 0;

            //Go through pairs and test collisions, share data, etc.
            foreach (var pair in pairs)
            {
                var a = Collidables[pair.aIndex];
                var b = Collidables[pair.bIndex];

                // Ensure the dynamic body is 'a' for consistent processing
                if (b.isDynamic && !a.isDynamic)
                {
                    (a, b) = (b, a);
                }

                var count = a.collider.Intersects(b, ref _contactCache);

                if (count > 0)
                {
                    var ab = (RigidbodyS)a;

                    var firstContact = _contactCache[0];
                    var manifold = new ContactManifoldSlim(ab, b, firstContact.normal, firstContact.penetrationDepth, startIndex, count);

                    for (int i = 0; i < count; i++)
                    {
                        allContacts[startIndex + i] = new ContactSlim(_contactCache[i]);
                    }

                    startIndex += count;

                    if (WorldSettings.Warmup)
                    {
                        if (ManifoldMap.TryGetValue(pair, out var oldManifold))
                        {
                            manifold.RetainData(ref allContacts, oldContacts, oldManifold);
                        }
                    }

                    manifold.CalculatePrestep(ref allContacts);

                    ManifoldMap[pair] = manifold;
                    OnContact?.Invoke(manifold);
                }
                else
                {
                    _removeCache.Add(pair);
                }
            }

            //If there are current manifolds that are not in the new broadphase, remove
            foreach (var key in ManifoldMap.Keys) if (!pairs.Contains(key)) _removeCache.Add(key);

            foreach (var key in _removeCache)
            {
                ManifoldMap.Remove(key);
                pairs.Remove(key);
            }
        }

        //Remember that the pair are still like wrong way, A ,b 
        private void NarrowPhase(HashSet<IntPair> pairs)
        {
            var dt = DeltaTime;
            var inverseDt = InverseDeltaTime;

            _currentManifolds.Clear();
            foreach (var p in pairs) _currentManifolds.Add(ManifoldMap[p]);

            if (WorldSettings.Warmup)
            {
                foreach (var m in _currentManifolds) m.Warmup(ref allContacts);
            }

            foreach (var rb in Bodies) rb.IntegrateForces(dt, WorldSettings);


            for (int iterations = 0; iterations < WorldSettings.DefaultSolverIterations; iterations++)
            {
                foreach (var m in _currentManifolds)
                {
                    m.Resolve(ref allContacts, inverseDt, WorldSettings, true);
                }
            }

            foreach (var rb in Bodies) rb.IntegrateVelocity(dt, WorldSettings);

            if (WorldSettings.Relaxation)
            {
                for (int relax = 0; relax < WorldSettings.DefaultSolverIterations; relax++)
                {
                    foreach (var m in _currentManifolds)
                    {
                        m.Resolve(ref allContacts, inverseDt, WorldSettings, false);
                    }
                }
            }

            foreach (var rb in Bodies) rb.FinalizePosition();

            // Perform a deep copy (copying data from one array to another)
            Array.Copy(allContacts, this.oldContacts, allContacts.Length);
        }
    }
}
