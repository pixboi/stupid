using stupid.Broadphase;
using stupid.Constraints;
using stupid.Maths;
using System;
using System.Collections.Generic;
using System.Linq;

namespace stupid
{
    public class World
    {
        public readonly WorldSettings WorldSettings;
        public readonly SortAndSweepBroadphase Broadphase;
        public List<Collidable> Collidables;

        List<RigidbodyS> Bodies = new List<RigidbodyS>();
        public int SimulationFrame;

        public static f32 DeltaTime, InverseDeltaTime, SubDelta, InverseSubDelta;

        int _indexCounter;

        Dictionary<IntPair, ContactManifoldSlim> ManifoldMap = new Dictionary<IntPair, ContactManifoldSlim>();
        List<IntPair> _removeCache = new List<IntPair>();

        ContactData[] _contactCache = new ContactData[4];
        public ContactSlim[] allContacts = new ContactSlim[5000];
        public ContactSlim[] oldContacts = new ContactSlim[5000];
        int _contactCount;

        int _manifoldCount;
        ContactManifoldSlim[] _manifolds = new ContactManifoldSlim[5000];

        public event Action<ContactManifoldSlim> OnContact;

        public World(WorldSettings worldSettings, int startSize = 1000)
        {
            WorldSettings = worldSettings;
            Collidables = new List<Collidable>(startSize);
            Broadphase = new SortAndSweepBroadphase(startSize);
            _indexCounter = 0;
            SimulationFrame = 0;
        }

        public Collidable AddCollidable(Collidable c)
        {
            c.Register(_indexCounter++);
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

            //The pairs, the hash set, determinism?

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

        //hash set determinism?
        void PrepareContacts(HashSet<IntPair> pairs)
        {
            _removeCache.Clear();
            _contactCount = 0;

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
                    var manifold = new ContactManifoldSlim(ab, b, firstContact.normal, firstContact.penetrationDepth, _contactCount, count);

                    //First, put the new ones in
                    for (int i = 0; i < count; i++)
                    {
                        allContacts[_contactCount + i] = new ContactSlim(_contactCache[i]);
                    }

                    //Then retain
                    if (WorldSettings.Warmup)
                    {
                        if (ManifoldMap.TryGetValue(pair, out var oldManifold))
                        {
                            manifold.RetainData(ref allContacts, oldContacts, oldManifold);
                        }
                    }

                    //Calculate the thing
                    manifold.CalculatePrestep(ref allContacts);
                    _contactCount += count;

                    //Fire off and finish
                    ManifoldMap[pair] = manifold;
                    OnContact?.Invoke(manifold);

                }
                else
                {
                    ManifoldMap.Remove(pair);
                    _removeCache.Add(pair);
                }
            }

            //If there are current manifolds that are not in the new broadphase, remove
            foreach (var key in _removeCache)
            {
                pairs.Remove(key);
            }
        }

        //Remember that the pair are still like wrong way, A ,b 
        private void NarrowPhase(HashSet<IntPair> pairs)
        {
            var dt = DeltaTime;
            var inverseDt = InverseDeltaTime;

            //For fast iteration just leave the array as is
            _manifoldCount = 0;
            foreach (var p in pairs)
            {
                _manifolds[_manifoldCount] = ManifoldMap[p];
                _manifoldCount++;
            }

            if (WorldSettings.Warmup)
            {
                for (int i = 0; i < _manifoldCount; i++)
                {
                    _manifolds[i].Warmup(ref allContacts);
                }
            }

            foreach (var rb in Bodies) rb.IntegrateForces(dt, WorldSettings);


            for (int iterations = 0; iterations < WorldSettings.DefaultSolverIterations; iterations++)
            {
                for (int i = 0; i < _manifoldCount; i++)
                {
                    _manifolds[i].Resolve(ref allContacts, inverseDt, WorldSettings, true);
                }
            }

            foreach (var rb in Bodies) rb.IntegrateVelocity(dt, WorldSettings);

            if (WorldSettings.Relaxation)
            {
                for (int relax = 0; relax < WorldSettings.DefaultSolverIterations; relax++)
                {
                    for (int i = 0; i < _manifoldCount; i++)
                    {
                        _manifolds[i].Resolve(ref allContacts, inverseDt, WorldSettings, false);
                    }
                }
            }

            foreach (var rb in Bodies) rb.FinalizePosition();

            Array.Copy(allContacts, oldContacts, allContacts.Length);
            Array.Clear(allContacts, 0, allContacts.Length);
        }
    }
}
