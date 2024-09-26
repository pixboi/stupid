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
        public int SimulationFrame;
        public readonly WorldSettings WorldSettings;
        public readonly SortAndSweepBroadphase Broadphase;
        public List<Collidable> Collidables;
        int _indexCounter;

        public static f32 DeltaTime, InverseDeltaTime, SubDelta, InverseSubDelta;

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
            SimulationFrame = 0;

            _indexCounter = 0;
            _contactCount = 0;
            _manifoldCount = 0;

            _boundsIndices = new BoundsIndex[startSize * 2];
        }

        public Collidable AddCollidable(Collidable c)
        {
            c.Register(_indexCounter++);
            Collidables.Add(c);
            return c;
        }

        BoundsIndex[] _boundsIndices;
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

            Array.Clear(_boundsIndices, 0, _boundsIndices.Length);
            int boundsLength = 0;
            foreach (Collidable c in Collidables)
            {
                _boundsIndices[boundsLength++] = new BoundsIndex(c.bounds, c.index);
            }

            var pairs = Broadphase.ComputePairs(_boundsIndices, boundsLength);

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

                if (c.isDynamic) c.tensor.UpdateInertiaTensor(c.transform);

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

                //Skip static + static
                if (!b.isDynamic && !a.isDynamic)
                {
                    _removeCache.Add(pair);
                    continue;
                }

                // Ensure the dynamic body is 'a' for consistent processing
                if (b.isDynamic && !a.isDynamic)
                {
                    (a, b) = (b, a);
                }

                var count = a.collider.Intersects(b, ref _contactCache);

                if (count > 0)
                {
                    var firstContact = _contactCache[0];
                    var manifold = new ContactManifoldSlim(a, b, firstContact.normal, firstContact.penetrationDepth, _contactCount, count);

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
                if (ManifoldMap.TryGetValue(p, out var m))
                {
                    _manifolds[_manifoldCount++] = m;
                }
            }

            var manifoldSpan = new ReadOnlySpan<ContactManifoldSlim>(_manifolds, 0, _manifoldCount);
            var contactSpan = new Span<ContactSlim>(allContacts, 0, _contactCount);

            if (WorldSettings.Warmup)
            {
                foreach (var manifold in manifoldSpan)
                {
                    manifold.Warmup(ref contactSpan);
                }
            }

            foreach (var c in Collidables)
            {
                c.IntegrateForces(dt, WorldSettings);
            }

            for (int iterations = 0; iterations < WorldSettings.DefaultSolverIterations; iterations++)
            {
                foreach (var manifold in manifoldSpan)
                {
                    manifold.Resolve(ref contactSpan, inverseDt, WorldSettings, true);
                }
            }

            foreach (var c in Collidables)
            {
                c.IntegrateVelocity(dt, WorldSettings);
            }

            if (WorldSettings.Relaxation)
            {
                for (int relax = 0; relax < WorldSettings.DefaultSolverVelocityIterations; relax++)
                {
                    foreach (var manifold in manifoldSpan)
                    {
                        manifold.Resolve(ref contactSpan, inverseDt, WorldSettings, false);
                    }
                }
            }


            Array.Copy(allContacts, oldContacts, allContacts.Length);
            Array.Clear(allContacts, 0, allContacts.Length);
        }
    }
}
