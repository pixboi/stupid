using Newtonsoft.Json;
using stupid.Broadphase;
using stupid.Constraints;
using stupid.Maths;
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Xml;

namespace stupid
{
    public class World
    {
        public int SimulationFrame;
        public readonly WorldSettings WorldSettings;
        public readonly SortAndSweepBroadphase Broadphase;
        public Collidable[] Collidables;
        int _indexCounter;

        public static f32 DeltaTime, InverseDeltaTime;

        BoundsIndex[] _boundsIndices;
        Dictionary<IntPair, ContactManifoldSlim> ManifoldMap;
        List<IntPair> _removeCache = new List<IntPair>();

        ContactData[] _contactCache = new ContactData[4];
        public ContactSlim[] allContacts = new ContactSlim[5000];
        public ContactSlim[] oldContacts = new ContactSlim[5000];
        int _contactCount;

        int _manifoldCount;
        ContactManifoldSlim[] _manifolds;
        RigidbodyData[] _data;

        public event Action<ContactManifoldSlim> OnContact;

        public World(in WorldSettings worldSettings, int startSize = 1000)
        {
            WorldSettings = worldSettings;
            Collidables = new Collidable[startSize];
            _boundsIndices = new BoundsIndex[startSize];
            Broadphase = new SortAndSweepBroadphase(startSize);
            _data = new RigidbodyData[startSize];
            ManifoldMap = new Dictionary<IntPair, ContactManifoldSlim>(startSize * startSize);
            _manifolds = new ContactManifoldSlim[startSize * startSize];
            SimulationFrame = 0;

            _indexCounter = 0;
            _contactCount = 0;
            _manifoldCount = 0;
        }

        public World(in WorldSettings settings, List<Collidable> collidables)
        {
            var startSize = collidables.Count;

            WorldSettings = settings;
            this.Collidables = new Collidable[startSize];
            foreach (var c in collidables) AddCollidable(c);

            _boundsIndices = new BoundsIndex[startSize];
            Broadphase = new SortAndSweepBroadphase(startSize);
            ManifoldMap = new Dictionary<IntPair, ContactManifoldSlim>(startSize * startSize);
            _manifolds = new ContactManifoldSlim[startSize * startSize];
            _data = new RigidbodyData[startSize];
            SimulationFrame = 0;

            _indexCounter = 0;
            _contactCount = 0;
            _manifoldCount = 0;
        }

        public Collidable AddCollidable(Collidable c)
        {
            Collidables[_indexCounter] = c;
            c.Register(_indexCounter);
            _indexCounter++;
            return c;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void ApplyData()
        {
            foreach (var collidable in Collidables)
            {
                if (collidable.isDynamic)
                    collidable.Apply(_data[collidable.index]);
            }
        }

        //hash set determinism?
        void PrepareContacts(HashSet<IntPair> pairs)
        {
            _removeCache.Clear();
            _contactCount = 0;
            _manifoldCount = 0;

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
                    var manifold = new ContactManifoldSlim(a, b, firstContact.normal, firstContact.penetrationDepth, this.WorldSettings, InverseDeltaTime, _contactCount, count);

                    //First, put the new ones in
                    for (int i = 0; i < count; i++)
                    {
                        allContacts[_contactCount + i] = new ContactSlim(a, b, _contactCache[i]);
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
                    manifold.CalculatePrestep(_data[manifold.aIndex], _data[manifold.bIndex], ref allContacts);
                    _contactCount += count;

                    //Fire off and finish
                    ManifoldMap[pair] = manifold;
                    OnContact?.Invoke(manifold);

                    //For fast iteration just leave the array as is
                    _manifolds[_manifoldCount++] = manifold;
                }
                else
                {
                    _removeCache.Add(pair);
                }
            }

            //If there are current manifolds that are not in the new broadphase, remove
            foreach (var key in _removeCache)
            {
                ManifoldMap.Remove(key);
                pairs.Remove(key);
            }
        }


        public void Simulate(in f32 dt)
        {
            #region broadphase
            if (dt != DeltaTime)
            {
                DeltaTime = dt;
                InverseDeltaTime = f32.one / dt;
            }

            //Broadphase
            foreach (var c in Collidables)
            {
                if (c.collider.NeedsRotationUpdate)
                {
                    c.transform.UpdateRotationMatrix();
                    c.collider.OnRotationUpdate();
                }

                c.CalculateBounds();

                if (c.isDynamic)
                    c.tensor.UpdateInertiaTensor(c.transform);
            }

            Array.Clear(_boundsIndices, 0, _boundsIndices.Length);
            int boundsLength = 0;
            foreach (Collidable c in Collidables) _boundsIndices[boundsLength++] = new BoundsIndex(c.bounds, c.index);
            //The pairs, the hash set, determinism?
            var broadPhasePairs = Broadphase.ComputePairs(_boundsIndices, boundsLength);
            #endregion

            #region narrowphase
            //Integrate Forces
            foreach (var c in Collidables)
            {
                if (c.isDynamic)
                {
                    c.IntegrateForces(dt, WorldSettings);
                    _data[c.index] = new RigidbodyData(c);
                }
            }

            //Preare contacts
            PrepareContacts(broadPhasePairs);
            var manifoldSpan = new ReadOnlySpan<ContactManifoldSlim>(_manifolds, 0, _manifoldCount);
            var contactSpan = new Span<ContactSlim>(allContacts, 0, _contactCount);

            //Warmstart
            if (WorldSettings.Warmup)
            {
                foreach (var m in manifoldSpan)
                    m.Warmup(ref _data[m.aIndex], ref _data[m.bIndex], ref contactSpan);
            }

            //Solve
            for (int iterations = 0; iterations < WorldSettings.DefaultSolverIterations; iterations++)
            {
                foreach (var m in manifoldSpan) m.Resolve(ref _data[m.aIndex], ref _data[m.bIndex], ref contactSpan);
            }

            ApplyData();

            //Apply the changes and integrate
            foreach (var c in Collidables)
            {
                if (c.isDynamic)
                {
                    c.IntegrateVelocity(dt, WorldSettings);
                    _data[c.index] = new RigidbodyData(c);
                }
            }

            if (WorldSettings.Relaxation)
            {
                for (int relax = 0; relax < WorldSettings.DefaultSolverVelocityIterations; relax++)
                {
                    foreach (var m in manifoldSpan) m.Resolve(ref _data[m.aIndex], ref _data[m.bIndex], ref contactSpan, false);
                }

                ApplyData();
            }

            //Finalize
            foreach (var c in Collidables)
                c.FinalizePosition();

            Array.Copy(allContacts, oldContacts, allContacts.Length);
            Array.Clear(allContacts, 0, allContacts.Length);
            SimulationFrame++;

            #endregion
        }

    }
}