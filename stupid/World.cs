﻿using stupid.Broadphase;
using stupid.Constraints;
using stupid.Maths;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.CompilerServices;

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
            NarrowPhase();

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
            _manifoldCount = 0;

            UpdateData();

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

        RigidbodyData[] _data = new RigidbodyData[1000];

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void UpdateData()
        {
            if (Collidables.Count > _data.Length)
            {
                _data = new RigidbodyData[Collidables.Count];
            }

            foreach (var collidable in Collidables)
            {
                _data[collidable.index] = new RigidbodyData(collidable);
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void ApplyData()
        {
            foreach (var d in _data)
            {
                Collidables[d.index].Apply(d);
            }
        }

        private void NarrowPhase()
        {
            var dt = DeltaTime;

            var manifoldSpan = new ReadOnlySpan<ContactManifoldSlim>(_manifolds, 0, _manifoldCount);
            var contactSpan = new Span<ContactSlim>(allContacts, 0, _contactCount);

            //Have some kind of centralized rigidbody data array here
            if (WorldSettings.Warmup)
            {
                foreach (var m in manifoldSpan)
                {
                    m.Warmup(ref _data[m.aIndex], ref _data[m.bIndex], ref contactSpan);
                }
            }

            ApplyData();

            foreach (var c in Collidables)
            {
                c.IntegrateForces(dt, WorldSettings);
            }

            UpdateData();

            //Now were operating on pure data
            for (int iterations = 0; iterations < WorldSettings.DefaultSolverIterations; iterations++)
            {
                for (int i = 0; i < _manifoldCount; i++)
                {
                    ref var m = ref _manifolds[i];
                    m.Resolve(ref _data[m.aIndex], ref _data[m.bIndex], ref contactSpan, true);
                }
            }

            //Apply the calcs from data
            ApplyData();

            foreach (var c in Collidables)
            {
                c.IntegrateVelocity(dt, WorldSettings);
            }

            if (WorldSettings.Relaxation)
            {
                UpdateData();

                for (int relax = 0; relax < WorldSettings.DefaultSolverVelocityIterations; relax++)
                {
                    for (int i = 0; i < _manifoldCount; i++)
                    {
                        ref var m = ref _manifolds[i];
                        m.Resolve(ref _data[m.aIndex], ref _data[m.bIndex], ref contactSpan, false);
                    }
                }

                ApplyData();
            }


            Array.Copy(allContacts, oldContacts, allContacts.Length);
            Array.Clear(allContacts, 0, allContacts.Length);
        }
    }
}
