using System;
using System.Collections.Generic;
using stupid.Colliders;
using stupid.Maths;

namespace stupid
{

    public class World
    {
        public WorldSettings WorldSettings { get; private set; }
        public SortAndSweepBroadphase Broadphase { get; set; }
        public DumbList<Collidable> Collidables { get; private set; }

        List<RigidbodyS> Bodies = new List<RigidbodyS>();
        public uint SimulationFrame { get; private set; }

        public static f32 DeltaTime, InverseDeltaTime, SubDelta, InverseSubDelta;

        int _counter;
        Dictionary<IntPair, ContactManifoldS> _manifolds = new Dictionary<IntPair, ContactManifoldS>();
        List<IntPair> _removeCache = new List<IntPair>();

        public event Action<ContactManifoldS> OnContact;

        public World(WorldSettings worldSettings, int startSize = 1000)
        {
            WorldSettings = worldSettings;
            Collidables = new DumbList<Collidable>(startSize);
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
                InverseSubDelta = f32.one / SubDelta;
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

                if (c is RigidbodyS rb) rb.tensor.CalculateInverseInertiaTensor(c.transform);
            }
        }

        ContactS[] contactVectorCache = new ContactS[8];
        void PrepareContacts(HashSet<IntPair> pairs)
        {
            _removeCache.Clear();

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

                var count = a.collider.Intersects(b, ref contactVectorCache);
                if (count > 0)
                {
                    var manifold = new ContactManifoldS(a, b, (byte)count, contactVectorCache);

                    if (WorldSettings.Warmup)
                    {
                        if (_manifolds.TryGetValue(pair, out var oldM))
                        {
                            manifold.PrepareWarmup(oldM);
                        }
                    }

                    _manifolds[pair] = manifold;
                    OnContact?.Invoke(manifold);
                }
                else
                {
                    _removeCache.Add(pair);
                }
            }

            //If there are current manifolds that are not in the new broadphase, remove
            foreach (var key in _manifolds.Keys) if (!pairs.Contains(key)) _removeCache.Add(key);

            foreach (var key in _removeCache)
            {
                _manifolds.Remove(key);
                pairs.Remove(key);
            }

        }

        public void Warmup(HashSet<IntPair> pairs)
        {
            if (WorldSettings.Warmup)
            {
                foreach (var pair in pairs)
                {
                    _manifolds[pair].Warmup();
                }
            }
        }

        private void NarrowPhase1(HashSet<IntPair> pairs)
        {
            var dt = SubDelta;
            var inverseDt = InverseSubDelta;

            for (int i = 0; i < WorldSettings.DefaultSolverIterations; i++)
            {
                foreach (var rb in Bodies) rb.IntegrateForces(dt, WorldSettings);

                if (WorldSettings.Warmup) Warmup(pairs);

                foreach (var pair in pairs)
                {
                    var manifold = _manifolds[pair];
                    manifold.Resolve(inverseDt, WorldSettings, true);
                    _manifolds[pair] = manifold;
                }

                foreach (var rb in Bodies) rb.IntegrateVelocity(dt, WorldSettings);

                if (WorldSettings.Relaxation)
                {
                    foreach (var pair in pairs)
                    {
                        var manifold = _manifolds[pair];
                        manifold.Resolve(inverseDt, WorldSettings, false);
                        _manifolds[pair] = manifold;
                    }
                }
            }
        }

        private void NarrowPhase(HashSet<IntPair> pairs)
        {
            var dt = DeltaTime;
            var inverseDt = InverseDeltaTime;

            if (WorldSettings.Warmup) Warmup(pairs);

            foreach (var rb in Bodies)
                rb.IntegrateForces(dt, WorldSettings);

            for (int i = 0; i < WorldSettings.DefaultSolverIterations; i++)
            {
                foreach (var pair in pairs)
                {
                    var manifold = _manifolds[pair];
                    manifold.Resolve(inverseDt, WorldSettings, true);
                    _manifolds[pair] = manifold;
                }
            }

            foreach (var rb in Bodies)
                rb.IntegrateVelocity(dt, WorldSettings);

            if (WorldSettings.Relaxation)
            {
                for (int i = 0; i < WorldSettings.DefaultSolverIterations; i++)
                {
                    foreach (var pair in pairs)
                    {
                        var manifold = _manifolds[pair];  // Retrieve the struct (copy)
                        manifold.Resolve(inverseDt, WorldSettings, false);
                        _manifolds[pair] = manifold;  // Reinsert the modified copy back into the dictionary
                    }
                }
            }
        }
    }
}
