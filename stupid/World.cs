using System;
using System.Collections.Generic;
using System.Linq;
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
        public int SimulationFrame { get; private set; }

        public static f32 DeltaTime, InverseDeltaTime, SubDelta, InverseSubDelta;

        int _counter;
        Dictionary<IntPair, ContactManifoldS> ManifoldMap = new Dictionary<IntPair, ContactManifoldS>();
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

        ContactS[] contactVectorCache = new ContactS[4];
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
                    var manifold = new ContactManifoldS(a, b, count, contactVectorCache);

                    if (WorldSettings.Warmup)
                    {
                        if (ManifoldMap.TryGetValue(pair, out var oldM))
                        {
                            //This retains data
                            manifold.RetainData(oldM);
                        }
                    }

                    //Then we calc the mass
                    manifold.CalculatePrestep();

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

        List<ContactManifoldS> _currentManifolds = new List<ContactManifoldS>(1000);

        //Maybe there just enough accuracy in fixed point to support tgs?
        private void NarrowPhase1(HashSet<IntPair> pairs)
        {
            var dt = SubDelta;
            var inverseDt = InverseSubDelta;

            _currentManifolds.Clear();
            foreach (var p in pairs)
                _currentManifolds.Add(ManifoldMap[p]);

            //Add a the current .SimulationFrame as an offset to the list, like rotate it with that, should be kind of deterministic?
            int frameOffset = SimulationFrame % _currentManifolds.Count;
            if (frameOffset > 0)
            {
                // Perform in-place rotation
                RotateListInPlace(_currentManifolds, frameOffset);
            }

            for (int substep = 0; substep < WorldSettings.DefaultSolverIterations; substep++)
            {


                foreach (var rb in Bodies)
                    rb.IntegrateForces(dt, WorldSettings);

                if (WorldSettings.Warmup)
                    foreach (var m in _currentManifolds) m.Warmup();

                for (int i = 0; i < _currentManifolds.Count; i++)
                {
                    var m = _currentManifolds[i];
                    m.Resolve(inverseDt, WorldSettings, true);
                    _currentManifolds[i] = m;
                }

                foreach (var rb in Bodies)
                    rb.IntegrateVelocity(dt, WorldSettings);

                for (int i = 0; i < _currentManifolds.Count; i++)
                {
                    var m = _currentManifolds[i];
                    m.SubtickUpdate();
                    _currentManifolds[i] = m;
                }

                if (WorldSettings.Relaxation)
                {
                    for (int i = 0; i < _currentManifolds.Count; i++)
                    {
                        var m = _currentManifolds[i];
                        m.Resolve(inverseDt, WorldSettings, false);
                        _currentManifolds[i] = m;
                    }
                }


            }

            foreach (var rb in Bodies) rb.FinalizePosition();

            //Save changes
            for (int i = 0; i < _currentManifolds.Count; i++)
            {
                var m = _currentManifolds[i];
                ManifoldMap[m.ToPair()] = m;
            }
        }

        // In-place rotation method
        private void RotateListInPlace(List<ContactManifoldS> list, int offset)
        {
            int n = list.Count;

            // Reverse the first part (0 to offset-1)
            ReverseList(list, 0, offset - 1);

            // Reverse the second part (offset to n-1)
            ReverseList(list, offset, n - 1);

            // Reverse the whole list (0 to n-1)
            ReverseList(list, 0, n - 1);
        }

        // Helper method to reverse a portion of the list in place
        private void ReverseList(List<ContactManifoldS> list, int start, int end)
        {
            while (start < end)
            {
                var temp = list[start];
                list[start] = list[end];
                list[end] = temp;
                start++;
                end--;
            }
        }

        private void NarrowPhase(HashSet<IntPair> pairs)
        {
            var dt = DeltaTime;
            var inverseDt = InverseDeltaTime;

            _currentManifolds.Clear();
            foreach (var p in pairs)
                _currentManifolds.Add(ManifoldMap[p]);

            /* this caused division by zero error?*/
            //Add a the current .SimulationFrame as an offset to the list, like rotate it with that, should be kind of deterministic?
            if (_currentManifolds.Count > 1)
            {
                int frameOffset = SimulationFrame % _currentManifolds.Count;
                if (frameOffset > 0)
                {
                    // Perform in-place rotation
                    RotateListInPlace(_currentManifolds, frameOffset);
                }
            }

            if (WorldSettings.Warmup)
                foreach (var m in _currentManifolds) m.Warmup();

            foreach (var rb in Bodies)
                rb.IntegrateForces(dt, WorldSettings);

            for (int iter = 0; iter < WorldSettings.DefaultSolverIterations; iter++)
            {
                for (int i = 0; i < _currentManifolds.Count; i++)
                {
                    var m = _currentManifolds[i];
                    m.Resolve(inverseDt, WorldSettings, true);
                    _currentManifolds[i] = m;
                }
            }

            foreach (var rb in Bodies)
                rb.IntegrateVelocity(dt, WorldSettings);

            if (WorldSettings.Relaxation)
            {
                for (int relax = 0; relax < WorldSettings.DefaultSolverIterations; relax++)
                {
                    for (int i = 0; i < _currentManifolds.Count; i++)
                    {
                        var m = _currentManifolds[i];
                        m.Resolve(inverseDt, WorldSettings, false);
                        _currentManifolds[i] = m;
                    }
                }
            }

            foreach (var rb in Bodies) rb.FinalizePosition();

            //Save changes
            for (int i = 0; i < _currentManifolds.Count; i++)
            {
                var m = _currentManifolds[i];
                ManifoldMap[m.ToPair()] = m;
            }
        }
    }
}
