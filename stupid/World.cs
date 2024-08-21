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
        public uint SimulationFrame { get; private set; }

        public static f32 DeltaTime;

        private int _counter;
        private Dictionary<IntPair, ContactManifoldS> _manifolds = new Dictionary<IntPair, ContactManifoldS>();
        private List<IntPair> _removeCache = new List<IntPair>();

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
            return c;
        }

        public void Simulate(f32 deltaTime)
        {
            DeltaTime = deltaTime;

            UpdateCollidableTransforms();
            var pairs = Broadphase.ComputePairs(Collidables);
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

                if (c is RigidbodyS rb)
                {
                    rb.tensor.CalculateInverseInertiaTensor(rb.transform.rotation);
                }
            }
        }

        ContactS[] contactVectorCache = new ContactS[8];
        private void UpdateManifold(IntPair pair)
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
                var arr = new ContactS[count];
                Array.Copy(contactVectorCache, arr, count);
                var manifold = new ContactManifoldS(a, b, arr);

                /* Warm starting doesnt seem to do anything
            // On STAY: Update the manifold while preserving warm start data
            if (_manifolds.TryGetValue(pair, out var old))
            {

                for (int i = 0; i < count; i++)
                {
                    var c1 = manifold.contacts[i];

                    for (int j = 0; j < old.contacts.Length; j++)
                    {
                        var c2 = old.contacts[j];

                        if (c1.featureId == c2.featureId)
                        {
                            //c1.accumulatedFriction = c2.accumulatedFriction;
                            //c1.accumulatedImpulse = c2.accumulatedImpulse;
                        }
                    }

                    manifold.contacts[i] = c1;
                }
                
            }
                */

                _manifolds[pair] = manifold;
                OnContact?.Invoke(manifold);
            }
            else
            {
                _manifolds.Remove(pair);
            }
        }

        private void NarrowPhase(HashSet<IntPair> pairs)
        {
            _removeCache.Clear();

            //Remove olds that are not in the new broadphase
            foreach (var key in _manifolds.Keys) if (!pairs.Contains(key)) _removeCache.Add(key);
            foreach (var key in _removeCache) _manifolds.Remove(key);

            //Update new and check cols
            foreach (var pair in pairs) UpdateManifold(pair);
            //If no narrowphase col, remove
            pairs.RemoveWhere(x => !_manifolds.ContainsKey(x));

            var subDelta = DeltaTime / (f32)WorldSettings.DefaultSolverIterations;
            var inverseSubDelta = DeltaTime * (f32)WorldSettings.DefaultSolverIterations;

            for (int i = 0; i < WorldSettings.DefaultSolverIterations; i++)
            {
                foreach (var pair in pairs)
                {
                    var manifold = _manifolds[pair];  // Retrieve the struct (copy)
                    manifold.Resolve(inverseSubDelta, WorldSettings, true);
                    _manifolds[pair] = manifold;  // Reinsert the modified copy back into the dictionary
                }

                IntegrateRigidbodies(subDelta);
                UpdateCollidableTransforms();
            }
        }

        private void IntegrateRigidbodies(f32 delta)
        {
            foreach (var c in Collidables)
            {
                if (c is RigidbodyS rb)
                {
                    rb.Integrate(delta, WorldSettings);
                }
            }
        }

    }
}
