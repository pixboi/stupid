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
                InverseSubDelta = deltaTime * (f32)WorldSettings.DefaultSolverIterations;
            }

            //Integrate forces, gravity etc.
            foreach (var c in Collidables) if (c is RigidbodyS rb) rb.IntegrateForces(DeltaTime, WorldSettings);

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

                if (c is RigidbodyS rb)
                {
                    rb.tensor.CalculateInverseInertiaTensor(c.transform);
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

                /*

                if (_manifolds.TryGetValue(pair, out var old))
                {
                    
                    for (int i = 0; i < count; i++)
                    {
                        var c = manifold.contacts[i];

                        for (int j = 0; j < old.contacts.Length; j++)
                        {
                            var oldContact = old.contacts[j];

                            if (c.featureId == oldContact.featureId)
                            {
                                c.accumulatedImpulse = oldContact.accumulatedImpulse;
                                c.accumulatedFriction = oldContact.accumulatedFriction;
                            }
                        }

                        manifold.contacts[i] = c;
                    }
                    
                }
                */


                //This adds stability, we solve them immeaditly, so we get sequentially more and more information
                //Only now it doesnt modify position?
                if (WorldSettings.Presolve)
                {
                    manifold.Resolve(DeltaTime, WorldSettings, true);

                }

                _manifolds[pair] = manifold;
                OnContact?.Invoke(manifold);
            }
            else
            {
                _manifolds.Remove(pair);
            }
        }

        void PrepareContacts(HashSet<IntPair> pairs)
        {
            _removeCache.Clear();
            //If there are current manifolds that are not in the new broadphase, remove
            foreach (var key in _manifolds.Keys) if (!pairs.Contains(key)) _removeCache.Add(key);
            foreach (var key in _removeCache) _manifolds.Remove(key);


            //Go through pairs and test collisions, share data, etc.
            foreach (var pair in pairs) UpdateManifold(pair);

            //Remove pairs that didnt survive the collision test
            pairs.RemoveWhere(x => !_manifolds.ContainsKey(x));
        }

        bool TGS = false;

        private void NarrowPhase(HashSet<IntPair> pairs)
        {
            var dt = DeltaTime;

            for (int i = 0; i < WorldSettings.DefaultSolverIterations; i++)
            {
                foreach (var pair in pairs)
                {
                    var manifold = _manifolds[pair];
                    manifold.Resolve(InverseDeltaTime, WorldSettings, true);
                    _manifolds[pair] = manifold;
                }

                if (WorldSettings.Relaxation)
                {
                    foreach (var pair in pairs)
                    {
                        var manifold = _manifolds[pair];  // Retrieve the struct (copy)
                        manifold.Resolve(InverseDeltaTime, WorldSettings, false);
                        _manifolds[pair] = manifold;  // Reinsert the modified copy back into the dictionary
                    }
                }


            }

            foreach (var c in Collidables) if (c is RigidbodyS rb) rb.IntegrateVelocity(dt, WorldSettings);

        }

        private void NarrowPhaseTGS(HashSet<IntPair> pairs)
        {
            var dt = InverseSubDelta;

            for (int i = 0; i < WorldSettings.DefaultSolverIterations; i++)
            {
                foreach (var pair in pairs)
                {
                    var manifold = _manifolds[pair];  // Retrieve the struct (copy)
                    manifold.Resolve(dt, WorldSettings, true);
                    _manifolds[pair] = manifold;  // Reinsert the modified copy back into the dictionary
                }


                foreach (var c in Collidables)
                {
                    if (c is RigidbodyS rb)
                    {
                        rb.IntegrateVelocity(SubDelta, WorldSettings);
                    }
                }


                if (WorldSettings.Relaxation)
                {
                    foreach (var pair in pairs)
                    {
                        var manifold = _manifolds[pair];  // Retrieve the struct (copy)
                        manifold.Resolve(dt, WorldSettings, false);
                        _manifolds[pair] = manifold;  // Reinsert the modified copy back into the dictionary
                    }
                }

            }

        }


    }
}
