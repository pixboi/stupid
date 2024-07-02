
using System.Collections.Generic;
using SoftFloat;

namespace stupid
{
    public class SortAndSweepBroadphase : IBroadphase
    {
        private AxisEndpoint[] endpointsX;
        private AxisEndpoint[] endpointsZ;
        private readonly List<ContactPair> pairs;
        private readonly HashSet<BodyPair> checkedPairs;
        private readonly List<Rigidbody> activeList;
        private int initialCapacity;

        public SortAndSweepBroadphase(int initialCapacity = 100)
        {
            this.initialCapacity = initialCapacity;
            this.endpointsX = new AxisEndpoint[initialCapacity * 2];
            this.endpointsZ = new AxisEndpoint[initialCapacity * 2];
            this.pairs = new List<ContactPair>(initialCapacity);
            this.checkedPairs = new HashSet<BodyPair>(new BodyPairComparer());
            this.activeList = new List<Rigidbody>(initialCapacity);
        }

        public List<ContactPair> ComputePairs(List<Rigidbody> rigidbodies)
        {
            pairs.Clear();
            checkedPairs.Clear();

            EnsureCapacity(rigidbodies.Count);

            int endpointCountX = 0;
            int endpointCountZ = 0;

            PopulateEndpoints(rigidbodies, endpointsX, 0, ref endpointCountX);
            PopulateEndpoints(rigidbodies, endpointsZ, 2, ref endpointCountZ);

            InsertionSort(endpointsX, endpointCountX);
            InsertionSort(endpointsZ, endpointCountZ);

            SweepAndPrune(endpointsX, endpointCountX);
            SweepAndPrune(endpointsZ, endpointCountZ);

            return pairs;
        }

        private void EnsureCapacity(int rigidbodyCount)
        {
            int requiredCapacity = rigidbodyCount * 2;
            if (endpointsX.Length < requiredCapacity)
            {
                endpointsX = new AxisEndpoint[requiredCapacity];
                endpointsZ = new AxisEndpoint[requiredCapacity];
            }
            if (activeList.Capacity < rigidbodyCount)
            {
                activeList.Capacity = rigidbodyCount;
            }
        }

        private void PopulateEndpoints(List<Rigidbody> rigidbodies, AxisEndpoint[] endpoints, int axis, ref int endpointCount)
        {
            foreach (var body in rigidbodies)
            {
                var bounds = body.collider.GetBounds();
                var min = axis == 0 ? bounds.Min.x : bounds.Min.z;
                var max = axis == 0 ? bounds.Max.x : bounds.Max.z;

                endpoints[endpointCount++] = new AxisEndpoint { Value = min, IsMin = true, Body = body };
                endpoints[endpointCount++] = new AxisEndpoint { Value = max, IsMin = false, Body = body };
            }
        }

        private struct AxisEndpoint
        {
            public sfloat Value;
            public bool IsMin;
            public Rigidbody Body;
        }

        private void InsertionSort(AxisEndpoint[] endpoints, int count)
        {
            for (int i = 1; i < count; i++)
            {
                var key = endpoints[i];
                int j = i - 1;
                while (j >= 0 && endpoints[j].Value > key.Value)
                {
                    endpoints[j + 1] = endpoints[j];
                    j--;
                }
                endpoints[j + 1] = key;
            }
        }

        private void SweepAndPrune(AxisEndpoint[] endpoints, int count)
        {
            activeList.Clear();

            for (int i = 0; i < count; i++)
            {
                var endpoint = endpoints[i];
                if (endpoint.IsMin)
                {
                    foreach (var activeBody in activeList)
                    {
                        if (endpoint.Body == activeBody) continue;

                        var idA = endpoint.Body.index;
                        var idB = activeBody.index;
                        var bodyPair = new BodyPair(idA, idB);

                        if (!checkedPairs.Add(bodyPair))
                            continue;

                        if (endpoint.Body.collider.GetBounds().Intersects(activeBody.collider.GetBounds()))
                        {
                            pairs.Add(new ContactPair { bodyA = endpoint.Body, bodyB = activeBody });
                        }
                    }
                    activeList.Add(endpoint.Body);
                }
                else
                {
                    activeList.Remove(endpoint.Body);
                }
            }
        }


        private struct BodyPair
        {
            public int BodyA;
            public int BodyB;

            public BodyPair(int bodyA, int bodyB)
            {
                if (bodyA > bodyB)
                {
                    BodyA = bodyB;
                    BodyB = bodyA;
                }
                else
                {
                    BodyA = bodyA;
                    BodyB = bodyB;
                }
            }

            public override bool Equals(object obj)
            {
                if (obj is BodyPair other)
                {
                    return BodyA == other.BodyA && BodyB == other.BodyB;
                }
                return false;
            }

            public override int GetHashCode()
            {
                return BodyA.GetHashCode() ^ BodyB.GetHashCode();
            }
        }

        private class BodyPairComparer : IEqualityComparer<BodyPair>
        {
            public bool Equals(BodyPair x, BodyPair y)
            {
                return x.BodyA == y.BodyA && x.BodyB == y.BodyB;
            }

            public int GetHashCode(BodyPair obj)
            {
                return obj.BodyA.GetHashCode() ^ obj.BodyB.GetHashCode();
            }
        }
    }
}
