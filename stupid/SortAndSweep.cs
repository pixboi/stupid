using System.Collections.Generic;
using SoftFloat;

namespace stupid
{
    public class SortAndSweepBroadphase : IBroadphase
    {
        private AxisEndpoint[] endpointsX;
        private AxisEndpoint[] endpointsY;
        private AxisEndpoint[] endpointsZ;
        private readonly List<ContactPair> pairs;
        private readonly Dictionary<(int, int), bool> checkedPairs;
        private int initialCapacity;

        public SortAndSweepBroadphase(int initialCapacity = 100)
        {
            this.initialCapacity = initialCapacity;
            this.endpointsX = new AxisEndpoint[initialCapacity * 2];
            this.endpointsY = new AxisEndpoint[initialCapacity * 2];
            this.endpointsZ = new AxisEndpoint[initialCapacity * 2];
            this.pairs = new List<ContactPair>(initialCapacity);
            this.checkedPairs = new Dictionary<(int, int), bool>(initialCapacity);
        }

        public List<ContactPair> ComputePairs(List<Rigidbody> rigidbodies)
        {
            pairs.Clear();
            checkedPairs.Clear();

            EnsureCapacity(rigidbodies.Count);

            int endpointCountX = 0;
            int endpointCountY = 0;
            int endpointCountZ = 0;

            PopulateEndpoints(rigidbodies, endpointsX, 0, ref endpointCountX);
            PopulateEndpoints(rigidbodies, endpointsY, 1, ref endpointCountY);
            PopulateEndpoints(rigidbodies, endpointsZ, 2, ref endpointCountZ);

            InsertionSort(endpointsX, endpointCountX);
            InsertionSort(endpointsY, endpointCountY);
            InsertionSort(endpointsZ, endpointCountZ);

            SweepAndPrune(endpointsX, endpointCountX);
            SweepAndPrune(endpointsY, endpointCountY);
            SweepAndPrune(endpointsZ, endpointCountZ);

            return pairs;
        }

        private void EnsureCapacity(int rigidbodyCount)
        {
            int requiredCapacity = rigidbodyCount * 2;
            if (endpointsX.Length < requiredCapacity)
            {
                endpointsX = new AxisEndpoint[requiredCapacity];
                endpointsY = new AxisEndpoint[requiredCapacity];
                endpointsZ = new AxisEndpoint[requiredCapacity];
            }
        }

        private void PopulateEndpoints(List<Rigidbody> rigidbodies, AxisEndpoint[] endpoints, int axis, ref int endpointCount)
        {
            foreach (var body in rigidbodies)
            {
                var bounds = body.collider.GetBounds();
                var min = axis == 0 ? bounds.Min.x : (axis == 1 ? bounds.Min.y : bounds.Min.z);
                var max = axis == 0 ? bounds.Max.x : (axis == 1 ? bounds.Max.y : bounds.Max.z);

                endpoints[endpointCount++] = new AxisEndpoint { Value = min, IsMin = true, Body = body };
                endpoints[endpointCount++] = new AxisEndpoint { Value = max, IsMin = false, Body = body };
            }
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
            var activeList = new List<Rigidbody>();

            for (int i = 0; i < count; i++)
            {
                var endpoint = endpoints[i];
                if (endpoint.IsMin)
                {
                    foreach (var activeBody in activeList)
                    {
                        int idA = endpoint.Body.index;
                        int idB = activeBody.index;
                        if (idA > idB)
                        {
                            (idA, idB) = (idB, idA);
                        }

                        if (checkedPairs.ContainsKey((idA, idB)))
                            continue;

                        if (endpoint.Body.collider.GetBounds().Intersects(activeBody.collider.GetBounds()))
                        {
                            pairs.Add(new ContactPair { bodyA = endpoint.Body, bodyB = activeBody });
                        }

                        checkedPairs[(idA, idB)] = true;
                    }
                    activeList.Add(endpoint.Body);
                }
                else
                {
                    activeList.Remove(endpoint.Body);
                }
            }
        }

        private struct AxisEndpoint
        {
            public sfloat Value;
            public bool IsMin;
            public Rigidbody Body;
        }
    }
}
