using SoftFloat;
using System.Collections.Generic;

namespace stupid
{
    public interface IBroadphase
    {
        HashSet<BodyPair> ComputePairs(List<Rigidbody> rigidbodies);
    }

    public class SortAndSweepBroadphase : IBroadphase
    {
        private readonly List<AxisEndpoint> endpointsX;
        private readonly List<AxisEndpoint> endpointsY;
        private readonly List<AxisEndpoint> endpointsZ;
        private readonly HashSet<BodyPair> pairs;
        private int rbCount = 0;

        public SortAndSweepBroadphase(int initialCapacity = 100)
        {
            endpointsX = new List<AxisEndpoint>(initialCapacity * 2);
            //  endpointsY = new List<AxisEndpoint>(initialCapacity * 2);
            //  endpointsZ = new List<AxisEndpoint>(initialCapacity * 2);
            pairs = new HashSet<BodyPair>(initialCapacity * 2);
        }

        private void Rebuild(List<Rigidbody> rigidbodies)
        {
            endpointsX.Clear();
            //  endpointsY.Clear();
            //  endpointsZ.Clear();
            foreach (var body in rigidbodies)
            {
                var bounds = body.collider.GetBounds();
                endpointsX.Add(new AxisEndpoint { Value = bounds.Min.x, IsMin = true, Body = body });
                endpointsX.Add(new AxisEndpoint { Value = bounds.Max.x, IsMin = false, Body = body });
                //  endpointsY.Add(new AxisEndpoint { Value = bounds.Min.y, IsMin = true, Body = body });
                // endpointsY.Add(new AxisEndpoint { Value = bounds.Max.y, IsMin = false, Body = body });
                // endpointsZ.Add(new AxisEndpoint { Value = bounds.Min.z, IsMin = true, Body = body });
                // endpointsZ.Add(new AxisEndpoint { Value = bounds.Max.z, IsMin = false, Body = body });
            }
            rbCount = rigidbodies.Count;
        }

        public HashSet<BodyPair> ComputePairs(List<Rigidbody> rigidbodies)
        {
            if (rbCount != rigidbodies.Count)
            {
                Rebuild(rigidbodies);
            }

            pairs.Clear();

            UpdateEndpoints(endpointsX, 0);
            // UpdateEndpoints(endpointsY, 1);
            // UpdateEndpoints(endpointsZ, 2);

            InsertionSort(endpointsX);
            // InsertionSort(endpointsY);
            // InsertionSort(endpointsZ);
            //
            SweepAndPrune(endpointsX);
            // SweepAndPrune(endpointsY);
            // SweepAndPrune(endpointsZ);

            return pairs;
        }

        private void UpdateEndpoints(List<AxisEndpoint> endpoints, int axis)
        {
            foreach (var endpoint in endpoints)
            {
                if (endpoint.Body.isSleeping) continue;

                var bounds = endpoint.Body.collider.GetBounds();
                switch (axis)
                {
                    case 0:
                        endpoint.Value = endpoint.IsMin ? bounds.Min.x : bounds.Max.x;
                        break;
                    case 1:
                        endpoint.Value = endpoint.IsMin ? bounds.Min.y : bounds.Max.y;
                        break;
                    case 2:
                        endpoint.Value = endpoint.IsMin ? bounds.Min.z : bounds.Max.z;
                        break;
                }
            }
        }

        private void InsertionSort(List<AxisEndpoint> endpoints)
        {
            for (int i = 1; i < endpoints.Count; i++)
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

        private void SweepAndPrune(List<AxisEndpoint> endpoints)
        {
            for (int i = 0; i < endpoints.Count; i++)
            {
                var me = endpoints[i];

                if (me.IsMin)
                {
                    var bounds = me.Body.collider.GetBounds();

                    for (int j = i + 1; j < endpoints.Count; j++)
                    {
                        var other = endpoints[j];

                        if (!other.IsMin && other.Body == me.Body)
                            break;

                        if (other.IsMin)
                        {
                            var pair = new BodyPair(me.Body.index, other.Body.index);

                            if (pairs.Contains(pair)) continue;

                            var otherBounds = other.Body.collider.GetBounds();

                            if (Intersects(bounds, otherBounds))
                            {
                                pairs.Add(pair);
                            }
                        }
                    }
                }
            }
        }

        private bool Intersects(Bounds a, Bounds b)
        {
            return !(a.Max.x <= b.Min.x || a.Min.x >= b.Max.x ||
                     a.Max.y <= b.Min.y || a.Min.y >= b.Max.y ||
                     a.Max.z <= b.Min.z || a.Min.z >= b.Max.z);
        }

        private class AxisEndpoint
        {
            public sfloat Value;
            public bool IsMin;
            public Rigidbody Body;

        }
    }
}
