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
        private readonly List<AxisEndpoint> endpoints;
        private readonly HashSet<BodyPair> pairs;
        private int rbCount = 0;

        public SortAndSweepBroadphase(int initialCapacity = 100)
        {
            endpoints = new List<AxisEndpoint>(initialCapacity * 2);
            pairs = new HashSet<BodyPair>(initialCapacity * 2);
        }

        private void Init(List<Rigidbody> rigidbodies)
        {
            endpoints.Clear();
            foreach (var body in rigidbodies)
            {
                var bounds = body.collider.GetBounds();
                endpoints.Add(new AxisEndpoint { Value = bounds.Min.x, IsMin = true, Body = body });
                endpoints.Add(new AxisEndpoint { Value = bounds.Max.x, IsMin = false, Body = body });
            }
            rbCount = rigidbodies.Count;
        }

        public HashSet<BodyPair> ComputePairs(List<Rigidbody> rigidbodies)
        {
            pairs.Clear();

            if (rbCount != rigidbodies.Count)
            {
                Init(rigidbodies);
            }

            UpdateEndpoints();
            InsertionSort(endpoints);
            SweepAndPrune(endpoints);

            return pairs;
        }

        private void UpdateEndpoints()
        {
            foreach (var endpoint in endpoints)
            {
                if (endpoint.Body.isSleeping) continue;

                var bounds = endpoint.Body.collider.GetBounds();
                endpoint.Value = endpoint.IsMin ? bounds.Min.x : bounds.Max.x;
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

        private bool Intersects(Bounds a, Bounds b)
        {
            return !(a.Max.x <= b.Min.x || a.Min.x >= b.Max.x ||
                     a.Max.y <= b.Min.y || a.Min.y >= b.Max.y ||
                     a.Max.z <= b.Min.z || a.Min.z >= b.Max.z);
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
                            var otherBounds = other.Body.collider.GetBounds();

                            if (Intersects(bounds,otherBounds))
                            {
                                pairs.Add(new BodyPair(me.Body.index, other.Body.index));
                            }
                        }
                    }
                }
            }
        }

        private class AxisEndpoint
        {
            public sfloat Value;
            public bool IsMin;
            public Rigidbody Body;
        }
    }
}