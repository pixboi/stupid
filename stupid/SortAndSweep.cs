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
        private readonly HashSet<BodyPair> pairs;
        private int rbCount = 0;

        public SortAndSweepBroadphase(int initialCapacity = 100)
        {
            endpointsX = new List<AxisEndpoint>(initialCapacity * 2);
            pairs = new HashSet<BodyPair>(initialCapacity * 2);
        }

        private void Rebuild(List<Rigidbody> rigidbodies)
        {
            endpointsX.Clear();
            foreach (var body in rigidbodies)
            {
                var bounds = body.collider.GetBounds();
                endpointsX.Add(new AxisEndpoint { Value = bounds.Min.x, IsMin = true, Body = body });
                endpointsX.Add(new AxisEndpoint { Value = bounds.Max.x, IsMin = false, Body = body });
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

            UpdateEndpoints(endpointsX);
            InsertionSort(endpointsX);
            SweepAndPrune(endpointsX);

            return pairs;
        }

        private void UpdateEndpoints(List<AxisEndpoint> endpoints)
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

        private void SweepAndPrune(List<AxisEndpoint> endpoints)
        {
            for (int i = 0; i < endpoints.Count; i++)
            {
                var me = endpoints[i];

                if (me.IsMin)
                {
                    //Lets start testing
                    var bounds = me.Body.collider.GetBounds();

                    for (int j = i + 1; j < endpoints.Count; j++)
                    {
                        var other = endpoints[j];

                        //This is my bound, get out
                        if (!other.IsMin && other.Body == me.Body)
                            break;

                        //We're asleep, no contact necessary
                        if (other.Body.isSleeping && me.Body.isSleeping) continue;

                        if (other.IsMin)
                        {
                            var pair = new BodyPair(me.Body.index, other.Body.index);
                            var otherBounds = other.Body.collider.GetBounds();

                            if (bounds.Intersects(otherBounds))
                            {
                                pairs.Add(pair);
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
