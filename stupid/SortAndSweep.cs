using SoftFloat;
using System.Collections.Generic;

namespace stupid
{
    public class SortAndSweepBroadphase : IBroadphase
    {
        private List<AxisEndpoint> endpoints;
        private List<ContactPair> pairs;
        private List<Rigidbody> activeList;
        private int lastBodyCount = 0;

        public SortAndSweepBroadphase(int initialCapacity = 100)
        {
            endpoints = new List<AxisEndpoint>(initialCapacity * 2);
            pairs = new List<ContactPair>(initialCapacity * 2);
            activeList = new List<Rigidbody>(initialCapacity);
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
        }

        private void UpdateEndpoints()
        {
            foreach (var endpoint in endpoints)
            {
                var bounds = endpoint.Body.collider.GetBounds();
                endpoint.Value = endpoint.IsMin ? bounds.Min.x : bounds.Max.x;
            }
        }

        public List<ContactPair> ComputePairs(List<Rigidbody> rigidbodies)
        {
            pairs.Clear();
            activeList.Clear();

            if (lastBodyCount != rigidbodies.Count)
            {
                Init(rigidbodies);
                lastBodyCount = rigidbodies.Count;
            }

            UpdateEndpoints();
            InsertionSort(endpoints);
            SweepAndPrune(endpoints);

            return pairs;
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
                var endpoint = endpoints[i];
                if (endpoint.IsMin)
                {
                    foreach (var activeBody in activeList)
                    {
                        if (activeBody.collider.GetBounds().Max.x < endpoint.Value)
                            break;

                        if (endpoint.Body != activeBody && endpoint.Body.collider.GetBounds().Intersects(activeBody.collider.GetBounds()))
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

        private class AxisEndpoint
        {
            public sfloat Value;
            public bool IsMin;
            public Rigidbody Body;
        }
    }
}