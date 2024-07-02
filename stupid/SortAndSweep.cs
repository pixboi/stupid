using System.Collections.Generic;
using SoftFloat;

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
        private int[] overlapCount;
        private int rbCount = 0;

        public SortAndSweepBroadphase(int initialCapacity = 100)
        {
            endpointsX = new List<AxisEndpoint>(initialCapacity * 2);
            endpointsY = new List<AxisEndpoint>(initialCapacity * 2);
            endpointsZ = new List<AxisEndpoint>(initialCapacity * 2);
            pairs = new HashSet<BodyPair>(initialCapacity * initialCapacity, new BodyPairComparer());
        }

        private void Rebuild(List<Rigidbody> rigidbodies)
        {
            endpointsX.Clear();
            endpointsY.Clear();
            endpointsZ.Clear();
            foreach (var body in rigidbodies)
            {
                var bounds = body.collider.GetBounds();
                endpointsX.Add(new AxisEndpoint { Value = bounds.Min.x, IsMin = true, Body = body });
                endpointsX.Add(new AxisEndpoint { Value = bounds.Max.x, IsMin = false, Body = body });
                endpointsY.Add(new AxisEndpoint { Value = bounds.Min.y, IsMin = true, Body = body });
                endpointsY.Add(new AxisEndpoint { Value = bounds.Max.y, IsMin = false, Body = body });
                endpointsZ.Add(new AxisEndpoint { Value = bounds.Min.z, IsMin = true, Body = body });
                endpointsZ.Add(new AxisEndpoint { Value = bounds.Max.z, IsMin = false, Body = body });
            }
            rbCount = rigidbodies.Count;
            overlapCount = new int[rbCount * rbCount];
        }

        public HashSet<BodyPair> ComputePairs(List<Rigidbody> rigidbodies)
        {
            if (rbCount != rigidbodies.Count)
            {
                Rebuild(rigidbodies);
            }

            pairs.Clear();
            System.Array.Clear(overlapCount, 0, overlapCount.Length);

            UpdateEndpoints(endpointsX, 'x');
            UpdateEndpoints(endpointsY, 'y');
            UpdateEndpoints(endpointsZ, 'z');
            InsertionSort(endpointsX);
            InsertionSort(endpointsY);
            InsertionSort(endpointsZ);

            FlagPairsInAxis(endpointsX, 0);
            FlagPairsInAxis(endpointsY, 1);
            FlagPairsInAxis(endpointsZ, 2);

            // Check overlap counts and perform detailed bounds checks
            for (int i = 0; i < overlapCount.Length; i++)
            {
                if (overlapCount[i] == 3)
                {
                    int aIndex = i / rbCount;
                    int bIndex = i % rbCount;

                    var bodyA = rigidbodies[aIndex];
                    var bodyB = rigidbodies[bIndex];

                    if (bodyA.collider.GetBounds().Intersects(bodyB.collider.GetBounds()))
                    {
                        pairs.Add(new BodyPair(aIndex, bIndex));
                    }
                }
            }

            return pairs;
        }

        private void FlagPairsInAxis(List<AxisEndpoint> endpoints, int axisIndex)
        {
            List<Rigidbody> activeList = new List<Rigidbody>();

            for (int i = 0; i < endpoints.Count; i++)
            {
                var me = endpoints[i];

                if (me.IsMin)
                {
                    foreach (var otherBody in activeList)
                    {
                        var pair = new BodyPair(me.Body.index, otherBody.index);
                        int index = GetPairIndex(pair.aIndex, pair.bIndex);
                        overlapCount[index]++;
                    }
                    activeList.Add(me.Body);
                }
                else
                {
                    activeList.Remove(me.Body);
                }
            }
        }


        private void UpdateEndpoints(List<AxisEndpoint> endpoints, char axis)
        {
            foreach (var endpoint in endpoints)
            {
                if (endpoint.Body.isSleeping) continue;

                var bounds = endpoint.Body.collider.GetBounds();
                switch (axis)
                {
                    case 'x':
                        endpoint.Value = endpoint.IsMin ? bounds.Min.x : bounds.Max.x;
                        break;
                    case 'y':
                        endpoint.Value = endpoint.IsMin ? bounds.Min.y : bounds.Max.y;
                        break;
                    case 'z':
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

        private int GetPairIndex(int a, int b)
        {
            return a * rbCount + b;
        }

        private class AxisEndpoint
        {
            public sfloat Value;
            public bool IsMin;
            public Rigidbody Body;
        }
    }

    public struct BodyPair
    {
        public int aIndex;
        public int bIndex;

        public BodyPair(int aIndex, int bIndex)
        {
            if (aIndex < bIndex)
            {
                this.aIndex = aIndex;
                this.bIndex = bIndex;
            }
            else
            {
                this.aIndex = bIndex;
                this.bIndex = aIndex;
            }
        }

        public override bool Equals(object obj)
        {
            return obj is BodyPair pair &&
                   aIndex == pair.aIndex &&
                   bIndex == pair.bIndex;
        }

        public override int GetHashCode()
        {
            unchecked
            {
                int hash = 17;
                hash = hash * 31 + aIndex;
                hash = hash * 31 + bIndex;
                return hash;
            }
        }
    }

    public class BodyPairComparer : IEqualityComparer<BodyPair>
    {
        public bool Equals(BodyPair x, BodyPair y)
        {
            return x.aIndex == y.aIndex && x.bIndex == y.bIndex;
        }

        public int GetHashCode(BodyPair obj)
        {
            return obj.GetHashCode();
        }
    }
}
