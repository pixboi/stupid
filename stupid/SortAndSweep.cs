using System.Collections.Generic;
using System.Runtime.CompilerServices;
using SoftFloat;

namespace stupid
{
    public interface IBroadphase
    {
        HashSet<BodyPair> ComputePairs(List<SRigidbody> rigidbodies);
    }

    public class SortAndSweepBroadphase : IBroadphase
    {
        private AxisEndpoint[] endpointsX;
        private AxisEndpoint[] endpointsY;
        private AxisEndpoint[] endpointsZ;
        private readonly HashSet<BodyPair> pairs;
        private int[] overlapCount;
        private int rbCount = 0;
        private SRigidbody[] activeList;
        private int activeListCount;

        public SortAndSweepBroadphase(int initialCapacity = 100)
        {
            endpointsX = new AxisEndpoint[initialCapacity * 2];
            endpointsY = new AxisEndpoint[initialCapacity * 2];
            endpointsZ = new AxisEndpoint[initialCapacity * 2];
            pairs = new HashSet<BodyPair>(initialCapacity * initialCapacity, new BodyPairComparer());
            activeList = new SRigidbody[initialCapacity];
        }

        private void Rebuild(List<SRigidbody> rigidbodies)
        {
            int endpointCapacity = rigidbodies.Count * 2;
            if (endpointsX.Length < endpointCapacity)
            {
                endpointsX = new AxisEndpoint[endpointCapacity];
                endpointsY = new AxisEndpoint[endpointCapacity];
                endpointsZ = new AxisEndpoint[endpointCapacity];
            }

            for (int i = 0; i < rigidbodies.Count; i++)
            {
                var body = rigidbodies[i];
                var bounds = body.collider.GetBounds();
                endpointsX[i * 2] = new AxisEndpoint { Value = bounds.Min.x, IsMin = true, Body = body };
                endpointsX[i * 2 + 1] = new AxisEndpoint { Value = bounds.Max.x, IsMin = false, Body = body };
                endpointsY[i * 2] = new AxisEndpoint { Value = bounds.Min.y, IsMin = true, Body = body };
                endpointsY[i * 2 + 1] = new AxisEndpoint { Value = bounds.Max.y, IsMin = false, Body = body };
                endpointsZ[i * 2] = new AxisEndpoint { Value = bounds.Min.z, IsMin = true, Body = body };
                endpointsZ[i * 2 + 1] = new AxisEndpoint { Value = bounds.Max.z, IsMin = false, Body = body };
            }
            rbCount = rigidbodies.Count;
            overlapCount = new int[rbCount * rbCount];
        }

        public HashSet<BodyPair> ComputePairs(List<SRigidbody> rigidbodies)
        {
            if (rbCount != rigidbodies.Count)
            {
                Rebuild(rigidbodies);
            }

            pairs.Clear();
            System.Array.Clear(overlapCount, 0, overlapCount.Length);

            UpdateEndpoints(endpointsX, 0);
            UpdateEndpoints(endpointsY, 1);
            UpdateEndpoints(endpointsZ, 2);
            InsertionSort(endpointsX, rbCount * 2);
            InsertionSort(endpointsY, rbCount * 2);
            InsertionSort(endpointsZ, rbCount * 2);

            FlagPairsInAxis(endpointsX, rbCount * 2);
            FlagPairsInAxis(endpointsY, rbCount * 2);
            FlagPairsInAxis(endpointsZ, rbCount * 2);

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

        private void FlagPairsInAxis(AxisEndpoint[] endpoints, int count)
        {
            activeListCount = 0;

            for (int i = 0; i < count; i++)
            {
                var me = endpoints[i];
                var aIndex = me.Body.index;

                if (me.IsMin)
                {
                    for (int j = 0; j < activeListCount; j++)
                    {
                        var otherBody = activeList[j];
                        var bIndex = otherBody.index;
                        //var pair = new BodyPair(me.Body.index, otherBody.index);
                        //int index = GetPairIndex(me.Body.index, otherBody.index);

                        if (aIndex < bIndex)
                        {
                            int index = aIndex * rbCount + bIndex;
                            overlapCount[index]++;
                        }
                        else
                        {
                            int index = bIndex * rbCount + aIndex;
                            overlapCount[index]++;
                        }

                        //int index = me.Body.index * rbCount + otherBody.index;

                    }
                    activeList[activeListCount++] = me.Body;
                }
                else
                {
                    for (int j = 0; j < activeListCount; j++)
                    {
                        if (activeList[j] == me.Body)
                        {
                            activeList[j] = activeList[--activeListCount];
                            break;
                        }
                    }
                }
            }
        }

        private void UpdateEndpoints(AxisEndpoint[] endpoints, int axis)
        {
            for (int i = 0; i < rbCount * 2; i++)
            {
                var endpoint = endpoints[i];
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
                endpoints[i] = endpoint; // Ensure the updated endpoint is stored back
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

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private int GetPairIndex(int a, int b)
        {
            return a * rbCount + b;
        }

        private struct AxisEndpoint
        {
            public sfloat Value;
            public bool IsMin;
            public SRigidbody Body;
        }
    }

    public struct BodyPair
    {
        public int aIndex;
        public int bIndex;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public BodyPair(int aIndex, int bIndex)
        {
            bool condition = aIndex < bIndex;
            this.aIndex = condition ? aIndex : bIndex;
            this.bIndex = condition ? bIndex : aIndex;
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
