using stupid.Maths;
using System;
using System.Collections.Generic;
using System.Runtime.CompilerServices;

namespace stupid.Broadphase
{
    public class SortAndSweepBroadphase
    {
        AxisEndpoint[] endpointsX;
        AxisEndpoint[] endpointsY;
        AxisEndpoint[] endpointsZ;

        private HashSet<IntPair> pairs;
        private int[] overlapCount;
        private int rbCount = 0;
        private Collidable[] activeList;
        private int activeListCount;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public SortAndSweepBroadphase(int initialCapacity = 100)
        {
            endpointsX = new AxisEndpoint[initialCapacity * 2];
            endpointsY = new AxisEndpoint[initialCapacity * 2];
            endpointsZ = new AxisEndpoint[initialCapacity * 2];
            pairs = new HashSet<IntPair>(initialCapacity * initialCapacity, new IntPairComparer());
            activeList = new Collidable[initialCapacity];
            overlapCount = new int[initialCapacity * initialCapacity];
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private void Rebuild(List<Collidable> rigidbodies)
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
                var bounds = body._bounds;
                endpointsX[i * 2] = new AxisEndpoint { Value = bounds.min.x, IsMin = true, Body = body };
                endpointsX[i * 2 + 1] = new AxisEndpoint { Value = bounds.max.x, IsMin = false, Body = body };
                endpointsY[i * 2] = new AxisEndpoint { Value = bounds.min.y, IsMin = true, Body = body };
                endpointsY[i * 2 + 1] = new AxisEndpoint { Value = bounds.max.y, IsMin = false, Body = body };
                endpointsZ[i * 2] = new AxisEndpoint { Value = bounds.min.z, IsMin = true, Body = body };
                endpointsZ[i * 2 + 1] = new AxisEndpoint { Value = bounds.max.z, IsMin = false, Body = body };
            }

            activeList = new Collidable[rigidbodies.Count];
            rbCount = rigidbodies.Count;
            overlapCount = new int[rbCount * rbCount];
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public HashSet<IntPair> ComputePairs(List<Collidable> rigidbodies)
        {
            if (rbCount != rigidbodies.Count)
            {
                Rebuild(rigidbodies);
            }

            pairs.Clear();
            Array.Clear(overlapCount, 0, overlapCount.Length);

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

                    //Skip static + static
                    if (!bodyA.isDynamic && !bodyB.isDynamic) continue;

                    var ab = bodyA._bounds;
                    var bb = bodyB._bounds;

                    if (ab.Intersects(bb))
                    {
                        pairs.Add(new IntPair(aIndex, bIndex));
                    }
                }
            }

            return pairs;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
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

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private void UpdateEndpoints(AxisEndpoint[] endpoints, int axis)
        {
            for (int i = 0; i < rbCount * 2; i++)
            {
                var endpoint = endpoints[i];

                var bounds = endpoint.Body._bounds;
                switch (axis)
                {
                    case 0:
                        endpoint.Value = endpoint.IsMin ? bounds.min.x : bounds.max.x;
                        break;
                    case 1:
                        endpoint.Value = endpoint.IsMin ? bounds.min.y : bounds.max.y;
                        break;
                    case 2:
                        endpoint.Value = endpoint.IsMin ? bounds.min.z : bounds.max.z;
                        break;
                }
                endpoints[i] = endpoint; // Ensure the updated endpoint is stored back
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
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

        public struct AxisEndpoint
        {
            public f32 Value;
            public bool IsMin;
            public Collidable Body;
        }
    }


}
