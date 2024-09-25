using stupid.Colliders;
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
        private int activeListCount;
        private BoundsIndex[] activeList;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public SortAndSweepBroadphase(int initialCapacity = 100)
        {
            int capacity = initialCapacity * 2;
            endpointsX = new AxisEndpoint[capacity];
            endpointsY = new AxisEndpoint[capacity];
            endpointsZ = new AxisEndpoint[capacity];
            pairs = new HashSet<IntPair>(initialCapacity * initialCapacity, new IntPairComparer());
            activeList = new BoundsIndex[initialCapacity];
            overlapCount = new int[initialCapacity * initialCapacity];
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private void Rebuild(BoundsIndex[] boundsIndices, int boundsLength)
        {
            int endpointCapacity = boundsLength * 2;

            if (endpointsX.Length < endpointCapacity)
            {
                // Expand capacity when needed, reduce unnecessary reallocation
                endpointsX = new AxisEndpoint[endpointCapacity];
                endpointsY = new AxisEndpoint[endpointCapacity];
                endpointsZ = new AxisEndpoint[endpointCapacity];
            }

            for (int i = 0; i < boundsLength; i++)
            {
                ref var boundsIndex = ref boundsIndices[i];
                var bounds = boundsIndex.bounds;

                // Precompute both min and max
                endpointsX[i * 2] = new AxisEndpoint { Value = bounds.min.x, IsMin = true, bodyIndex = boundsIndex.bodyIndex };
                endpointsX[i * 2 + 1] = new AxisEndpoint { Value = bounds.max.x, IsMin = false, bodyIndex = boundsIndex.bodyIndex };
                endpointsY[i * 2] = new AxisEndpoint { Value = bounds.min.y, IsMin = true, bodyIndex = boundsIndex.bodyIndex };
                endpointsY[i * 2 + 1] = new AxisEndpoint { Value = bounds.max.y, IsMin = false, bodyIndex = boundsIndex.bodyIndex };
                endpointsZ[i * 2] = new AxisEndpoint { Value = bounds.min.z, IsMin = true, bodyIndex = boundsIndex.bodyIndex };
                endpointsZ[i * 2 + 1] = new AxisEndpoint { Value = bounds.max.z, IsMin = false, bodyIndex = boundsIndex.bodyIndex };
            }

            rbCount = boundsLength;
            activeList = new BoundsIndex[boundsLength];
            overlapCount = new int[rbCount * rbCount];
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public HashSet<IntPair> ComputePairs(BoundsIndex[] boundsIndices, int boundsLength)
        {
            if (rbCount != boundsLength)
            {
                Rebuild(boundsIndices, boundsLength);
            }

            pairs.Clear();
            Array.Clear(overlapCount, 0, overlapCount.Length);

            // Update the endpoints
            UpdateEndpoints(ref endpointsX, boundsIndices, 0);
            UpdateEndpoints(ref endpointsY, boundsIndices, 1);
            UpdateEndpoints(ref endpointsZ, boundsIndices, 2);

            // Sort the endpoints
            InsertionSort(endpointsX, rbCount * 2);
            InsertionSort(endpointsY, rbCount * 2);
            InsertionSort(endpointsZ, rbCount * 2);

            // Flag overlapping pairs in each axis
            FlagPairsInAxis(endpointsX, rbCount * 2, boundsIndices);
            FlagPairsInAxis(endpointsY, rbCount * 2, boundsIndices);
            FlagPairsInAxis(endpointsZ, rbCount * 2, boundsIndices);

            // Final pass: check 3-axis overlaps
            for (int i = 0; i < overlapCount.Length; i++)
            {
                if (overlapCount[i] == 3)
                {
                    int aIndex = i / rbCount;
                    int bIndex = i % rbCount;

                    var boundsA = boundsIndices[aIndex].bounds;
                    var boundsB = boundsIndices[bIndex].bounds;

                    if (boundsA.Intersects(boundsB))
                    {
                        pairs.Add(new IntPair(aIndex, bIndex));
                    }
                }
            }

            return pairs;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private void FlagPairsInAxis(AxisEndpoint[] endpoints, int count, BoundsIndex[] boundsIndices)
        {
            activeListCount = 0;

            for (int i = 0; i < count; i++)
            {
                ref var me = ref endpoints[i];
                int aIndex = me.bodyIndex;

                if (me.IsMin)
                {
                    for (int j = 0; j < activeListCount; j++)
                    {
                        int bIndex = activeList[j].bodyIndex;

                        if (aIndex < bIndex)
                        {
                            overlapCount[aIndex * rbCount + bIndex]++;
                        }
                        else
                        {
                            overlapCount[bIndex * rbCount + aIndex]++;
                        }
                    }

                    activeList[activeListCount++] = boundsIndices[aIndex];  // Use ref for efficient storage
                }
                else
                {
                    // Remove from activeList
                    for (int j = 0; j < activeListCount; j++)
                    {
                        if (activeList[j].bodyIndex == aIndex)
                        {
                            activeList[j] = activeList[--activeListCount];
                            break;
                        }
                    }
                }
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private void UpdateEndpoints(ref AxisEndpoint[] endpoints, BoundsIndex[] boundsIndices, int axis)
        {
            for (int i = 0; i < rbCount * 2; i++)
            {
                ref var endpoint = ref endpoints[i];
                var bounds = boundsIndices[endpoint.bodyIndex].bounds;

                switch (axis)
                {
                    case 0: endpoint.Value = endpoint.IsMin ? bounds.min.x : bounds.max.x; break;
                    case 1: endpoint.Value = endpoint.IsMin ? bounds.min.y : bounds.max.y; break;
                    case 2: endpoint.Value = endpoint.IsMin ? bounds.min.z : bounds.max.z; break;
                }
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static void InsertionSort(AxisEndpoint[] endpoints, int count)
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
    }

    public struct AxisEndpoint
    {
        public f32 Value;
        public bool IsMin;
        public int bodyIndex;  // Use bodyIndex for referencing
    }

    public readonly struct BoundsIndex
    {
        public readonly BoundsS bounds;
        public readonly int bodyIndex;

        public BoundsIndex(BoundsS bounds, int index)
        {
            this.bounds = bounds;
            this.bodyIndex = index;
        }
    }
}
