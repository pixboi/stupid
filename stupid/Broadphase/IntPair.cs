using System;
using System.Collections.Generic;

namespace stupid.Broadphase
{
    public readonly struct IntPair : IEquatable<IntPair>
    {
        public readonly int aIndex;
        public readonly int bIndex;

        public IntPair(int aIndex, int bIndex)
        {
            bool condition = aIndex < bIndex;
            this.aIndex = condition ? aIndex : bIndex;
            this.bIndex = condition ? bIndex : aIndex;
        }

        public override bool Equals(object? obj)
        {
            return obj is IntPair pair && Equals(pair);
        }

        public bool Equals(IntPair other)
        {
            return aIndex == other.aIndex &&
                   bIndex == other.bIndex;
        }

        public override int GetHashCode()
        {
            return HashCode.Combine(aIndex, bIndex);
        }
    }

    public class IntPairComparer : IEqualityComparer<IntPair>
    {
        public bool Equals(IntPair x, IntPair y)
        {
            return x.aIndex == y.aIndex && x.bIndex == y.bIndex;
        }

        public int GetHashCode(IntPair obj)
        {
            return obj.GetHashCode();
        }
    }
}
