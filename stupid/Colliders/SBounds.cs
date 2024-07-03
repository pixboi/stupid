
using SoftFloat;
using stupid.Maths;

namespace stupid.Colliders
{
    /// <summary>
    /// Represents an axis-aligned bounding box defined by minimum and maximum points.
    /// </summary>
    public struct SBounds
    {
        public Vector3S Min;
        public Vector3S Max;

        public SBounds(Vector3S min, Vector3S max)
        {
            Min = min;
            Max = max;
        }

        public bool Intersects(SBounds other)
        {
            if (Max.x < other.Min.x || Min.x > other.Max.x) return false;
            if (Max.y < other.Min.y || Min.y > other.Max.y) return false;
            if (Max.z < other.Min.z || Min.z > other.Max.z) return false;
            return true;
        }

        public void Union(SBounds other)
        {
            Min = new Vector3S(MathS.Min(Min.x, other.Min.x), MathS.Min(Min.y, other.Min.y), MathS.Min(Min.z, other.Min.z));
            Max = new Vector3S(MathS.Max(Max.x, other.Max.x), MathS.Max(Max.y, other.Max.y), MathS.Max(Max.z, other.Max.z));
        }

        public static SBounds Union(SBounds a, SBounds b)
        {
            return new SBounds(
                new Vector3S(MathS.Min(a.Min.x, b.Min.x), MathS.Min(a.Min.y, b.Min.y), MathS.Min(a.Min.z, b.Min.z)),
                new Vector3S(MathS.Max(a.Max.x, b.Max.x), MathS.Max(a.Max.y, b.Max.y), MathS.Max(a.Max.z, b.Max.z))
            );
        }

        public Vector3S Center => (Min + Max) * (sfloat)0.5f;

        public int MaximumExtent()
        {
            Vector3S diag = Max - Min;
            if (diag.x > diag.y && diag.x > diag.z)
                return 0;
            else if (diag.y > diag.z)
                return 1;
            else
                return 2;
        }

        public bool Contains(Vector3S point)
        {
            return point.x >= Min.x && point.x <= Max.x &&
                   point.y >= Min.y && point.y <= Max.y &&
                   point.z >= Min.z && point.z <= Max.z;
        }

        public bool ContainsBounds(SBounds other)
        {
            return Contains(other.Min) && Contains(other.Max);
        }
    }
}
