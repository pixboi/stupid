
using stupid.Maths;

namespace stupid.Colliders
{
    /// <summary>
    /// Represents an axis-aligned bounding box defined by minimum and maximum points.
    /// </summary>
    public struct SBounds
    {
        public Vector3S min;
        public Vector3S max;

        public SBounds(Vector3S min, Vector3S max)
        {
            this.min = min;
            this.max = max;
        }

        public bool Intersects(SBounds other)
        {
            if (max.x < other.min.x || min.x > other.max.x) return false;
            if (max.y < other.min.y || min.y > other.max.y) return false;
            if (max.z < other.min.z || min.z > other.max.z) return false;
            return true;
        }

        public static SBounds Union(SBounds a, SBounds b)
        {
            return new SBounds(
                new Vector3S(MathS.Min(a.min.x, b.min.x), MathS.Min(a.min.y, b.min.y), MathS.Min(a.min.z, b.min.z)),
                new Vector3S(MathS.Max(a.max.x, b.max.x), MathS.Max(a.max.y, b.max.y), MathS.Max(a.max.z, b.max.z))
            );
        }

        public void Union(SBounds other) => this = Union(this, other);

        public Vector3S Center => (min + max) * f32.half;
        public Vector3S Size => max - min;

        public int MaximumExtent()
        {
            Vector3S diag = Size;
            if (diag.x > diag.y && diag.x > diag.z)
                return 0;
            else if (diag.y > diag.z)
                return 1;
            else
                return 2;
        }

        public bool Contains(Vector3S point)
        {
            return point.x >= min.x && point.x <= max.x &&
                   point.y >= min.y && point.y <= max.y &&
                   point.z >= min.z && point.z <= max.z;
        }

        public bool ContainsBounds(SBounds other)
        {
            return Contains(other.min) && Contains(other.max);
        }
    }
}
