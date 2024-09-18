using stupid.Maths;

namespace stupid.Colliders
{
    /// <summary>
    /// Represents an axis-aligned bounding box defined by minimum and maximum points.
    /// </summary>
    /// 
    public readonly struct BoundsS
    {
        public readonly Vector3S min, max;
        public Vector3S center => (min + max) * f32.half;
        public Vector3S size => max - min;
        public Vector3S halfSize => size * f32.half;

        public BoundsS(in Vector3S min, in Vector3S max)
        {
            this.min = min;
            this.max = max;
        }

        public bool Intersects(in BoundsS other)
        {
            if (max.x < other.min.x || min.x > other.max.x) return false;
            if (max.y < other.min.y || min.y > other.max.y) return false;
            if (max.z < other.min.z || min.z > other.max.z) return false;
            return true;
        }

        public bool Intersects(in BoundsS other, out Vector3S penetrationDepth)
        {
            penetrationDepth = Vector3S.zero;

            if (max.x < other.min.x || min.x > other.max.x) return false;
            if (max.y < other.min.y || min.y > other.max.y) return false;
            if (max.z < other.min.z || min.z > other.max.z) return false;

            // Calculate the penetration depth on each axis
            var x = (max.x < other.max.x) ? max.x - other.min.x : other.max.x - min.x;
            var y = (max.y < other.max.y) ? max.y - other.min.y : other.max.y - min.y;
            var z = (max.z < other.max.z) ? max.z - other.min.z : other.max.z - min.z;

            penetrationDepth = new Vector3S(x, y, z);
            return true;
        }

        public static BoundsS Union(in BoundsS a, in BoundsS b)
        {
            return new BoundsS(
                new Vector3S(MathS.Min(a.min.x, b.min.x), MathS.Min(a.min.y, b.min.y), MathS.Min(a.min.z, b.min.z)),
                new Vector3S(MathS.Max(a.max.x, b.max.x), MathS.Max(a.max.y, b.max.y), MathS.Max(a.max.z, b.max.z))
            );
        }

        public bool Contains(in Vector3S point)
        {
            return point.x >= min.x && point.x <= max.x &&
                   point.y >= min.y && point.y <= max.y &&
                   point.z >= min.z && point.z <= max.z;
        }

        public bool Contains(in BoundsS other) => Contains(other.min) && Contains(other.max);

    }
}
