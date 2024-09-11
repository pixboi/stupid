using stupid.Maths;

namespace stupid.Colliders
{
    /// <summary>
    /// Represents an axis-aligned bounding box defined by minimum and maximum points.
    /// </summary>
    public struct BoundsS
    {
        public Vector3S min, max;
        public Vector3S center => (min + max) * f32.half;
        public Vector3S size => max - min;
        public Vector3S halfSize => size * f32.half;

        public BoundsS(Vector3S min, Vector3S max)
        {
            this.min = min;
            this.max = max;
        }

        public bool Intersects(BoundsS other)
        {
            if (max.x < other.min.x || min.x > other.max.x) return false;
            if (max.y < other.min.y || min.y > other.max.y) return false;
            if (max.z < other.min.z || min.z > other.max.z) return false;
            return true;
        }

        public bool Intersects(BoundsS other, out Vector3S penetrationDepth)
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

        public void Union(BoundsS other) => this = Union(this, other);
        public static BoundsS Union(BoundsS a, BoundsS b)
        {
            return new BoundsS(
                new Vector3S(MathS.Min(a.min.x, b.min.x), MathS.Min(a.min.y, b.min.y), MathS.Min(a.min.z, b.min.z)),
                new Vector3S(MathS.Max(a.max.x, b.max.x), MathS.Max(a.max.y, b.max.y), MathS.Max(a.max.z, b.max.z))
            );
        }

        public int MaximumExtent()
        {
            Vector3S diag = size;
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

        public bool Contains(BoundsS other)
        {
            return Contains(other.min) && Contains(other.max);
        }

        public f32 GetSurfaceArea()
        {
            f32 x_size = max.x - min.x;
            f32 y_size = max.y - min.y;
            f32 z_size = max.z - min.z;
            return f32.two * ((x_size * y_size) + (x_size * z_size) + (y_size * z_size));
        }

        public f32 GetPerimeter()
        {
            var x_size = max.x - min.x;
            var y_size = max.y - min.y;
            return f32.two * (x_size + y_size);
        }

        public void Expand(f32 amount)
        {
            min -= new Vector3S(amount, amount, amount);
            max += new Vector3S(amount, amount, amount);
        }

        public void Encapsulate(Vector3S point)
        {
            min = Vector3S.Min(min, point);
            max = Vector3S.Max(max, point);
        }

        public void Encapsulate(BoundsS other)
        {
            min = Vector3S.Min(min, other.min);
            max = Vector3S.Max(max, other.max);
        }
    }
}
