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

        public bool Contains(SBounds other)
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

        /// <summary>
        /// Expands the bounds by the given amount in all directions.
        /// </summary>
        /// <param name="amount">The amount to expand the bounds by.</param>
        public void Expand(f32 amount)
        {
            min -= new Vector3S(amount, amount, amount);
            max += new Vector3S(amount, amount, amount);
        }

        /// <summary>
        /// Expands the bounds to encapsulate a given point.
        /// </summary>
        /// <param name="point">The point to encapsulate.</param>
        public void Encapsulate(Vector3S point)
        {
            min = Vector3S.Min(min, point);
            max = Vector3S.Max(max, point);
        }

        /// <summary>
        /// Expands the bounds to encapsulate another bounding box.
        /// </summary>
        /// <param name="other">The bounding box to encapsulate.</param>
        public void Encapsulate(SBounds other)
        {
            min = Vector3S.Min(min, other.min);
            max = Vector3S.Max(max, other.max);
        }

        /// <summary>
        /// Checks if a given ray intersects the bounding box.
        /// </summary>
        /// <param name="ray">The ray to test for intersection.</param>
        /// <returns>True if the ray intersects the bounding box; otherwise, false.</returns>
        public bool IntersectRay(Ray ray)
        {
            f32 tmin = (min.x - ray.Origin.x) / ray.Direction.x;
            f32 tmax = (max.x - ray.Origin.x) / ray.Direction.x;

            if (tmin > tmax)
            {
                f32 temp = tmin;
                tmin = tmax;
                tmax = temp;
            }

            f32 tymin = (min.y - ray.Origin.y) / ray.Direction.y;
            f32 tymax = (max.y - ray.Origin.y) / ray.Direction.y;

            if (tymin > tymax)
            {
                f32 temp = tymin;
                tymin = tymax;
                tymax = temp;
            }

            if ((tmin > tymax) || (tymin > tmax))
            {
                return false;
            }

            if (tymin > tmin)
            {
                tmin = tymin;
            }

            if (tymax < tmax)
            {
                tmax = tymax;
            }

            f32 tzmin = (min.z - ray.Origin.z) / ray.Direction.z;
            f32 tzmax = (max.z - ray.Origin.z) / ray.Direction.z;

            if (tzmin > tzmax)
            {
                f32 temp = tzmin;
                tzmin = tzmax;
                tzmax = temp;
            }

            if ((tmin > tzmax) || (tzmin > tmax))
            {
                return false;
            }

            return true;
        }
    }
}
