using stupid.Maths;
using System.Numerics;

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

        //Expect direction to be like, with the max length
        public static BoundsS ConvertFromRay(in Vector3S origin, in Vector3S normalizedDirection, in f32 length)
        {
            Vector3S end = origin + normalizedDirection * length;

            // Calculate the min and max bounds that fully enclose the ray's path
            Vector3S min = Vector3S.Min(origin, end);
            Vector3S max = Vector3S.Max(origin, end);

            return new BoundsS(min, max);
        }

        public bool RayTest(in Vector3S rayOrigin, in Vector3S rayDirection)
        {
            f32 tmin = f32.minValue;
            f32 tmax = f32.maxValue;

            // Check X axis
            if (MathS.Abs(rayDirection.x) > f32.epsilon)
            {
                var tx1 = (min.x - rayOrigin.x) / rayDirection.x;
                var tx2 = (max.x - rayOrigin.x) / rayDirection.x;
                tmin = MathS.Max(tmin, MathS.Min(tx1, tx2));
                tmax = MathS.Min(tmax, MathS.Max(tx1, tx2));
            }
            else if (rayOrigin.x < min.x || rayOrigin.x > max.x)
            {
                return false; // Parallel to X-axis and outside bounds
            }

            // Check Y axis
            if (MathS.Abs(rayDirection.y) > f32.epsilon)
            {
                var ty1 = (min.y - rayOrigin.y) / rayDirection.y;
                var ty2 = (max.y - rayOrigin.y) / rayDirection.y;
                tmin = MathS.Max(tmin, MathS.Min(ty1, ty2));
                tmax = MathS.Min(tmax, MathS.Max(ty1, ty2));
            }
            else if (rayOrigin.y < min.y || rayOrigin.y > max.y)
            {
                return false; // Parallel to Y-axis and outside bounds
            }

            // Check Z axis
            if (MathS.Abs(rayDirection.z) > f32.epsilon)
            {
                var tz1 = (min.z - rayOrigin.z) / rayDirection.z;
                var tz2 = (max.z - rayOrigin.z) / rayDirection.z;
                tmin = MathS.Max(tmin, MathS.Min(tz1, tz2));
                tmax = MathS.Min(tmax, MathS.Max(tz1, tz2));
            }
            else if (rayOrigin.z < min.z || rayOrigin.z > max.z)
            {
                return false; // Parallel to Z-axis and outside bounds
            }

            // Check if the intersection exists in positive ray direction
            return tmax >= MathS.Max(tmin, f32.zero);
        }



        public bool Contains(in BoundsS other) => Contains(other.min) && Contains(other.max);

    }
}
