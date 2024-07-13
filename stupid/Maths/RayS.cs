using stupid.Colliders;
using System;

namespace stupid.Maths
{
    public readonly struct RayS
    {
        public readonly Vector3S Origin;
        public readonly Vector3S Direction;

        public RayS(Vector3S origin, Vector3S direction)
        {
            Origin = origin;
            Direction = direction;
        }
    }

    public struct RaycastHitS
    {
        public RigidbodyS Rigidbody;
        public f32 Distance;
        public Vector3S Point;
        public Vector3S Normal;

        public RaycastHitS(RigidbodyS rigidbody, f32 distance, Vector3S point, Vector3S normal)
        {
            Rigidbody = rigidbody;
            Distance = distance;
            Point = point;
            Normal = normal;
        }
    }

    public static class RayExtensions
    {
        public static bool Intersects(this RayS ray, BoundsS bounds, out f32 distance, out Vector3S point, out Vector3S normal)
        {
            Vector3S invDir = new Vector3S(f32.one / ray.Direction.x, f32.one / ray.Direction.y, f32.one / ray.Direction.z);

            f32 t1 = (bounds.min.x - ray.Origin.x) * invDir.x;
            f32 t2 = (bounds.max.x - ray.Origin.x) * invDir.x;
            f32 t3 = (bounds.min.y - ray.Origin.y) * invDir.y;
            f32 t4 = (bounds.max.y - ray.Origin.y) * invDir.y;
            f32 t5 = (bounds.min.z - ray.Origin.z) * invDir.z;
            f32 t6 = (bounds.max.z - ray.Origin.z) * invDir.z;

            f32 tMin = MathS.Max(MathS.Max(MathS.Min(t1, t2), MathS.Min(t3, t4)), MathS.Min(t5, t6));
            f32 tMax = MathS.Min(MathS.Min(MathS.Max(t1, t2), MathS.Max(t3, t4)), MathS.Max(t5, t6));

            if (tMin > tMax || tMax < f32.zero)
            {
                distance = f32.maxValue;
                point = new Vector3S();
                normal = new Vector3S();
                return false;
            }

            distance = tMin;

            // Calculate the intersection point
            point = ray.Origin + ray.Direction * distance;

            // Calculate the normal
            if (MathS.Abs(point.x - bounds.min.x) < f32.epsilon) normal = new Vector3S(-f32.one, f32.zero, f32.zero);
            else if (MathS.Abs(point.x - bounds.max.x) < f32.epsilon) normal = new Vector3S(f32.one, f32.zero, f32.zero);
            else if (MathS.Abs(point.y - bounds.min.y) < f32.epsilon) normal = new Vector3S(f32.zero, -f32.one, f32.zero);
            else if (MathS.Abs(point.y - bounds.max.y) < f32.epsilon) normal = new Vector3S(f32.zero, f32.one, f32.zero);
            else if (MathS.Abs(point.z - bounds.min.z) < f32.epsilon) normal = new Vector3S(f32.zero, f32.zero, -f32.one);
            else if (MathS.Abs(point.z - bounds.max.z) < f32.epsilon) normal = new Vector3S(f32.zero, f32.zero, f32.one);
            else normal = Vector3S.zero;

            return true;
        }
    }
}
