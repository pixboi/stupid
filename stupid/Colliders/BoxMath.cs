using stupid.Maths;
using System.Collections.Generic;
using System.Runtime.CompilerServices;

namespace stupid.Colliders
{
    public static partial class CollisionMath
    {
        public static int BoxVsBox(BoxColliderS a, BoxColliderS b, ref ContactS contact)
        {
            Vector3S relativePosition = b.collidable.transform.position - a.collidable.transform.position;

            f32 minPen = f32.maxValue;
            Vector3S minAxis = Vector3S.zero;
            int best = -1;

            // Check for overlaps on the primary axes of both boxes
            if (!TryAxis(relativePosition, a.axes[0], a, b, 0, ref minPen, ref minAxis, ref best)) return 0;
            if (!TryAxis(relativePosition, a.axes[1], a, b, 1, ref minPen, ref minAxis, ref best)) return 0;
            if (!TryAxis(relativePosition, a.axes[2], a, b, 2, ref minPen, ref minAxis, ref best)) return 0;

            if (!TryAxis(relativePosition, b.axes[0], a, b, 3, ref minPen, ref minAxis, ref best)) return 0;
            if (!TryAxis(relativePosition, b.axes[1], a, b, 4, ref minPen, ref minAxis, ref best)) return 0;
            if (!TryAxis(relativePosition, b.axes[2], a, b, 5, ref minPen, ref minAxis, ref best)) return 0;

            int bestSingleAxis = best;

            // Check for overlaps on the cross product of axes pairs
            if (!TryAxis(relativePosition, Vector3S.Cross(a.axes[0], b.axes[0]), a, b, 6, ref minPen, ref minAxis, ref best)) return 0;
            if (!TryAxis(relativePosition, Vector3S.Cross(a.axes[0], b.axes[1]), a, b, 7, ref minPen, ref minAxis, ref best)) return 0;
            if (!TryAxis(relativePosition, Vector3S.Cross(a.axes[0], b.axes[2]), a, b, 8, ref minPen, ref minAxis, ref best)) return 0;

            if (!TryAxis(relativePosition, Vector3S.Cross(a.axes[1], b.axes[0]), a, b, 9, ref minPen, ref minAxis, ref best)) return 0;
            if (!TryAxis(relativePosition, Vector3S.Cross(a.axes[1], b.axes[1]), a, b, 10, ref minPen, ref minAxis, ref best)) return 0;
            if (!TryAxis(relativePosition, Vector3S.Cross(a.axes[1], b.axes[2]), a, b, 11, ref minPen, ref minAxis, ref best)) return 0;

            if (!TryAxis(relativePosition, Vector3S.Cross(a.axes[2], b.axes[0]), a, b, 12, ref minPen, ref minAxis, ref best)) return 0;
            if (!TryAxis(relativePosition, Vector3S.Cross(a.axes[2], b.axes[1]), a, b, 13, ref minPen, ref minAxis, ref best)) return 0;
            if (!TryAxis(relativePosition, Vector3S.Cross(a.axes[2], b.axes[2]), a, b, 14, ref minPen, ref minAxis, ref best)) return 0;

            if (best == -1)
            {
                throw new System.Exception("OOBB collision error");
            }

            var normal = minAxis.Normalize();
            if (Vector3S.Dot(normal, relativePosition) > f32.zero) normal = -normal;

            contact.normal = normal;
            contact.penetrationDepth = minPen;

            if (GetContactPoint(a, b, out var point))
            {
                contact.point = point;
            }
            else
            {
                //just use the average transform position?
                contact.point = (a.collidable.transform.position + b.collidable.transform.position) * f32.half;
            }

            return 1;
        }

        static List<Vector3S> points = new List<Vector3S>();
        public static bool GetContactPoint(BoxColliderS a, BoxColliderS b, out Vector3S averagePoint)
        {
            points.Clear();
            averagePoint = Vector3S.zero;

            foreach (var v in a.vertices)
            {
                if (b.ContainsPoint(v))
                    points.Add(v);
            }

            foreach (var v in b.vertices)
            {
                if (a.ContainsPoint(v))
                    points.Add(v);
            }

            if (points.Count == 0) return false;

            foreach (var p in points) averagePoint += p;

            averagePoint = averagePoint / (f32)points.Count;
            return true;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static bool TryAxis(Vector3S relativePosition, Vector3S axis, BoxColliderS a, BoxColliderS b, int index, ref f32 minOverlap, ref Vector3S minAxis, ref int best)
        {
            if (axis.sqrMagnitude < f32.epsilon) return true; // Skip zero-length axes
            axis = axis.Normalize();

            f32 pA = ProjectBox(axis, a);
            f32 pB = ProjectBox(axis, b);
            f32 distance = MathS.Abs(Vector3S.Dot(relativePosition, axis));
            f32 overlap = pA + pB - distance;
            if (overlap < f32.zero) return false;

            if (overlap < minOverlap)
            {
                minOverlap = overlap;
                minAxis = axis;
                best = index;
            }

            return true;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static f32 ProjectBox(Vector3S axis, BoxColliderS box)
        {
            return
                box.halfSize.x * MathS.Abs(Vector3S.Dot(axis, box.axes[0])) +
                box.halfSize.y * MathS.Abs(Vector3S.Dot(axis, box.axes[1])) +
                box.halfSize.z * MathS.Abs(Vector3S.Dot(axis, box.axes[2]));
        }
    }
}
