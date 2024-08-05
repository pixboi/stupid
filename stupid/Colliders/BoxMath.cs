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
                // Case where there is a collision but none of the vertices overlap each box.
                var bestEdgeA = FindBestEdge(a, -normal);
                var bestEdgeB = FindBestEdge(b, normal);

                var totalIntersectionPoint = Vector3S.zero;
                int intersectionCount = 0;

                void CheckEdgeIntersections((Vector3S, Vector3S, Vector3S) edge, BoxColliderS other)
                {
                    var edgeDirection = (edge.Item2 - edge.Item1).NormalizeWithMagnitude(out var edgeMagnitude);

                    // Check intersection with the first point of the edge
                    if (other.RayTest(edge.Item1, edgeDirection, edgeMagnitude, out var intersectionPoint1))
                    {
                        totalIntersectionPoint += intersectionPoint1;
                        intersectionCount++;
                    }

                    // Check intersection with the second point of the edge
                    if (other.RayTest(edge.Item2, -edgeDirection, edgeMagnitude, out var intersectionPoint2))
                    {
                        totalIntersectionPoint += intersectionPoint2;
                        intersectionCount++;
                    }
                }

                CheckEdgeIntersections(bestEdgeA, b);
                CheckEdgeIntersections(bestEdgeB, a);

                if (intersectionCount == 0)
                {
                    // No intersections found, use intersection points from the centers
                    var pointA = a.GetIntersectionPointFromLocalCenter(-normal);
                    var pointB = b.GetIntersectionPointFromLocalCenter(normal);
                    contact.point = (pointA + pointB) * f32.half;
                }
                else
                {
                    // Average of intersection points
                    contact.point = totalIntersectionPoint / (f32)intersectionCount;
                }


            }

            return 1;
        }

        static (Vector3S, Vector3S, Vector3S) FindBestEdge(BoxColliderS shape, Vector3S normal)
        {
            var edges = new (Vector3S, Vector3S, Vector3S)[12];
            shape.GetAllEdges(ref edges);

            var minDotProduct = f32.minValue;
            int bestEdgeIndex = -1;

            for (int i = 0; i < edges.Length; i++)
            {
                var edgeNormal = shape.collidable.transform.TransformDirection(edges[i].Item3);
                var dotProduct = Vector3S.Dot(normal, edgeNormal);
                if (dotProduct > minDotProduct)
                {
                    minDotProduct = dotProduct;
                    bestEdgeIndex = i;
                }
            }

            return edges[bestEdgeIndex];
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
