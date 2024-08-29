using stupid.Maths;
using System;
using System.Collections.Generic;
using System.Runtime.CompilerServices;

namespace stupid.Colliders
{
    public static partial class CollisionMath
    {
        public static int BoxVsBox(in BoxColliderS a, in BoxColliderS b, ref ContactS[] contacts)
        {
            Vector3S relativePosition = b.collidable.transform.position - a.collidable.transform.position;

            f32 minPen = f32.maxValue;
            Vector3S minAxis = Vector3S.zero;
            int best = -1;

            // Check for overlaps on the primary axes of both boxes
            //These are normalized on UpdateBox()
            if (!TryAxis(relativePosition, a.axes[0], a, b, 0, ref minPen, ref minAxis, ref best)) return 0;
            if (!TryAxis(relativePosition, a.axes[1], a, b, 1, ref minPen, ref minAxis, ref best)) return 0;
            if (!TryAxis(relativePosition, a.axes[2], a, b, 2, ref minPen, ref minAxis, ref best)) return 0;

            if (!TryAxis(relativePosition, b.axes[0], a, b, 3, ref minPen, ref minAxis, ref best)) return 0;
            if (!TryAxis(relativePosition, b.axes[1], a, b, 4, ref minPen, ref minAxis, ref best)) return 0;
            if (!TryAxis(relativePosition, b.axes[2], a, b, 5, ref minPen, ref minAxis, ref best)) return 0;

            // Check for overlaps on the cross product of axes pairs
            //Need normalizations
            if (!TryAxis(relativePosition, Vector3S.Cross(a.axes[0], b.axes[0]).NormalizeInPlace(), a, b, 6, ref minPen, ref minAxis, ref best)) return 0;
            if (!TryAxis(relativePosition, Vector3S.Cross(a.axes[0], b.axes[1]).NormalizeInPlace(), a, b, 7, ref minPen, ref minAxis, ref best)) return 0;
            if (!TryAxis(relativePosition, Vector3S.Cross(a.axes[0], b.axes[2]).NormalizeInPlace(), a, b, 8, ref minPen, ref minAxis, ref best)) return 0;

            if (!TryAxis(relativePosition, Vector3S.Cross(a.axes[1], b.axes[0]).NormalizeInPlace(), a, b, 9, ref minPen, ref minAxis, ref best)) return 0;
            if (!TryAxis(relativePosition, Vector3S.Cross(a.axes[1], b.axes[1]).NormalizeInPlace(), a, b, 10, ref minPen, ref minAxis, ref best)) return 0;
            if (!TryAxis(relativePosition, Vector3S.Cross(a.axes[1], b.axes[2]).NormalizeInPlace(), a, b, 11, ref minPen, ref minAxis, ref best)) return 0;

            if (!TryAxis(relativePosition, Vector3S.Cross(a.axes[2], b.axes[0]).NormalizeInPlace(), a, b, 12, ref minPen, ref minAxis, ref best)) return 0;
            if (!TryAxis(relativePosition, Vector3S.Cross(a.axes[2], b.axes[1]).NormalizeInPlace(), a, b, 13, ref minPen, ref minAxis, ref best)) return 0;
            if (!TryAxis(relativePosition, Vector3S.Cross(a.axes[2], b.axes[2]).NormalizeInPlace(), a, b, 14, ref minPen, ref minAxis, ref best)) return 0;

            if (best == -1)
            {
                throw new Exception("OOBB collision error");
            }

            var normalV = minAxis.Normalize();
            if (Vector3S.Dot(normalV, relativePosition) > f32.zero) normalV = -normalV;

            int count = 0;

            //A vert on b
            if (GetContactPoint(a, b))
            {
                foreach (var p in pointCache)
                {
                    var vertex = p.Item1;
                    var feature = p.Item2;
                    var pen = minPen;

                    if (b.RayTest(vertex, normalV, minPen, out var pointInBox, out var distance))
                    {
                        pen = distance;
                    }

                    contacts[count++] = new ContactS(vertex, normalV, -pen, a.collidable, b.collidable, feature);
                }
            }


            //B vert on a
            if (count == 0)
            {
                if (GetContactPoint(b, a))
                {
                    foreach (var p in pointCache)
                    {
                        var vertex = p.Item1;
                        var feature = p.Item2;
                        var pen = minPen;

                        if (a.RayTest(vertex, -normalV, minPen, out var pointInBox, out var distance))
                        {
                            pen = distance;
                        }

                        contacts[count++] = new ContactS(vertex, normalV, -pen, a.collidable, b.collidable, feature);
                    }
                }
            }


            return count;

            /*
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

            a.GetAllEdges(ref edgeCache);
            foreach (var e in edgeCache)
            {
                CheckEdgeIntersections(e, b);
            }

            if (intersectionCount == 0)
            {
                b.GetAllEdges(ref edgeCache);
                foreach (var e in edgeCache)
                {
                    CheckEdgeIntersections(e, a);
                }
            }

            if (intersectionCount == 0) return 0;

            // Average of intersection points
            contacts.point = totalIntersectionPoint / (f32)intersectionCount;
            var normal = minAxis.Normalize();
            if (Vector3S.Dot(normal, relativePosition) > f32.zero) normal = -normal;

            contacts.normal = normal;
            contacts.penetrationDepth = minPen;


            return 1;
            */
        }

        static List<(Vector3S, int)> pointCache = new List<(Vector3S, int)>();


        //A vertex on B
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool GetContactPoint(in BoxColliderS a, in BoxColliderS b)
        {
            pointCache.Clear();

            for (int i = 0; i < 8; i++)
            {
                var v = a.vertices[i];
                if (b.ContainsPoint(v))
                {
                    pointCache.Add((v, i));
                }
            }

            if (pointCache.Count == 0) return false;
            return true;
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static bool TryAxis(in Vector3S relativePosition, in Vector3S axis, in BoxColliderS a, in BoxColliderS b, int index, ref f32 minOverlap, ref Vector3S minAxis, ref int best)
        {
            if (axis.sqrMagnitude < f32.epsilon) return true; // Skip zero-length axes

            // Calculate projection and overlap
            f32 pA = ProjectBox(axis, a);
            f32 pB = ProjectBox(axis, b);
            f32 distance = Vector3S.AbsDot(relativePosition, axis);

            pA.AddInPlace(pB);
            pA.SubtractInPlace(distance);

            f32 overlap = pA;
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
        private static f32 ProjectBox(in Vector3S axis, in BoxColliderS box)
        {
            long absDot0 = Vector3S.RawAbsDot(axis, box.axes[0]);
            long absDot1 = Vector3S.RawAbsDot(axis, box.axes[1]);
            long absDot2 = Vector3S.RawAbsDot(axis, box.axes[2]);
            var projection = ((box.halfSize.x.rawValue * absDot0) + (box.halfSize.y.rawValue * absDot1) + (box.halfSize.z.rawValue * absDot2));

            return new f32(projection >> f32.FractionalBits);
        }

    }
}
