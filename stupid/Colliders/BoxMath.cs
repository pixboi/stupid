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

            long minPenRaw = long.MaxValue;
            Vector3S minAxis = Vector3S.zero;
            int best = -1;

            // Check for overlaps on the primary axes of both boxes
            if (!TryAxis(relativePosition, a.rightAxis, a, b, 0, ref minPenRaw, ref minAxis, ref best)) return 0;
            if (!TryAxis(relativePosition, a.upAxis, a, b, 1, ref minPenRaw, ref minAxis, ref best)) return 0;
            if (!TryAxis(relativePosition, a.forwardAxis, a, b, 2, ref minPenRaw, ref minAxis, ref best)) return 0;

            if (!TryAxis(relativePosition, b.rightAxis, a, b, 3, ref minPenRaw, ref minAxis, ref best)) return 0;
            if (!TryAxis(relativePosition, b.upAxis, a, b, 4, ref minPenRaw, ref minAxis, ref best)) return 0;
            if (!TryAxis(relativePosition, b.forwardAxis, a, b, 5, ref minPenRaw, ref minAxis, ref best)) return 0;

            int bestSingle = best;

            // Check for overlaps on the cross product of axes pairs
            //Need normalizations

            if (!TryAxis(relativePosition, Vector3S.Cross(a.rightAxis, b.rightAxis).NormalizeInPlace(), a, b, 6, ref minPenRaw, ref minAxis, ref best)) return 0; //0,0
            if (!TryAxis(relativePosition, Vector3S.Cross(a.rightAxis, b.upAxis).NormalizeInPlace(), a, b, 7, ref minPenRaw, ref minAxis, ref best)) return 0; //0,1
            if (!TryAxis(relativePosition, Vector3S.Cross(a.rightAxis, b.forwardAxis).NormalizeInPlace(), a, b, 8, ref minPenRaw, ref minAxis, ref best)) return 0; //0,2

            if (!TryAxis(relativePosition, Vector3S.Cross(a.upAxis, b.rightAxis).NormalizeInPlace(), a, b, 9, ref minPenRaw, ref minAxis, ref best)) return 0; //1,0
            if (!TryAxis(relativePosition, Vector3S.Cross(a.upAxis, b.upAxis).NormalizeInPlace(), a, b, 10, ref minPenRaw, ref minAxis, ref best)) return 0; //1,1
            if (!TryAxis(relativePosition, Vector3S.Cross(a.upAxis, b.forwardAxis).NormalizeInPlace(), a, b, 11, ref minPenRaw, ref minAxis, ref best)) return 0; //1,2

            if (!TryAxis(relativePosition, Vector3S.Cross(a.forwardAxis, b.rightAxis).NormalizeInPlace(), a, b, 12, ref minPenRaw, ref minAxis, ref best)) return 0; //2,0
            if (!TryAxis(relativePosition, Vector3S.Cross(a.forwardAxis, b.upAxis).NormalizeInPlace(), a, b, 13, ref minPenRaw, ref minAxis, ref best)) return 0; //2,1
            if (!TryAxis(relativePosition, Vector3S.Cross(a.forwardAxis, b.forwardAxis).NormalizeInPlace(), a, b, 14, ref minPenRaw, ref minAxis, ref best)) return 0; //2,2

            if (best == -1)
            {
                throw new Exception("OOBB collision error");
            }

            var normalV = minAxis.Normalize();
            if (Vector3S.Dot(normalV, relativePosition) < f32.zero) normalV = -normalV;

            int count = 0;
            f32 minPen = new f32(minPenRaw);
            minPen = -MathS.Abs(minPen);

            //A vert on b
            if (GetContactPoint(a, b))
            {
                foreach (var p in pointCache)
                {
                    var vertex = p.Item1;
                    var feature = p.Item2;
                    var pen = minPen;

                    // if (b.RayTest(vertex, -normalV, minPen, out var pointInBox, out var distance)) pen = distance;

                    contacts[count++] = new ContactS(vertex, normalV, pen, a.collidable, b.collidable, feature);
                }
            }

            if (count == 0)
            {
                if (GetContactPoint(b, a))
                {
                    foreach (var p in pointCache)
                    {
                        var vertex = p.Item1;
                        var feature = p.Item2;
                        var pen = minPen;

                        //if (a.RayTest(vertex, normalV, minPen, out var pointInBox, out var distance)) pen = distance;

                        contacts[count++] = new ContactS(vertex, normalV, pen, a.collidable, b.collidable, feature);
                    }
                }
            }

            // Edge-to-edge contact points
            if (count == 0)
            {
                var edgesA = BoxColliderS.BOX_EDGES;
                var edgesB = BoxColliderS.BOX_EDGES;

                foreach (var edgeA in edgesA)
                {
                    var startA = a.vertices[edgeA.a];
                    var endA = a.vertices[edgeA.b];

                    foreach (var edgeB in edgesB)
                    {
                        var startB = b.vertices[edgeB.a];
                        var endB = b.vertices[edgeB.b];

                        // Calculate closest point between the edges
                        var contactPoint = FindClosestPointBetweenEdges(startA, endA, startB, endB, out var edgeDistance);

                        // Add the contact if the distance is valid
                        if (edgeDistance <= MathS.Abs(minPen))
                        {
                            contacts[count++] = new ContactS(contactPoint, normalV, minPen, a.collidable, b.collidable, (byte)(9 + count));
                            if (count >= contacts.Length) break; // Avoid overflow in the contacts array
                        }
                    }
                }
            }

            return count;

        }

        static List<(Vector3S, byte)> pointCache = new List<(Vector3S, byte)>();

        //A vertex on B
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool GetContactPoint(in BoxColliderS a, in BoxColliderS b)
        {
            pointCache.Clear();

            for (int i = 0; i < 8; i++)
            {
                var v = a.vertices[i];
                if (b.ContainsPoint(v)) pointCache.Add((v, (byte)i));
            }

            if (pointCache.Count == 0) return false;

            return true;
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static bool TryAxis(in Vector3S relativePosition, in Vector3S axis, in BoxColliderS a, in BoxColliderS b, int index, ref long minOverlap, ref Vector3S minAxis, ref int best)
        {
            if (axis.sqrMagnitude <= f32.epsilon) return true; // Skip zero-length axes

            // Calculate projection and overlap using raw values
            long pA = ProjectBoxRaw(axis, a);
            long pB = ProjectBoxRaw(axis, b);

            long distance = Vector3S.RawAbsDot(relativePosition, axis);
            long penetration = (pA + pB) - distance;

            if (penetration < 0) return false;

            if (penetration < minOverlap)
            {
                minOverlap = penetration;
                minAxis = axis;
                best = index;
            }

            return true;
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static long ProjectBoxRaw(in Vector3S axis, in BoxColliderS box)
        {
            long absDot0 = Vector3S.RawAbsDot(axis, box.rightAxis);
            long absDot1 = Vector3S.RawAbsDot(axis, box.upAxis);
            long absDot2 = Vector3S.RawAbsDot(axis, box.forwardAxis);

            return ((box.halfSize.x.rawValue * absDot0) +
                    (box.halfSize.y.rawValue * absDot1) +
                    (box.halfSize.z.rawValue * absDot2)) >> f32.FractionalBits;
        }



        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static Vector3S FindClosestPointBetweenEdges(Vector3S startA, Vector3S endA, Vector3S startB, Vector3S endB, out f32 edgeDistance)
        {
            // Edge vectors
            var d1 = endA - startA;
            var d2 = endB - startB;
            var r = startA - startB;

            f32 a = Vector3S.Dot(d1, d1); // Length of edge A squared
            f32 e = Vector3S.Dot(d2, d2); // Length of edge B squared
            f32 f = Vector3S.Dot(d2, r);

            f32 s, t;

            if (a <= f32.epsilon && e <= f32.epsilon)
            {
                // Both edges degenerate into points
                s = f32.zero;
                t = f32.zero;
                edgeDistance = (startA - startB).Magnitude();
                return startA;
            }
            if (a <= f32.epsilon)
            {
                // First edge degenerates into a point
                s = f32.zero;
                t = MathS.Clamp(f / e, f32.zero, f32.one);
            }
            else
            {
                f32 c = Vector3S.Dot(d1, r);
                if (e <= f32.epsilon)
                {
                    // Second edge degenerates into a point
                    t = f32.zero;
                    s = MathS.Clamp(-c / a, f32.zero, f32.one);
                }
                else
                {
                    // The general case
                    f32 b = Vector3S.Dot(d1, d2);
                    f32 denom = a * e - b * b;

                    if (denom != f32.zero)
                    {
                        s = MathS.Clamp((b * f - c * e) / denom, f32.zero, f32.one);
                    }
                    else
                    {
                        s = f32.zero;
                    }

                    t = (b * s + f) / e;

                    if (t < f32.zero)
                    {
                        t = f32.zero;
                        s = MathS.Clamp(-c / a, f32.zero, f32.one);
                    }
                    else if (t > f32.one)
                    {
                        t = f32.one;
                        s = MathS.Clamp((b - c) / a, f32.zero, f32.one);
                    }
                }
            }

            Vector3S closestPointA = startA + d1 * s;
            Vector3S closestPointB = startB + d2 * t;

            edgeDistance = (closestPointA - closestPointB).Magnitude();
            return (closestPointA + closestPointB) * f32.half;
        }

    }
}
