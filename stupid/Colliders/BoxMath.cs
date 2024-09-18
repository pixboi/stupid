using stupid.Maths;
using System;
using System.Collections.Generic;
using System.Linq;
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

                    contacts[count++] = new ContactS(vertex, normalV, pen, a.collidable, b.collidable, feature);
                    if (count == contacts.Length) return count; // Early exit if max contacts reached
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

                        contacts[count++] = new ContactS(vertex, normalV, pen, a.collidable, b.collidable, feature + 8);
                        if (count == contacts.Length) return count; // Early exit if max contacts reached
                    }
                }
            }

            if (count == 0)
            {
                var edges = BoxColliderS.BOX_EDGES;

                for (int i = 0; i < edges.Length; i++)
                {
                    var start = a.vertices[edges[i].a];
                    var end = a.vertices[edges[i].b];
                    var dir = end - start;

                    // Create a base feature ID using the edge index (i)
                    int baseFeatureID = 16 + (i * 2); // Each edge has 2 potential directions to check

                    // Check in the positive direction
                    if (b.RaycastBox(start, dir, out var p1, out var m1))
                    {
                        contacts[count++] = new ContactS(p1, normalV, minPen, a.collidable, b.collidable, baseFeatureID);
                        if (count == contacts.Length) return count; // Early exit if max contacts reached
                    }

                    // Check in the negative direction
                    if (b.RaycastBox(start, -dir, out var p2, out var m2))
                    {
                        contacts[count++] = new ContactS(p2, normalV, minPen, a.collidable, b.collidable, baseFeatureID + 1);
                        if (count == contacts.Length) return count; // Early exit if max contacts reached
                    }
                }
            }

            return count;
        }


        static List<(Vector3S, int)> pointCache = new List<(Vector3S, int)>();
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool GetContactPoint(in BoxColliderS a, in BoxColliderS b)
        {

            pointCache.Clear();
            //A vertex on B
            for (int i = 0; i < 8; i++)
            {
                var v = a.vertices[i];
                if (b.ContainsPoint(v)) pointCache.Add((v, i));
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





    }
}
