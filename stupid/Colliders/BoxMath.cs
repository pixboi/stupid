using stupid.Constraints;
using stupid.Maths;
using System;
using System.Collections.Generic;
using System.Runtime.CompilerServices;

namespace stupid.Colliders
{
    public static partial class CollisionMath
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int BoxVsBox(in BoxColliderS a, in BoxColliderS b, ref ContactData[] contacts)
        {
            Vector3S relativePosition = b._collidable.transform.position - a._collidable.transform.position;

            f32 minPen = f32.maxValue;
            Vector3S minAxis = Vector3S.zero;
            int best = -1;

            // Check for overlaps on the primary axes of both boxes
            if (!TryAxis(relativePosition, a.rightAxis, a, b, 0, ref minPen, ref minAxis, ref best)) return 0;
            if (!TryAxis(relativePosition, a.upAxis, a, b, 1, ref minPen, ref minAxis, ref best)) return 0;
            if (!TryAxis(relativePosition, a.forwardAxis, a, b, 2, ref minPen, ref minAxis, ref best)) return 0;

            if (!TryAxis(relativePosition, b.rightAxis, a, b, 3, ref minPen, ref minAxis, ref best)) return 0;
            if (!TryAxis(relativePosition, b.upAxis, a, b, 4, ref minPen, ref minAxis, ref best)) return 0;
            if (!TryAxis(relativePosition, b.forwardAxis, a, b, 5, ref minPen, ref minAxis, ref best)) return 0;

            // Check for overlaps on the cross product of axes pairs
            //Need normalizations
            if (!TryAxis(relativePosition, Vector3S.Cross(a.rightAxis, b.rightAxis).Normalize(), a, b, 6, ref minPen, ref minAxis, ref best)) return 0; //0,0
            if (!TryAxis(relativePosition, Vector3S.Cross(a.rightAxis, b.upAxis).Normalize(), a, b, 7, ref minPen, ref minAxis, ref best)) return 0; //0,1
            if (!TryAxis(relativePosition, Vector3S.Cross(a.rightAxis, b.forwardAxis).Normalize(), a, b, 8, ref minPen, ref minAxis, ref best)) return 0; //0,2

            if (!TryAxis(relativePosition, Vector3S.Cross(a.upAxis, b.rightAxis).Normalize(), a, b, 9, ref minPen, ref minAxis, ref best)) return 0; //1,0
            if (!TryAxis(relativePosition, Vector3S.Cross(a.upAxis, b.upAxis).Normalize(), a, b, 10, ref minPen, ref minAxis, ref best)) return 0; //1,1
            if (!TryAxis(relativePosition, Vector3S.Cross(a.upAxis, b.forwardAxis).Normalize(), a, b, 11, ref minPen, ref minAxis, ref best)) return 0; //1,2

            if (!TryAxis(relativePosition, Vector3S.Cross(a.forwardAxis, b.rightAxis).Normalize(), a, b, 12, ref minPen, ref minAxis, ref best)) return 0; //2,0
            if (!TryAxis(relativePosition, Vector3S.Cross(a.forwardAxis, b.upAxis).Normalize(), a, b, 13, ref minPen, ref minAxis, ref best)) return 0; //2,1
            if (!TryAxis(relativePosition, Vector3S.Cross(a.forwardAxis, b.forwardAxis).Normalize(), a, b, 14, ref minPen, ref minAxis, ref best)) return 0; //2,2

            if (best == -1)
            {
                throw new Exception("OOBB collision error");
            }

            var normalV = minAxis.Normalize();
            if (Vector3S.Dot(normalV, relativePosition) < f32.zero) normalV = -normalV;

            int count = 0;
            minPen = -minPen;

            Span<Vector3S> aVertices = stackalloc Vector3S[8]; // Using stackalloc for fast allocation on the stack
            a.GetWorldVertices(ref aVertices);

            //A vert on b
            if (GetContactPoint(ref aVertices, b))
            {
                foreach (var p in pointCache)
                {
                    var vertex = p.Item1;
                    var feature = p.Item2;

                    contacts[count++] = new ContactData(vertex, normalV, minPen, feature);
                    if (count == contacts.Length) return count; // Early exit if max contacts reached
                }
            }

            Span<Vector3S> bVertices = stackalloc Vector3S[8]; // Using stackalloc for fast allocation on the stack
            b.GetWorldVertices(ref bVertices);

            if (count == 0)
            {
                if (GetContactPoint(ref bVertices, a))
                {
                    foreach (var p in pointCache)
                    {
                        var vertex = p.Item1;
                        var feature = p.Item2;

                        contacts[count++] = new ContactData(vertex, normalV, minPen, feature + 8);
                        if (count == contacts.Length) return count; // Early exit if max contacts reached
                    }
                }
            }

            if (count == 0)
            {
                var edges = BoxColliderS.BOX_EDGES;

                for (int i = 0; i < edges.Length; i++)
                {
                    var start = aVertices[edges[i].a];
                    var end = aVertices[edges[i].b];
                    var dir = end - start;

                    // Create a base feature ID using the edge index (i)
                    int baseFeatureID = 16 + (i * 2); // Each edge has 2 potential directions to check

                    // Check in the positive direction
                    if (b.RaycastBox(start, dir, out var p1, out var m1))
                    {
                        contacts[count++] = new ContactData(p1, normalV, minPen, baseFeatureID);
                        if (count == contacts.Length) return count; // Early exit if max contacts reached
                    }

                    // Check in the negative direction
                    if (b.RaycastBox(start, -dir, out var p2, out var m2))
                    {
                        contacts[count++] = new ContactData(p2, normalV, minPen, baseFeatureID + 1);
                        if (count == contacts.Length) return count; // Early exit if max contacts reached
                    }
                }
            }

            return count;
        }


        static List<(Vector3S, int)> pointCache = new List<(Vector3S, int)>();
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool GetContactPoint(ref Span<Vector3S> vertices, in BoxColliderS b)
        {
            pointCache.Clear();

            //A vertex on B
            for (int i = 0; i < 8; i++)
            {
                ref var v = ref vertices[i];
                if (b.ContainsPoint(v)) pointCache.Add((v, i));
            }

            if (pointCache.Count == 0) return false;

            return true;
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static bool TryAxis(in Vector3S relativePosition, in Vector3S axis, in BoxColliderS a, in BoxColliderS b, int index, ref f32 minOverlap, ref Vector3S minAxis, ref int best)
        {
            if (axis.sqrMagnitude <= f32.epsilon) return true; // Skip zero-length axes

            // Calculate projection and overlap using raw values
            var pA = ProjectBox(axis, a);
            var pB = ProjectBox(axis, b);

            var distance = Vector3S.AbsDot(relativePosition, axis);
            var penetration = (pA + pB) - distance;

            if (penetration < f32.zero) return false;

            if (penetration < minOverlap)
            {
                minOverlap = penetration;
                minAxis = axis;
                best = index;
            }

            return true;
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static f32 ProjectBox(in Vector3S axis, in BoxColliderS box)
        {
            var absDot0 = Vector3S.AbsDot(axis, box.rightAxis);
            var absDot1 = Vector3S.AbsDot(axis, box.upAxis);
            var absDot2 = Vector3S.AbsDot(axis, box.forwardAxis);

            return ((box.halfSize.x * absDot0) + (box.halfSize.y * absDot1) + (box.halfSize.z * absDot2));
        }
    }
}
