using stupid.Maths;
using System;
using System.Collections.Generic;

namespace stupid.Colliders
{
    public static class GJK_Solver
    {
        private const int MaxIterations = 32; // Maximum number of iterations to prevent infinite loop

        public static bool Intersect(BoxColliderS a, BoxColliderS b, out ContactS contact)
        {
            contact = new ContactS();
            Vector3S initialDirection = b.collidable.transform.position - a.collidable.transform.position;
            Vector3S direction = initialDirection == Vector3S.zero ? new Vector3S(1, 0, 0) : initialDirection;
            Vector3S support = Support(a, b, direction);
            List<Vector3S> simplex = new List<Vector3S> { support };
            direction = -support;

            int iterations = 0;

            while (iterations < MaxIterations)
            {
                iterations++;
                support = Support(a, b, direction);

                if (Vector3S.Dot(support, direction) <= f32.zero)
                {
                    return false; // No collision
                }

                simplex.Add(support);

                if (HandleSimplex(simplex, ref direction))
                {

                    EPA(simplex, a, b, out contact);
                    return true; // Collision detected
                }
            }

            return false; // If the loop exits without returning, assume no collision
        }

        public static bool EPA(List<Vector3S> simplex, BoxColliderS a, BoxColliderS b, out ContactS contact)
        {
            contact = new ContactS();
            List<Vector3S> polytope = new List<Vector3S>(simplex);
            List<(Vector3S, Vector3S, Vector3S)> faces = new List<(Vector3S, Vector3S, Vector3S)>
    {
        (polytope[0], polytope[1], polytope[2]),
        (polytope[0], polytope[2], polytope[3]),
        (polytope[0], polytope[3], polytope[1]),
        (polytope[1], polytope[3], polytope[2])
    };

            Vector3S minNormal = default;
            f32 minDistance = f32.maxValue;
            int maxIterations = 64; // Prevent infinite loops
            int iteration = 0;

            while (minDistance == f32.maxValue && iteration < maxIterations)
            {
                iteration++;
                (Vector3S a, Vector3S b, Vector3S c) closestFace = default;
                minDistance = f32.maxValue;

                for (int i = 0; i < faces.Count; i++)
                {
                    var face = faces[i];
                    Vector3S normal = Vector3S.Cross(face.Item2 - face.Item1, face.Item3 - face.Item1).Normalize();
                    f32 distance = Vector3S.Dot(normal, face.Item1);

                    if (distance < f32.zero)
                    {
                        distance *= f32.negativeOne;
                        normal = -normal;
                    }

                    if (distance < minDistance)
                    {
                        minDistance = distance;
                        minNormal = normal;
                        closestFace = face;
                    }
                }

                Vector3S support = Support(a, b, minNormal);
                f32 sDistance = Vector3S.Dot(minNormal, support);

                if (MathS.Abs(sDistance - minDistance) > f32.epsilon)
                {
                    polytope.Add(support);

                    faces.Remove(closestFace);
                    faces.Add((closestFace.a, closestFace.b, support));
                    faces.Add((closestFace.b, closestFace.c, support));
                    faces.Add((closestFace.c, closestFace.a, support));

                    minDistance = f32.maxValue; // Reset minDistance to continue the loop
                }
            }

            if (iteration >= maxIterations)
            {
                contact = new ContactS
                {
                    point = Vector3S.zero,
                    normal = Vector3S.zero,
                    penetrationDepth = f32.zero
                };
                return false;
            }

            contact = new ContactS
            {
                point = polytope[0], // This might need to be refined for the actual closest point on the polytope
                normal = minNormal,
                penetrationDepth = minDistance + f32.epsilon
            };

            return true;
        }



        private static Vector3S Support(BoxColliderS a, BoxColliderS b, Vector3S direction)
        {
            Vector3S pointA = a.SupportFunction(direction);
            Vector3S pointB = b.SupportFunction(-direction);
            return pointA - pointB;
        }

        private static bool HandleSimplex(List<Vector3S> simplex, ref Vector3S direction)
        {
            switch (simplex.Count)
            {
                case 2:
                    return Line(simplex, ref direction);
                case 3:
                    return Triangle(simplex, ref direction);
                case 4:
                    return Tetrahedron(simplex, ref direction);
                default:
                    return false;
            }
        }

        private static bool Line(List<Vector3S> simplex, ref Vector3S direction)
        {
            Vector3S a = simplex[1];
            Vector3S b = simplex[0];
            Vector3S ab = b - a;
            Vector3S ao = -a;

            if (Vector3S.Dot(ab, ao) > f32.zero)
            {
                direction = Vector3S.Cross(Vector3S.Cross(ab, ao), ab);
            }
            else
            {
                simplex.RemoveAt(0);
                direction = ao;
            }
            return false;
        }

        private static bool Triangle(List<Vector3S> simplex, ref Vector3S direction)
        {
            Vector3S a = simplex[2];
            Vector3S b = simplex[1];
            Vector3S c = simplex[0];
            Vector3S ab = b - a;
            Vector3S ac = c - a;
            Vector3S ao = -a;

            Vector3S abc = Vector3S.Cross(ab, ac);

            if (Vector3S.Dot(Vector3S.Cross(abc, ac), ao) > f32.zero)
            {
                if (Vector3S.Dot(ac, ao) > f32.zero)
                {
                    simplex.RemoveAt(1);
                    direction = Vector3S.Cross(Vector3S.Cross(ac, ao), ac);
                }
                else
                {
                    return Line(simplex, ref direction);
                }
            }
            else
            {
                if (Vector3S.Dot(Vector3S.Cross(ab, abc), ao) > f32.zero)
                {
                    return Line(simplex, ref direction);
                }
                else
                {
                    if (Vector3S.Dot(abc, ao) > f32.zero)
                    {
                        direction = abc;
                    }
                    else
                    {
                        var temp = simplex[0];
                        simplex[0] = simplex[1];
                        simplex[1] = temp;
                        direction = -abc;
                    }
                }
            }
            return false;
        }

        private static bool Tetrahedron(List<Vector3S> simplex, ref Vector3S direction)
        {
            Vector3S a = simplex[3];
            Vector3S b = simplex[2];
            Vector3S c = simplex[1];
            Vector3S d = simplex[0];
            Vector3S ab = b - a;
            Vector3S ac = c - a;
            Vector3S ad = d - a;
            Vector3S ao = -a;

            Vector3S abc = Vector3S.Cross(ab, ac);
            Vector3S acd = Vector3S.Cross(ac, ad);
            Vector3S adb = Vector3S.Cross(ad, ab);

            if (Vector3S.Dot(abc, ao) > f32.zero)
            {
                simplex.RemoveAt(0);
                direction = abc;
            }
            else if (Vector3S.Dot(acd, ao) > f32.zero)
            {
                simplex.RemoveAt(1);
                direction = acd;
            }
            else if (Vector3S.Dot(adb, ao) > f32.zero)
            {
                simplex.RemoveAt(2);
                direction = adb;
            }
            else
            {
                return true;
            }
            return false;
        }
    }
}