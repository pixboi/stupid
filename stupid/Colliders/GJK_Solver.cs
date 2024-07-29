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
                    //Need the contact here, but dont know how
                    return true; // Collision detected
                }
            }

            return false; // If the loop exits without returning, assume no collision
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