using System;
using System.Collections.Generic;
using stupid.Maths;

namespace stupid.Colliders
{
    public static class GJK_Solver
    {
        private const int MaxIterations = 100; // Maximum number of iterations to prevent infinite loop

        public static bool Intersect(BoxColliderS a, BoxColliderS b, out ContactS contact, bool debug = false)
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

                if (debug)
                {
                    Console.WriteLine($"Iteration: {iterations}, Support: {support}, Direction: {direction}");
                }

                if (Vector3S.Dot(support, direction) <= f32.zero)
                {
                    return false; // No collision
                }

                simplex.Add(support);

                if (HandleSimplex(simplex, ref direction))
                {
                    // Perform EPA to find contact point and normal
                    return EPA(a, b, simplex, out contact);
                }
            }

            if (debug)
            {
                Console.WriteLine("GJK failed to converge within the iteration limit.");
            }

            return false; // If the loop exits without returning, assume no collision
        }

        private static Vector3S Support(BoxColliderS a, BoxColliderS b, Vector3S direction)
        {
            Vector3S pointA = a.GetFarthestPointInDirection(direction);
            Vector3S pointB = b.GetFarthestPointInDirection(-direction);
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

        private static bool EPA(BoxColliderS a, BoxColliderS b, List<Vector3S> simplex, out ContactS contact)
        {
            contact = new ContactS();

            List<Vector3S> polytope = new List<Vector3S>(simplex);
            List<(Vector3S, Vector3S, Vector3S)> faces = new List<(Vector3S, Vector3S, Vector3S)>();
            List<Vector3S> normals = new List<Vector3S>();
            f32 penetrationDepth = f32.zero;

            int iterations = 0;

            // Initialize the faces of the polytope from the simplex
            if (simplex.Count == 4)
            {
                AddFace(faces, normals, simplex[0], simplex[1], simplex[2]);
                AddFace(faces, normals, simplex[0], simplex[1], simplex[3]);
                AddFace(faces, normals, simplex[0], simplex[2], simplex[3]);
                AddFace(faces, normals, simplex[1], simplex[2], simplex[3]);
            }

            while (iterations < MaxIterations)
            {
                iterations++;
                f32 minDistance = f32.maxValue;
                int closestFaceIndex = -1;

                // Find the closest face to the origin
                for (int i = 0; i < faces.Count; i++)
                {
                    f32 distance = Vector3S.Dot(normals[i], faces[i].Item1);

                    if (distance < minDistance)
                    {
                        minDistance = distance;
                        closestFaceIndex = i;
                    }
                }

                if (closestFaceIndex == -1)
                {
                    break; // Something went wrong
                }

                // Get the closest face vertices and normal
                var (faceA, faceB, faceC) = faces[closestFaceIndex];
                Vector3S normal = normals[closestFaceIndex];

                // Calculate the new support point in the direction of the face normal
                Vector3S support = Support(a, b, normal);

                // Calculate the penetration depth
                penetrationDepth = Vector3S.Dot(normal, support);

                if (penetrationDepth - minDistance < f32.epsilon)
                {
                    // Project the centroid of the face onto the normal to find the contact point
                    Vector3S closestPoint = Vector3S.ProjectPointOnPlane((faceA + faceB + faceC) / (f32)3, normal, faceA);

                    // Transform the contact point to world space
                    closestPoint = a.collidable.transform.ToWorldPoint(closestPoint);

                    contact.point = closestPoint;
                    contact.normal = normal;
                    contact.penetrationDepth = penetrationDepth;
                    return true;
                }

                // Remove the closest face and add the new faces formed with the new support point
                faces.RemoveAt(closestFaceIndex);
                normals.RemoveAt(closestFaceIndex);

                AddFace(faces, normals, faceA, faceB, support);
                AddFace(faces, normals, faceB, faceC, support);
                AddFace(faces, normals, faceC, faceA, support);
            }

            return false;
        }

        private static void AddFace(List<(Vector3S, Vector3S, Vector3S)> faces, List<Vector3S> normals, Vector3S a, Vector3S b, Vector3S c)
        {
            Vector3S normal = Vector3S.Cross(b - a, c - a).Normalize();
            faces.Add((a, b, c));
            normals.Add(normal);
        }

    }
}
