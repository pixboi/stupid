using System;
using System.Collections.Generic;
using stupid.Maths;

namespace stupid.GJK
{
    public class EPA
    {
        private struct Face
        {
            public Vector3S A;
            public Vector3S B;
            public Vector3S C;
            public Vector3S Normal;
            public f32 Distance;

            public Face(Vector3S a, Vector3S b, Vector3S c)
            {
                A = a;
                B = b;
                C = c;
                Normal = Vector3S.Cross(B - A, C - A).Normalize();
                Distance = Vector3S.Dot(Normal, A);
            }
        }

        public Vector3S FindPenetrationDepth(ISupport shapeA, ISupport shapeB, GJK.Simplex simplex, int maxIterations = 50)
        {
            List<Vector3S> polytope = new List<Vector3S>(simplex.Points);
            List<Face> faces = new List<Face>();

            // Initialize the polytope with the faces of the tetrahedron
            InitializePolytopeFaces(polytope, faces);

            int iterations = 0;
            while (iterations < maxIterations)
            {
                iterations++;
                int closestFaceIndex = -1;
                f32 minDistance = f32.maxValue;

                // Find the closest face to the origin
                for (int i = 0; i < faces.Count; i++)
                {
                    if (faces[i].Distance < minDistance)
                    {
                        minDistance = faces[i].Distance;
                        closestFaceIndex = i;
                    }
                }

                if (closestFaceIndex == -1)
                {
                    break;
                }

                Face closestFace = faces[closestFaceIndex];
                Vector3S newPoint = Support(shapeA, shapeB, closestFace.Normal);

                f32 distanceToNewPoint = Vector3S.Dot(closestFace.Normal, newPoint);

                // Check if the new point is close enough to the face
                if (MathS.Abs(distanceToNewPoint - closestFace.Distance) < f32.epsilon)
                {
                    return closestFace.Normal * distanceToNewPoint;
                }

                // Add the new point to the polytope and update the faces
                UpdatePolytopeAndFaces(newPoint, ref polytope, ref faces, closestFaceIndex);
            }

            return new Vector3S(0, 0, 0); // Failed to converge, returning zero vector as penetration depth
        }

        private void InitializePolytopeFaces(List<Vector3S> polytope, List<Face> faces)
        {
            // Create the initial faces from the tetrahedron points
            faces.Add(new Face(polytope[0], polytope[1], polytope[2]));
            faces.Add(new Face(polytope[0], polytope[1], polytope[3]));
            faces.Add(new Face(polytope[0], polytope[2], polytope[3]));
            faces.Add(new Face(polytope[1], polytope[2], polytope[3]));
        }

        private void UpdatePolytopeAndFaces(Vector3S newPoint, ref List<Vector3S> polytope, ref List<Face> faces, int closestFaceIndex)
        {
            List<Face> newFaces = new List<Face>();
            List<(Vector3S, Vector3S)> edges = new List<(Vector3S, Vector3S)>();

            // Remove the closest face and collect its edges
            CollectEdges(faces[closestFaceIndex], ref edges);
            faces.RemoveAt(closestFaceIndex);

            // Iterate over the remaining faces and collect unique edges
            for (int i = faces.Count - 1; i >= 0; i--)
            {
                if (IsVisible(faces[i], newPoint))
                {
                    CollectEdges(faces[i], ref edges);
                    faces.RemoveAt(i);
                }
            }

            // Create new faces from the edges and the new point
            foreach (var edge in edges)
            {
                newFaces.Add(new Face(edge.Item1, edge.Item2, newPoint));
            }

            faces.AddRange(newFaces);
        }

        private void CollectEdges(Face face, ref List<(Vector3S, Vector3S)> edges)
        {
            AddEdge(face.A, face.B, ref edges);
            AddEdge(face.B, face.C, ref edges);
            AddEdge(face.C, face.A, ref edges);
        }

        private void AddEdge(Vector3S a, Vector3S b, ref List<(Vector3S, Vector3S)> edges)
        {
            // Add the edge if it is not already in the list, otherwise remove the existing one
            var reverseEdge = (b, a);
            if (!edges.Remove(reverseEdge))
            {
                edges.Add((a, b));
            }
        }

        private bool IsVisible(Face face, Vector3S point)
        {
            return Vector3S.Dot(face.Normal, point - face.A) > f32.zero;
        }

        private Vector3S Support(ISupport shapeA, ISupport shapeB, Vector3S direction)
        {
            Vector3S pointA = shapeA.Support(direction);
            Vector3S pointB = shapeB.Support(-direction);
            return pointA - pointB;
        }
    }
}
