using System;
using System.Collections.Generic;
using stupid.Maths;

namespace stupid.GJK
{
    public class EPA
    {
        private struct Edge
        {
            public Vector3S A;
            public Vector3S B;
            public Vector3S Normal;
            public f32 Distance;
        }

        public Vector3S FindPenetrationDepth(ISupport shapeA, ISupport shapeB, GJK.Simplex simplex)
        {
            List<Vector3S> polytope = simplex.Points;
            List<Edge> edges = new List<Edge>();

            while (true)
            {
                int closestEdgeIndex = -1;
                f32 minDistance = f32.maxValue;

                for (int i = 0; i < polytope.Count; i++)
                {
                    Vector3S A = polytope[i];
                    Vector3S B = polytope[(i + 1) % polytope.Count];
                    Vector3S edgeNormal = Vector3S.Cross(B - A, polytope[0] - A).Normalize();
                    f32 distance = Vector3S.Dot(edgeNormal, A);

                    if (distance < minDistance)
                    {
                        minDistance = distance;
                        closestEdgeIndex = i;
                    }

                    edges.Add(new Edge { A = A, B = B, Normal = edgeNormal, Distance = distance });
                }

                Edge closestEdge = edges[closestEdgeIndex];
                Vector3S newPoint = Support(shapeA, shapeB, closestEdge.Normal);

                f32 distanceToNewPoint = Vector3S.Dot(closestEdge.Normal, newPoint);

                if (MathS.Abs(distanceToNewPoint - closestEdge.Distance) < f32.epsilon)
                {
                    return closestEdge.Normal * distanceToNewPoint;
                }

                polytope.Insert(closestEdgeIndex + 1, newPoint);
                edges.Clear();
            }
        }

        private Vector3S Support(ISupport shapeA, ISupport shapeB, Vector3S direction)
        {
            Vector3S pointA = shapeA.Support(direction);
            Vector3S pointB = shapeB.Support(-direction);
            return pointA - pointB;
        }
    }

}
