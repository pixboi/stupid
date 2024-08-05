using System;
using System.Collections.Generic;
using stupid.Maths;

namespace stupid.GJK
{
    public interface ISupport
    {
        Vector3S Support(Vector3S direction);
    }

    public class GJK
    {
        public struct Simplex
        {
            public List<Vector3S> Points;

            public Simplex(Vector3S a)
            {
                Points = new List<Vector3S> { a };
            }

            public void Add(Vector3S point)
            {
                Points.Insert(0, point);
            }

            public void RemoveLast()
            {
                Points.RemoveAt(Points.Count - 1);
            }

            public void RemoveAt(int index)
            {
                Points.RemoveAt(index);
            }

            public Vector3S Last()
            {
                return Points[Points.Count - 1];
            }
        }

        public bool Intersect(ISupport shapeA, ISupport shapeB, out Simplex simplex, int maxIterations = 50)
        {
            Vector3S direction = new Vector3S(1, 0, 0); // Arbitrary initial direction
            simplex = new Simplex(Support(shapeA, shapeB, direction));

            direction = -simplex.Last();

            int iterations = 0;
            while (iterations < maxIterations)
            {
                iterations++;
                Vector3S A = Support(shapeA, shapeB, direction);
                if (Vector3S.Dot(A, direction) <= f32.zero)
                {
                    return false;
                }

                simplex.Add(A);

                if (SimplexContainsOrigin(ref simplex, ref direction))
                {
                    return true;
                }
            }

            return false; // Failed to converge
        }

        private Vector3S Support(ISupport shapeA, ISupport shapeB, Vector3S direction)
        {
            Vector3S pointA = shapeA.Support(direction);
            Vector3S pointB = shapeB.Support(-direction);
            return pointA - pointB;
        }

        private bool SimplexContainsOrigin(ref Simplex simplex, ref Vector3S direction)
        {
            Vector3S A = simplex.Points[0];
            Vector3S AO = -A;

            if (simplex.Points.Count == 4)
            {
                Vector3S B = simplex.Points[1];
                Vector3S C = simplex.Points[2];
                Vector3S D = simplex.Points[3];

                Vector3S AB = B - A;
                Vector3S AC = C - A;
                Vector3S AD = D - A;

                Vector3S ABC = Vector3S.Cross(AB, AC);
                Vector3S ACD = Vector3S.Cross(AC, AD);
                Vector3S ADB = Vector3S.Cross(AD, AB);

                if (Vector3S.Dot(ABC, AO) > f32.zero)
                {
                    simplex.RemoveAt(3); // Remove point D
                    direction = ABC;
                }
                else if (Vector3S.Dot(ACD, AO) > f32.zero)
                {
                    simplex.RemoveAt(1); // Remove point B
                    direction = ACD;
                }
                else if (Vector3S.Dot(ADB, AO) > f32.zero)
                {
                    simplex.RemoveAt(2); // Remove point C
                    direction = ADB;
                }
                else
                {
                    return true; // Origin is inside the tetrahedron
                }
            }
            else if (simplex.Points.Count == 3)
            {
                Vector3S B = simplex.Points[1];
                Vector3S C = simplex.Points[2];

                Vector3S AB = B - A;
                Vector3S AC = C - A;

                Vector3S ABC = Vector3S.Cross(AB, AC);

                if (Vector3S.Dot(Vector3S.Cross(ABC, AC), AO) > f32.zero)
                {
                    simplex.RemoveAt(1); // Remove point B
                    direction = Vector3S.Cross(AC, AO);
                }
                else if (Vector3S.Dot(Vector3S.Cross(AB, ABC), AO) > f32.zero)
                {
                    simplex.RemoveAt(2); // Remove point C
                    direction = Vector3S.Cross(AB, AO);
                }
                else
                {
                    direction = ABC;
                }
            }
            else if (simplex.Points.Count == 2)
            {
                Vector3S B = simplex.Points[1];
                Vector3S AB = B - A;
                direction = Vector3S.Cross(Vector3S.Cross(AB, AO), AB);
            }
            else
            {
                direction = AO;
            }

            return false;
        }
    }
}
