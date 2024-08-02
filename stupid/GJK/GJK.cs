using System;
using System.Collections.Generic;
using System.Numerics;
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

            public Vector3S Last()
            {
                return Points[Points.Count - 1];
            }
        }

        public bool Intersect(ISupport shapeA, ISupport shapeB)
        {
            Vector3S direction = new Vector3S(1, 0, 0); // Arbitrary initial direction
            Simplex simplex = new Simplex(Support(shapeA, shapeB, direction));

            direction = -simplex.Last();

            while (true)
            {
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

            if (simplex.Points.Count == 3)
            {
                Vector3S B = simplex.Points[1];
                Vector3S C = simplex.Points[2];

                Vector3S AB = B - A;
                Vector3S AC = C - A;

                Vector3S ABPerp = Vector3S.Cross(Vector3S.Cross(AC, AB), AB);
                Vector3S ACPerp = Vector3S.Cross(Vector3S.Cross(AB, AC), AC);

                if (Vector3S.Dot(ABPerp, AO) > f32.zero)
                {
                    simplex.RemoveLast();
                    direction = ABPerp;
                }
                else if (Vector3S.Dot(ACPerp, AO) > f32.zero)
                {
                    simplex.Points.RemoveAt(1);
                    direction = ACPerp;
                }
                else
                {
                    return true;
                }
            }
            else
            {
                Vector3S B = simplex.Points[1];
                Vector3S AB = B - A;
                direction = Vector3S.Cross(Vector3S.Cross(AB, AO), AB);
            }

            return false;
        }
    }

}
