using SoftFloat;
using stupid.Maths;

namespace stupid
{
    public class Matrix3S
    {
        public sfloat[,] Elements { get; }

        public Matrix3S(sfloat[,] elements)
        {
            Elements = elements;
        }

        public static Vector3S operator *(Matrix3S m, Vector3S v)
        {
            return new Vector3S(
                m.Elements[0, 0] * v.x + m.Elements[0, 1] * v.y + m.Elements[0, 2] * v.z,
                m.Elements[1, 0] * v.x + m.Elements[1, 1] * v.y + m.Elements[1, 2] * v.z,
                m.Elements[2, 0] * v.x + m.Elements[2, 1] * v.y + m.Elements[2, 2] * v.z
            );
        }

        public static Matrix3S Inverse(Matrix3S m)
        {
            sfloat determinant =
                m.Elements[0, 0] * (m.Elements[1, 1] * m.Elements[2, 2] - m.Elements[1, 2] * m.Elements[2, 1]) -
                m.Elements[0, 1] * (m.Elements[1, 0] * m.Elements[2, 2] - m.Elements[1, 2] * m.Elements[2, 0]) +
                m.Elements[0, 2] * (m.Elements[1, 0] * m.Elements[2, 1] - m.Elements[1, 1] * m.Elements[2, 0]);

            sfloat invDet = sfloat.one / determinant;

            sfloat[,] inv = new sfloat[3, 3];
            inv[0, 0] = invDet * (m.Elements[1, 1] * m.Elements[2, 2] - m.Elements[1, 2] * m.Elements[2, 1]);
            inv[0, 1] = invDet * (m.Elements[0, 2] * m.Elements[2, 1] - m.Elements[0, 1] * m.Elements[2, 2]);
            inv[0, 2] = invDet * (m.Elements[0, 1] * m.Elements[1, 2] - m.Elements[0, 2] * m.Elements[1, 1]);
            inv[1, 0] = invDet * (m.Elements[1, 2] * m.Elements[2, 0] - m.Elements[1, 0] * m.Elements[2, 2]);
            inv[1, 1] = invDet * (m.Elements[0, 0] * m.Elements[2, 2] - m.Elements[0, 2] * m.Elements[2, 0]);
            inv[1, 2] = invDet * (m.Elements[0, 2] * m.Elements[1, 0] - m.Elements[0, 0] * m.Elements[1, 2]);
            inv[2, 0] = invDet * (m.Elements[1, 0] * m.Elements[2, 1] - m.Elements[1, 1] * m.Elements[2, 0]);
            inv[2, 1] = invDet * (m.Elements[0, 1] * m.Elements[2, 0] - m.Elements[0, 0] * m.Elements[2, 1]);
            inv[2, 2] = invDet * (m.Elements[0, 0] * m.Elements[1, 1] - m.Elements[0, 1] * m.Elements[1, 0]);

            return new Matrix3S(inv);
        }
    }
}
