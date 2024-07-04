using SoftFloat;

namespace stupid.Maths
{
    public class Matrix3S
    {
        public sfloat[,] Elements { get; }

        public Matrix3S(sfloat[,] elements)
        {
            Elements = elements;
        }

        // New constructor that takes 3 vectors
        public Matrix3S(Vector3S row1, Vector3S row2, Vector3S row3)
        {
            Elements = new sfloat[3, 3]
            {
                { row1.x, row1.y, row1.z },
                { row2.x, row2.y, row2.z },
                { row3.x, row3.y, row3.z }
            };
        }

        public static Vector3S operator *(Matrix3S m, Vector3S v)
        {
            return new Vector3S(
                m.Elements[0, 0] * v.x + m.Elements[0, 1] * v.y + m.Elements[0, 2] * v.z,
                m.Elements[1, 0] * v.x + m.Elements[1, 1] * v.y + m.Elements[1, 2] * v.z,
                m.Elements[2, 0] * v.x + m.Elements[2, 1] * v.y + m.Elements[2, 2] * v.z
            );
        }

        public static Matrix3S operator *(Matrix3S a, Matrix3S b)
        {
            sfloat[,] result = new sfloat[3, 3];
            for (int i = 0; i < 3; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    result[i, j] = sfloat.zero;
                    for (int k = 0; k < 3; k++)
                    {
                        result[i, j] += a.Elements[i, k] * b.Elements[k, j];
                    }
                }
            }
            return new Matrix3S(result);
        }

        public Matrix3S Transpose()
        {
            sfloat[,] transposed = new sfloat[3, 3];
            for (int i = 0; i < 3; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    transposed[i, j] = Elements[j, i];
                }
            }
            return new Matrix3S(transposed);
        }

        public static Matrix3S Scale(Vector3S v)
        {
            return new Matrix3S(new sfloat[,] {
                { v.x, sfloat.zero, sfloat.zero },
                { sfloat.zero, v.y, sfloat.zero },
                { sfloat.zero, sfloat.zero, v.z }
            });
        }

        public static Matrix3S Rotate(SQuaternion q)
        {
            sfloat xx = q.x * q.x;
            sfloat yy = q.y * q.y;
            sfloat zz = q.z * q.z;
            sfloat xy = q.x * q.y;
            sfloat xz = q.x * q.z;
            sfloat yz = q.y * q.z;
            sfloat wx = q.w * q.x;
            sfloat wy = q.w * q.y;
            sfloat wz = q.w * q.z;

            return new Matrix3S(new sfloat[,] {
                { sfloat.one - sfloat.two * (yy + zz), sfloat.two * (xy - wz), sfloat.two * (xz + wy) },
                { sfloat.two * (xy + wz), sfloat.one - sfloat.two * (xx + zz), sfloat.two * (yz - wx) },
                { sfloat.two * (xz - wy), sfloat.two * (yz + wx), sfloat.one - sfloat.two * (xx + yy) }
            });
        }

        public Matrix3S Inverse()
        {
            // Simplified inverse for diagonal matrix used for inertia tensor
            sfloat[,] inverse = new sfloat[3, 3];
            for (int i = 0; i < 3; i++)
            {
                inverse[i, i] = Elements[i, i] != sfloat.zero ? sfloat.one / Elements[i, i] : sfloat.zero;
            }
            return new Matrix3S(inverse);
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
