using System;

namespace stupid.Maths
{
    public class Matrix3S
    {
        public f32[,] Elements { get; }

        public Matrix3S(f32[,] elements)
        {
            Elements = elements;
        }

        // New constructor that takes 3 vectors
        public Matrix3S(Vector3S row1, Vector3S row2, Vector3S row3)
        {
            Elements = new f32[3, 3]
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
            f32[,] result = new f32[3, 3];
            for (int i = 0; i < 3; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    result[i, j] = f32.zero;
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
            f32[,] transposed = new f32[3, 3];
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
            return new Matrix3S(new f32[,] {
                { v.x, f32.zero, f32.zero },
                { f32.zero, v.y, f32.zero },
                { f32.zero, f32.zero, v.z }
            });
        }

        public static Matrix3S Rotate(SQuaternion q)
        {
            f32 xx = q.x * q.x;
            f32 yy = q.y * q.y;
            f32 zz = q.z * q.z;
            f32 xy = q.x * q.y;
            f32 xz = q.x * q.z;
            f32 yz = q.y * q.z;
            f32 wx = q.w * q.x;
            f32 wy = q.w * q.y;
            f32 wz = q.w * q.z;

            return new Matrix3S(new f32[,] {
                { f32.one - f32.two * (yy + zz), f32.two * (xy - wz), f32.two * (xz + wy) },
                { f32.two * (xy + wz), f32.one - f32.two * (xx + zz), f32.two * (yz - wx) },
                { f32.two * (xz - wy), f32.two * (yz + wx), f32.one - f32.two * (xx + yy) }
            });
        }

        public Matrix3S Inverse()
        {
            // Simplified inverse for diagonal matrix used for inertia tensor
            f32[,] inverse = new f32[3, 3];
            for (int i = 0; i < 3; i++)
            {
                inverse[i, i] = Elements[i, i] != f32.zero ? f32.one / Elements[i, i] : f32.zero;
            }
            return new Matrix3S(inverse);
        }

        public static Matrix3S Inverse(Matrix3S m)
        {
            f32 determinant =
                m.Elements[0, 0] * (m.Elements[1, 1] * m.Elements[2, 2] - m.Elements[1, 2] * m.Elements[2, 1]) -
                m.Elements[0, 1] * (m.Elements[1, 0] * m.Elements[2, 2] - m.Elements[1, 2] * m.Elements[2, 0]) +
                m.Elements[0, 2] * (m.Elements[1, 0] * m.Elements[2, 1] - m.Elements[1, 1] * m.Elements[2, 0]);

            f32 invDet = f32.one / determinant;

            f32[,] inv = new f32[3, 3];
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
