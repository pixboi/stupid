using System;
using System.Numerics;

namespace stupid.Maths
{
    public struct QuaternionS
    {
        public f32 x;
        public f32 y;
        public f32 z;
        public f32 w;

        public static readonly QuaternionS identity = new QuaternionS(f32.zero, f32.zero, f32.zero, f32.one);
        public static readonly QuaternionS zero = new QuaternionS(f32.zero, f32.zero, f32.zero, f32.zero);

        public QuaternionS(f32 x, f32 y, f32 z, f32 w)
        {
            this.x = x;
            this.y = y;
            this.z = z;
            this.w = w;
        }

        public QuaternionS(float x, float y, float z, float w)
        {
            this.x = f32.FromFloat(x);
            this.y = f32.FromFloat(y);
            this.z = f32.FromFloat(z);
            this.w = f32.FromFloat(w);
        }

        public static QuaternionS FromAxisAngle(Vector3S axis, f32 angle)
        {
            f32 halfAngle = angle * f32.half;
            f32 sinHalfAngle = MathS.Sin(halfAngle);
            return new QuaternionS(
                axis.x * sinHalfAngle,
                axis.y * sinHalfAngle,
                axis.z * sinHalfAngle,
                MathS.Cos(halfAngle)
            );
        }

        public static QuaternionS FromEulerAngles(Vector3S eulerAngles)
        {
            f32 c1 = MathS.Cos(eulerAngles.y * f32.half);
            f32 c2 = MathS.Cos(eulerAngles.z * f32.half);
            f32 c3 = MathS.Cos(eulerAngles.x * f32.half);
            f32 s1 = MathS.Sin(eulerAngles.y * f32.half);
            f32 s2 = MathS.Sin(eulerAngles.z * f32.half);
            f32 s3 = MathS.Sin(eulerAngles.x * f32.half);

            return new QuaternionS(
                s1 * c2 * c3 + c1 * s2 * s3,
                c1 * s2 * c3 - s1 * c2 * s3,
                c1 * c2 * s3 + s1 * s2 * c3,
                c1 * c2 * c3 - s1 * s2 * s3
            );
        }

        public static QuaternionS operator *(QuaternionS a, QuaternionS b)
        {
            return new QuaternionS(
                a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
                a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
                a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
                a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z
            );
        }

        public static Vector3S operator *(QuaternionS q, Vector3S v)
        {
            Vector3S u = new Vector3S(q.x, q.y, q.z);
            f32 s = q.w;

            return (f32)2 * Vector3S.Dot(u, v) * u
                 + (s * s - Vector3S.Dot(u, u)) * v
                 + (f32)2 * s * Vector3S.Cross(u, v);
        }

        public static QuaternionS operator /(QuaternionS q, f32 scalar)
        {
            return new QuaternionS(q.x / scalar, q.y / scalar, q.z / scalar, q.w / scalar);
        }

        public f32 MagnitudeSquared()
        {
            return (x * x) + (y * y) + (z * z) + (w * w);
        }

        public f32 Magnitude()
        {
            f32 magnitudeSquared = MagnitudeSquared();
            return magnitudeSquared > f32.zero ? MathS.Sqrt(magnitudeSquared) : f32.zero;
        }

        public static QuaternionS Inverse(QuaternionS q)
        {
            f32 magnitudeSquared = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
            return new QuaternionS(-q.x / magnitudeSquared, -q.y / magnitudeSquared, -q.z / magnitudeSquared, q.w / magnitudeSquared);
        }

        public QuaternionS Normalize()
        {
            f32 magnitude = Magnitude();

            if (magnitude > f32.zero)
            {
                f32 invMagnitude = f32.one / magnitude;
                return new QuaternionS(x * invMagnitude, y * invMagnitude, z * invMagnitude, w * invMagnitude);
            }

            return identity;
        }

        public QuaternionS Conjugate()
        {
            return new QuaternionS(-x, -y, -z, w);
        }

        public override string ToString()
        {
            return $"Quaternion({x}, {y}, {z}, {w})";
        }

        public override bool Equals(object? obj)
        {
            return obj is QuaternionS quaternion &&
                   x.Equals(quaternion.x) &&
                   y.Equals(quaternion.y) &&
                   z.Equals(quaternion.z) &&
                   w.Equals(quaternion.w);
        }

        public override int GetHashCode()
        {
            return HashCode.Combine(x, y, z, w);
        }

        public static bool operator ==(QuaternionS left, QuaternionS right) => left.Equals(right);

        public static bool operator !=(QuaternionS left, QuaternionS right) => !(left == right);
    }
}
