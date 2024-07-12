using System;
using System.Runtime.CompilerServices;

namespace stupid.Maths
{
    public struct QuaternionS : IEquatable<QuaternionS>
    {
        public readonly f32 x;
        public readonly f32 y;
        public readonly f32 z;
        public readonly f32 w;

        public static readonly QuaternionS identity = new QuaternionS(f32.zero, f32.zero, f32.zero, f32.one);
        public static readonly QuaternionS zero = new QuaternionS(f32.zero, f32.zero, f32.zero, f32.zero);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public QuaternionS(f32 x, f32 y, f32 z, f32 w)
        {
            this.x = x;
            this.y = y;
            this.z = z;
            this.w = w;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public QuaternionS(float x, float y, float z, float w)
        {
            this.x = f32.FromFloat(x);
            this.y = f32.FromFloat(y);
            this.z = f32.FromFloat(z);
            this.w = f32.FromFloat(w);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
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

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
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

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static QuaternionS operator *(QuaternionS a, QuaternionS b)
        {
            return new QuaternionS(
                a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
                a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
                a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
                a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z
            );
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3S operator *(QuaternionS q, Vector3S v)
        {
            Vector3S u = new Vector3S(q.x, q.y, q.z);
            f32 s = q.w;

            return f32.two * Vector3S.Dot(u, v) * u
                 + (s * s - Vector3S.Dot(u, u)) * v
                 + f32.two * s * Vector3S.Cross(u, v);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static QuaternionS operator /(QuaternionS q, f32 scalar)
        {
            return new QuaternionS(q.x / scalar, q.y / scalar, q.z / scalar, q.w / scalar);
        }

        public f32 SqrMagnitude => (x * x) + (y * y) + (z * z) + (w * w);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public f32 Magnitude()
        {
            f32 magnitudeSquared = SqrMagnitude;
            return magnitudeSquared > f32.zero ? MathS.Sqrt(magnitudeSquared) : f32.zero;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static QuaternionS Inverse(QuaternionS q)
        {
            f32 magSq = q.SqrMagnitude;
            if (magSq > f32.epsilon)
            {
                f32 invMagSq = f32.one / magSq;
                return new QuaternionS(-q.x * invMagSq, -q.y * invMagSq, -q.z * invMagSq, q.w * invMagSq);
            }
            // Handle zero magnitude quaternion
            return QuaternionS.identity;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public QuaternionS Inverse() => Inverse(this);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
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

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public QuaternionS Conjugate()
        {
            return new QuaternionS(-x, -y, -z, w);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public override string ToString()
        {
            return $"Quaternion({x}, {y}, {z}, {w})";
        }

        public override bool Equals(object obj)
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

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Equals(QuaternionS other)
        {
            return x.Equals(other.x) && y.Equals(other.y) && z.Equals(other.z) && w.Equals(other.w);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool operator ==(QuaternionS left, QuaternionS right) => left.Equals(right);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool operator !=(QuaternionS left, QuaternionS right) => !(left == right);
    }
}
