using System;
using System.Runtime.CompilerServices;

namespace stupid.Maths
{
    public struct Vector3S : IEquatable<Vector3S>
    {
        public f32 x, y, z;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Vector3S(f32 x, f32 y, f32 z)
        {
            this.x = x;
            this.y = y;
            this.z = z;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Vector3S(f32 x)
        {
            this.x = x;
            this.y = x;
            this.z = x;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Vector3S(float x, float y, float z)
        {
            this.x = f32.FromFloat(x);
            this.y = f32.FromFloat(y);
            this.z = f32.FromFloat(z);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Vector3S(float x)
        {
            this.x = f32.FromFloat(x);
            this.y = this.x;
            this.z = this.x;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3S operator +(Vector3S a, Vector3S b) => new Vector3S(a.x + b.x, a.y + b.y, a.z + b.z);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3S operator -(Vector3S a, Vector3S b) => new Vector3S(a.x - b.x, a.y - b.y, a.z - b.z);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3S operator -(Vector3S a) => new Vector3S(-a.x, -a.y, -a.z);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3S operator *(Vector3S a, f32 d) => new Vector3S(a.x * d, a.y * d, a.z * d);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3S operator /(Vector3S a, f32 d) => new Vector3S(a.x / d, a.y / d, a.z / d);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3S operator *(f32 d, Vector3S a) => new Vector3S(a.x * d, a.y * d, a.z * d);

        public static readonly Vector3S zero = new Vector3S(0f, 0f, 0f);
        public static readonly Vector3S one = new Vector3S(1f, 1f, 1f);
        public static readonly Vector3S up = new Vector3S(0f, 1f, 0f);
        public static readonly Vector3S down = new Vector3S(0f, -1f, 0f);
        public static readonly Vector3S left = new Vector3S(-1f, 0f, 0f);
        public static readonly Vector3S right = new Vector3S(1f, 0f, 0f);
        public static readonly Vector3S forward = new Vector3S(0f, 0f, 1f);
        public static readonly Vector3S back = new Vector3S(0f, 0f, -1f);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public override string ToString() => $"({x}, {y}, {z})";

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static f32 Dot(Vector3S a, Vector3S b) => a.x * b.x + a.y * b.y + a.z * b.z;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public f32 Dot(Vector3S b) => Dot(this, b);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3S ProjectPointOnPlane(Vector3S point, Vector3S planeNormal, Vector3S planePoint)
        {
            Vector3S toPoint = point - planePoint;
            f32 distance = Vector3S.Dot(toPoint, planeNormal);
            return point - planeNormal * distance;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3S Cross(Vector3S a, Vector3S b) => new Vector3S(
            a.y * b.z - a.z * b.y,
            a.z * b.x - a.x * b.z,
            a.x * b.y - a.y * b.x
        );

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Vector3S Cross(Vector3S b) => Cross(this, b);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public f32 Magnitude()
        {
            f32 magnitudeSquared = sqrMagnitude;
            return magnitudeSquared > f32.zero ? MathS.Sqrt(magnitudeSquared) : f32.zero;
        }

        public f32 sqrMagnitude => (x * x + y * y + z * z);
        public f32 magnitude => Magnitude();

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static f32 Distance(Vector3S a, Vector3S b) => (a - b).Magnitude();

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static f32 DistanceSquared(Vector3S a, Vector3S b) => (a - b).sqrMagnitude;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Vector3S Normalize()
        {
            f32 mag = Magnitude();
            return mag > f32.zero ? this / mag : zero;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Vector3S NormalizeWithMagnitude(out f32 mag)
        {
            mag = Magnitude();
            return mag > f32.zero ? this / mag : zero;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3S Min(Vector3S a, Vector3S b) => new Vector3S(
            MathS.Min(a.x, b.x),
            MathS.Min(a.y, b.y),
            MathS.Min(a.z, b.z)
        );

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3S Max(Vector3S a, Vector3S b) => new Vector3S(
            MathS.Max(a.x, b.x),
            MathS.Max(a.y, b.y),
            MathS.Max(a.z, b.z)
        );

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3S Abs(Vector3S a) => new Vector3S(MathS.Abs(a.x), MathS.Abs(a.y), MathS.Abs(a.z));

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Vector3S Abs() => Abs(this);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Vector3S Clamp(f32 min, f32 max) => Clamp(this, min, max);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3S Clamp(Vector3S v, f32 min, f32 max) => new Vector3S(
            MathS.Clamp(v.x, min, max),
            MathS.Clamp(v.y, min, max),
            MathS.Clamp(v.z, min, max)
        );

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Vector3S ClampMagnitude(f32 min, f32 max) => ClampMagnitude(this, min, max);

        public static Vector3S ClampMagnitude(Vector3S v, f32 min, f32 max)
        {
            f32 sqrMagnitude = v.sqrMagnitude;
            if (sqrMagnitude > max * max)
            {
                f32 scale = max / MathS.Sqrt(sqrMagnitude);
                return v * scale;
            }
            else if (sqrMagnitude < min * min)
            {
                f32 scale = min / MathS.Sqrt(sqrMagnitude);
                return v * scale;
            }
            return v;
        }

        public Vector3S Rotate(QuaternionS rotation)
        {
            var qVector = new QuaternionS(x, y, z, f32.zero);
            var qConjugate = rotation.Conjugate();
            var qResult = rotation * qVector * qConjugate;
            return new Vector3S(qResult.x, qResult.y, qResult.z);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Equals(Vector3S other) => x.Equals(other.x) && y.Equals(other.y) && z.Equals(other.z);

        public override bool Equals(object obj) => obj is Vector3S other && Equals(other);

        public override int GetHashCode() => HashCode.Combine(x, y, z);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool operator ==(Vector3S left, Vector3S right) => left.Equals(right);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool operator !=(Vector3S left, Vector3S right) => !(left == right);
    }
}
