using System;

namespace stupid.Maths
{
    public struct Vector3S : IEquatable<Vector3S>
    {
        public f32 x, y, z;
        public Vector3S(f32 x, f32 y, f32 z)
        {
            this.x = x;
            this.y = y;
            this.z = z;
        }

        public Vector3S(f32 x)
        {
            this.x = x;
            this.y = x;
            this.z = x;
        }

        public Vector3S(float x, float y, float z)
        {
            this.x = f32.FromFloat(x);
            this.y = f32.FromFloat(y);
            this.z = f32.FromFloat(z);
        }

        public Vector3S(float x)
        {
            this.x = f32.FromFloat(x);
            this.y = this.x;
            this.z = this.x;
        }

        public static Vector3S operator +(Vector3S a, Vector3S b) => new Vector3S(a.x + b.x, a.y + b.y, a.z + b.z);
        public static Vector3S operator -(Vector3S a, Vector3S b) => new Vector3S(a.x - b.x, a.y - b.y, a.z - b.z);
        public static Vector3S operator -(Vector3S a) => new Vector3S(-a.x, -a.y, -a.z);
        public static Vector3S operator *(Vector3S a, f32 d) => new Vector3S(a.x * d, a.y * d, a.z * d);
        public static Vector3S operator /(Vector3S a, f32 d) => new Vector3S(a.x / d, a.y / d, a.z / d);
        public static Vector3S operator *(f32 d, Vector3S a) => new Vector3S(a.x * d, a.y * d, a.z * d);

        public static readonly Vector3S zero = new Vector3S(0f, 0f, 0f);
        public static readonly Vector3S one = new Vector3S(1f, 1f, 1f);
        public static readonly Vector3S up = new Vector3S(0f, 1f, 0f);
        public static readonly Vector3S down = new Vector3S(0f, -1f, 0f);
        public static readonly Vector3S left = new Vector3S(-1f, 0f, 0f);
        public static readonly Vector3S right = new Vector3S(1f, 0f, 0f);
        public static readonly Vector3S forward = new Vector3S(0f, 0f, 1f);
        public static readonly Vector3S back = new Vector3S(0f, 0f, -1f);

        public override string ToString() => $"({x}, {y}, {z})";
        public static f32 Dot(Vector3S a, Vector3S b) => a.x * b.x + a.y * b.y + a.z * b.z;
        public static Vector3S Cross(Vector3S a, Vector3S b) => new Vector3S(
            a.y * b.z - a.z * b.y,
            a.z * b.x - a.x * b.z,
            a.x * b.y - a.y * b.x
        );

        public f32 Magnitude()
        {
            f32 magnitudeSquared = SqrMagnitude;
            return magnitudeSquared > f32.zero ? MathS.Sqrt(magnitudeSquared) : f32.zero;
        }

        public f32 SqrMagnitude
        {
            get
            {
                return (x * x + y * y + z * z);
            }
        }

        public static f32 Distance(Vector3S a, Vector3S b) => (a - b).Magnitude();
        public static f32 DistanceSquared(Vector3S a, Vector3S b) => (a - b).SqrMagnitude;

        public Vector3S Normalize()
        {
            var mag = Magnitude();
            if (mag > f32.zero)
                return this / mag;
            return zero;
        }

        public Vector3S NormalizeWithMagnitude(out f32 mag)
        {
            mag = Magnitude();
            if (mag > f32.zero)
                return this / mag;
            return zero;
        }

        public static Vector3S Min(Vector3S a, Vector3S b)
        {
            return new Vector3S(
                MathS.Min(a.x, b.x),
                MathS.Min(a.y, b.y),
                MathS.Min(a.z, b.z)
            );
        }

        public static Vector3S Max(Vector3S a, Vector3S b)
        {
            return new Vector3S(
                MathS.Max(a.x, b.x),
                MathS.Max(a.y, b.y),
                MathS.Max(a.z, b.z)
            );
        }

        public Vector3S Clamp(f32 min, f32 max) => Clamp(this, min, max);
        public static Vector3S Clamp(Vector3S v, f32 min, f32 max)
        {
            return new Vector3S(MathS.Clamp(v.x, min, max), MathS.Clamp(v.y, min, max), MathS.Clamp(v.z, min, max));
        }

        public Vector3S ClampMagnitude(f32 min, f32 max) => ClampMagnitude(this, min, max);
        public static Vector3S ClampMagnitude(Vector3S v, f32 min, f32 max)
        {
            f32 sqrMagnitude = v.SqrMagnitude;
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


        public bool Equals(Vector3S other) => x.Equals(other.x) && y.Equals(other.y) && z.Equals(other.z);

        public override bool Equals(object obj) => obj is Vector3S other && Equals(other);

        public override int GetHashCode() => HashCode.Combine(x, y, z);

        public static bool operator ==(Vector3S left, Vector3S right) => left.Equals(right);

        public static bool operator !=(Vector3S left, Vector3S right) => !(left == right);
    }
}
