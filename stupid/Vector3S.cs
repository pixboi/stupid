using SoftFloat;
using System;

namespace stupid
{
    public struct Vector3S : IEquatable<Vector3S>
    {
        public sfloat x, y, z;

        public Vector3S(sfloat x, sfloat y, sfloat z)
        {
            this.x = x;
            this.y = y;
            this.z = z;
        }

        public Vector3S(float x, float y, float z)
        {
            this.x = (sfloat)x;
            this.y = (sfloat)y;
            this.z = (sfloat)z;
        }

        public static Vector3S operator +(Vector3S a, Vector3S b) => new Vector3S(a.x + b.x, a.y + b.y, a.z + b.z);
        public static Vector3S operator -(Vector3S a, Vector3S b) => new Vector3S(a.x - b.x, a.y - b.y, a.z - b.z);
        public static Vector3S operator *(Vector3S a, sfloat d) => new Vector3S(a.x * d, a.y * d, a.z * d);
        public static Vector3S operator /(Vector3S a, sfloat d) => new Vector3S(a.x / d, a.y / d, a.z / d);
        public static Vector3S operator *(sfloat d, Vector3S a) => new Vector3S(a.x * d, a.y * d, a.z * d);

        public static readonly Vector3S zero = new Vector3S(0f, 0f, 0f);
        public static readonly Vector3S one = new Vector3S(1f, 1f, 1f);
        public static readonly Vector3S up = new Vector3S(0f, 1f, 0f);
        public static readonly Vector3S down = new Vector3S(0f, -1f, 0f);
        public static readonly Vector3S left = new Vector3S(-1f, 0f, 0f);
        public static readonly Vector3S right = new Vector3S(1f, 0f, 0f);
        public static readonly Vector3S forward = new Vector3S(0f, 0f, 1f);
        public static readonly Vector3S back = new Vector3S(0f, 0f, -1f);

        public override string ToString() => $"({x}, {y}, {z})";
        public static sfloat Dot(Vector3S a, Vector3S b) => a.x * b.x + a.y * b.y + a.z * b.z;
        public static Vector3S Cross(Vector3S a, Vector3S b) => new Vector3S(
            a.y * b.z - a.z * b.y,
            a.z * b.x - a.x * b.z,
            a.x * b.y - a.y * b.x
        );

        public sfloat Magnitude() => libm.sqrtf(x * x + y * y + z * z);

        public sfloat MagnitudeSquared() => x * x + y * y + z * z;

        public static sfloat Distance(Vector3S a, Vector3S b) => (a - b).Magnitude();

        public Vector3S Normalize()
        {
            var mag = Magnitude();
            if (mag > sfloat.zero)
                return this / mag;
            return zero;
        }

        public bool Equals(Vector3S other) => x.Equals(other.x) && y.Equals(other.y) && z.Equals(other.z);

        public override bool Equals(object obj) => obj is Vector3S other && Equals(other);

        public override int GetHashCode() => HashCode.Combine(x, y, z);

        public static bool operator ==(Vector3S left, Vector3S right) => left.Equals(right);

        public static bool operator !=(Vector3S left, Vector3S right) => !(left == right);
    }
}
