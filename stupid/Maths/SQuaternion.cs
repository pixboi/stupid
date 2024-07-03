using SoftFloat;
using stupid.Maths;

namespace stupid.Maths
{
    public struct SQuaternion
    {
        public sfloat x;
        public sfloat y;
        public sfloat z;
        public sfloat w;

        public static readonly SQuaternion Identity = new SQuaternion(sfloat.zero, sfloat.zero, sfloat.zero, sfloat.one);

        public SQuaternion(sfloat x, sfloat y, sfloat z, sfloat w)
        {
            this.x = x;
            this.y = y;
            this.z = z;
            this.w = w;
        }

        public static SQuaternion FromAxisAngle(Vector3S axis, sfloat angle)
        {
            sfloat halfAngle = angle * (sfloat)0.5f;
            sfloat sinHalfAngle = libm.sinf(halfAngle);
            return new SQuaternion(
                axis.x * sinHalfAngle,
                axis.y * sinHalfAngle,
                axis.z * sinHalfAngle,
                libm.cosf(halfAngle)
            );
        }

        public static SQuaternion operator *(SQuaternion a, SQuaternion b)
        {
            return new SQuaternion(
                a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
                a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
                a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
                a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z
            );
        }

        public static Vector3S operator *(SQuaternion q, Vector3S v)
        {
            Vector3S u = new Vector3S(q.x, q.y, q.z);
            sfloat s = q.w;

            return (sfloat)2.0f * Vector3S.Dot(u, v) * u
                 + (s * s - Vector3S.Dot(u, u)) * v
                 + (sfloat)2.0f * s * Vector3S.Cross(u, v);
        }

        public sfloat Magnitude()
        {
            return libm.sqrtf(x * x + y * y + z * z + w * w);
        }

        public SQuaternion Normalize()
        {
            sfloat magnitude = Magnitude();
            return new SQuaternion(x / magnitude, y / magnitude, z / magnitude, w / magnitude);
        }

        public SQuaternion Conjugate()
        {
            return new SQuaternion(-x, -y, -z, w);
        }

        public static SQuaternion FromAngularVelocity(Vector3S angularVelocity)
        {
            sfloat angle = angularVelocity.Magnitude();
            if (angle < (sfloat)0.0001f)
            {
                return Identity;
            }

            Vector3S axis = angularVelocity / angle;
            return FromAxisAngle(axis, angle);
        }

        public override string ToString()
        {
            return $"Quaternion({x}, {y}, {z}, {w})";
        }
    }
}
