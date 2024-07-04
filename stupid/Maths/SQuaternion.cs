using SoftFloat;

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

        public SQuaternion(float x, float y, float z, float w)
        {
            this.x = (sfloat)x;
            this.y = (sfloat)y;
            this.z = (sfloat)z;
            this.w = (sfloat)w;
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

        public static SQuaternion FromEulerAngles(Vector3S eulerAngles)
        {
            sfloat c1 = libm.cosf(eulerAngles.y * (sfloat)0.5f);
            sfloat c2 = libm.cosf(eulerAngles.z * (sfloat)0.5f);
            sfloat c3 = libm.cosf(eulerAngles.x * (sfloat)0.5f);
            sfloat s1 = libm.sinf(eulerAngles.y * (sfloat)0.5f);
            sfloat s2 = libm.sinf(eulerAngles.z * (sfloat)0.5f);
            sfloat s3 = libm.sinf(eulerAngles.x * (sfloat)0.5f);

            return new SQuaternion(
                s1 * c2 * c3 + c1 * s2 * s3,
                c1 * s2 * c3 - s1 * c2 * s3,
                c1 * c2 * s3 + s1 * s2 * c3,
                c1 * c2 * c3 - s1 * s2 * s3
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
            if (magnitude > sfloat.Epsilon)
            {
                sfloat invMag = sfloat.one / magnitude;
                return new SQuaternion(x * invMag, y * invMag, z * invMag, w * invMag);
            }
            return Identity;
        }

        public SQuaternion Conjugate()
        {
            return new SQuaternion(-x, -y, -z, w);
        }

        public override string ToString()
        {
            return $"Quaternion({x}, {y}, {z}, {w})";
        }
    }
}
