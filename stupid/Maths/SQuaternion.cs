namespace stupid.Maths
{
    public struct SQuaternion
    {
        public f32 x;
        public f32 y;
        public f32 z;
        public f32 w;

        public static readonly SQuaternion Identity = new SQuaternion(f32.zero, f32.zero, f32.zero, f32.one);

        public SQuaternion(f32 x, f32 y, f32 z, f32 w)
        {
            this.x = x;
            this.y = y;
            this.z = z;
            this.w = w;
        }

        public SQuaternion(float x, float y, float z, float w)
        {
            this.x = f32.FromFloat(x);
            this.y = f32.FromFloat(y);
            this.z = f32.FromFloat(z);
            this.w = f32.FromFloat(w);
        }

        public static SQuaternion FromAxisAngle(Vector3S axis, f32 angle)
        {
            f32 halfAngle = angle * (f32)0.5f;
            f32 sinHalfAngle = MathS.Sin(halfAngle);
            return new SQuaternion(
                axis.x * sinHalfAngle,
                axis.y * sinHalfAngle,
                axis.z * sinHalfAngle,
                MathS.Cos(halfAngle)
            );
        }

        public static SQuaternion FromEulerAngles(Vector3S eulerAngles)
        {
            f32 c1 = MathS.Cos(eulerAngles.y * f32.half);
            f32 c2 = MathS.Cos(eulerAngles.z * f32.half);
            f32 c3 = MathS.Cos(eulerAngles.x * f32.half);
            f32 s1 = MathS.Sin(eulerAngles.y * f32.half);
            f32 s2 = MathS.Sin(eulerAngles.z * f32.half);
            f32 s3 = MathS.Sin(eulerAngles.x * f32.half);

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
            f32 s = q.w;

            return (f32)2.0f * Vector3S.Dot(u, v) * u
                 + (s * s - Vector3S.Dot(u, u)) * v
                 + (f32)2.0f * s * Vector3S.Cross(u, v);
        }

        public f32 Magnitude()
        {
            return MathS.Sqrt(x * x + y * y + z * z + w * w);
        }

        public SQuaternion Normalize()
        {
            f32 magnitude = Magnitude();
            if (magnitude > f32.FromFloat(1e-6f))
            {
                f32 invMag = f32.one / magnitude;
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
