using System;
using System.Runtime.CompilerServices;

namespace stupid.Maths
{
    public partial struct Vector3S : IEquatable<Vector3S>
    {
        // Static readonly vectors
        public static readonly Vector3S zero = new Vector3S(0f, 0f, 0f);
        public static readonly Vector3S one = new Vector3S(1f, 1f, 1f);
        public static readonly Vector3S up = new Vector3S(0f, 1f, 0f);
        public static readonly Vector3S down = new Vector3S(0f, -1f, 0f);
        public static readonly Vector3S left = new Vector3S(-1f, 0f, 0f);
        public static readonly Vector3S right = new Vector3S(1f, 0f, 0f);
        public static readonly Vector3S forward = new Vector3S(0f, 0f, 1f);
        public static readonly Vector3S back = new Vector3S(0f, 0f, -1f);

        #region LERPING

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3S Lerp(in Vector3S a, in Vector3S b, f32 t)
        {
            // Clamp the interpolation factor to the range [0, 1]
            t = MathS.Clamp(t, f32.zero, f32.one);

            // Perform linear interpolation
            return new Vector3S(
                a.x + (b.x - a.x) * t,
                a.y + (b.y - a.y) * t,
                a.z + (b.z - a.z) * t
            );
        }

        #endregion

        #region MAGNITUDE

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public f32 Magnitude()
        {
            // Use sqrMagnitude to avoid recomputing the squared magnitude
            var sqrMag = sqrMagnitude;
            return sqrMag > f32.zero ? MathS.Sqrt(sqrMag) : f32.zero;
        }

        // Squared magnitude, shifted for fractional bits
        public f32 sqrMagnitude
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                f32 result;
                result.rawValue = (x.rawValue * x.rawValue + y.rawValue * y.rawValue + z.rawValue * z.rawValue) >> f32.FractionalBits;
                return result;
            }
        }

        #endregion


        #region DOT PRODUCT

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static f32 Dot(in Vector3S a, in Vector3S b)
        {
            f32 result;

            long dotX = a.x.rawValue * b.x.rawValue;
            long dotY = a.y.rawValue * b.y.rawValue;
            long dotZ = a.z.rawValue * b.z.rawValue;

            result.rawValue = (dotX + dotY + dotZ) >> f32.FractionalBits;
            return result;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static f32 AbsDot(in Vector3S a, in Vector3S b)
        {
            f32 result = Dot(a, b);
            return MathS.Abs(result);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static long RawAbsDot(in Vector3S a, in Vector3S b)
        {
            long dotProduct = ((a.x.rawValue * b.x.rawValue) +
                               (a.y.rawValue * b.y.rawValue) +
                               (a.z.rawValue * b.z.rawValue)) >> f32.FractionalBits;

            // Use bitwise operation to get the absolute value
            long mask = dotProduct >> 63; // Create a mask based on the sign of dotProduct
            return (dotProduct + mask) ^ mask;
        }

        #endregion

        #region CROSS PRODUCT

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3S Cross(in Vector3S a, in Vector3S b)
        {
            Vector3S result;

            // Calculate the cross product and maintain fixed-point precision
            result.x.rawValue = (a.y.rawValue * b.z.rawValue - a.z.rawValue * b.y.rawValue) >> f32.FractionalBits;
            result.y.rawValue = (a.z.rawValue * b.x.rawValue - a.x.rawValue * b.z.rawValue) >> f32.FractionalBits;
            result.z.rawValue = (a.x.rawValue * b.y.rawValue - a.y.rawValue * b.x.rawValue) >> f32.FractionalBits;

            return result;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3S CrossAdd(in Vector3S a, in Vector3S b, in Vector3S parent)
        {
            Vector3S result;

            // Calculate the cross product and maintain fixed-point precision
            result.x.rawValue = parent.x.rawValue + ((a.y.rawValue * b.z.rawValue - a.z.rawValue * b.y.rawValue) >> f32.FractionalBits);
            result.y.rawValue = parent.y.rawValue + ((a.z.rawValue * b.x.rawValue - a.x.rawValue * b.z.rawValue) >> f32.FractionalBits);
            result.z.rawValue = parent.z.rawValue + ((a.x.rawValue * b.y.rawValue - a.y.rawValue * b.x.rawValue) >> f32.FractionalBits);

            return result;
        }


        #endregion

        #region DISTANCE

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static f32 Distance(in Vector3S a, in Vector3S b)
        {
            f32 result;
            result = (a - b).Magnitude();
            return result;
        }


        #endregion

        #region MIN/MAX/ABS

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3S Min(in Vector3S a, in Vector3S b) => new Vector3S(
            MathS.Min(a.x, b.x),
            MathS.Min(a.y, b.y),
            MathS.Min(a.z, b.z)
        );

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3S Max(in Vector3S a, in Vector3S b) => new Vector3S(
            MathS.Max(a.x, b.x),
            MathS.Max(a.y, b.y),
            MathS.Max(a.z, b.z)
        );

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3S Abs(in Vector3S a)
        {
            Vector3S result;

            result.x.rawValue = (a.x.rawValue >= 0) ? a.x.rawValue : -a.x.rawValue;
            result.y.rawValue = (a.y.rawValue >= 0) ? a.y.rawValue : -a.y.rawValue;
            result.z.rawValue = (a.z.rawValue >= 0) ? a.z.rawValue : -a.z.rawValue;

            return result;
        }


        #endregion

        #region NORMALIZATION

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

        #endregion

        #region CLAMPING

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3S Clamp(in Vector3S v, f32 min, f32 max) => new Vector3S(
            MathS.Clamp(v.x, min, max),
            MathS.Clamp(v.y, min, max),
            MathS.Clamp(v.z, min, max)
        );

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3S ClampMagnitude(in Vector3S v, f32 min, f32 max)
        {
            f32 sqrMagnitude = v.sqrMagnitude;

            if (sqrMagnitude > max * max)
            {
                // Clamp to max magnitude
                f32 magnitude = MathS.Sqrt(sqrMagnitude);
                f32 scale = max / magnitude;
                return v * scale;
            }
            else if (sqrMagnitude < min * min && sqrMagnitude > f32.zero)
            {
                // Clamp to min magnitude, ensuring we don't divide by zero
                f32 magnitude = MathS.Sqrt(sqrMagnitude);
                f32 scale = min / magnitude;
                return v * scale;
            }

            return v; // Already within the desired magnitude range
        }

        #endregion

        #region ROTATION

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Vector3S Rotate(in QuaternionS rotation)
        {
            var qVector = new QuaternionS(x, y, z, f32.zero);
            var qConjugate = rotation.Conjugate();
            var qResult = rotation * qVector * qConjugate;
            return new Vector3S(qResult.x, qResult.y, qResult.z);
        }

        #endregion
    }
}
