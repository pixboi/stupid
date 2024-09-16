using System;
using System.Runtime.CompilerServices;

namespace stupid.Maths
{
    public partial struct Vector3S : IEquatable<Vector3S>
    {
        #region MAGNITUDE

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public f32 Magnitude()
        {
            // Use sqrMagnitude to avoid recomputing the squared magnitude
            var sqrMag = sqrMagnitude;
            return sqrMag > f32.zero ? MathS.Sqrt(sqrMag) : f32.zero;
        }

        // Combined rawSqrMagnitude and sqrMagnitude into one method for efficiency
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private long ComputeRawSqrMagnitude()
        {
            long xx = x.rawValue * x.rawValue;
            long yy = y.rawValue * y.rawValue;
            long zz = z.rawValue * z.rawValue;
            return xx + yy + zz;
        }

        // Raw squared magnitude (in raw long form)
        public long rawSqrMagnitude
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get => ComputeRawSqrMagnitude();
        }

        // Squared magnitude, shifted for fractional bits
        public f32 sqrMagnitude
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                long rawSqrMag = ComputeRawSqrMagnitude();
                return new f32(rawSqrMag >> f32.FractionalBits);
            }
        }

        #endregion


        #region DOT PRODUCT

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static f32 Dot(in Vector3S a, in Vector3S b)
        {
            long dotX = a.x.rawValue * b.x.rawValue;
            long dotY = a.y.rawValue * b.y.rawValue;
            long dotZ = a.z.rawValue * b.z.rawValue;
            return new f32((dotX + dotY + dotZ) >> f32.FractionalBits);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static f32 AbsDot(in Vector3S a, in Vector3S b) => new f32(RawAbsDot(a, b));

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
            // Calculate the cross product and maintain fixed-point precision
            long xRaw = (a.y.rawValue * b.z.rawValue - a.z.rawValue * b.y.rawValue);
            long yRaw = (a.z.rawValue * b.x.rawValue - a.x.rawValue * b.z.rawValue);
            long zRaw = (a.x.rawValue * b.y.rawValue - a.y.rawValue * b.x.rawValue);

            // Return the new Vector3S with properly scaled values
            return new Vector3S(
                new f32(xRaw >> f32.FractionalBits),  // Scale the result to fixed-point
                new f32(yRaw >> f32.FractionalBits),
                new f32(zRaw >> f32.FractionalBits)
            );
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void CrossInPlace(in Vector3S b)
        {
            // Compute the cross product in fixed-point
            long xRaw = (this.y.rawValue * b.z.rawValue - this.z.rawValue * b.y.rawValue);
            long yRaw = (this.z.rawValue * b.x.rawValue - this.x.rawValue * b.z.rawValue);
            long zRaw = (this.x.rawValue * b.y.rawValue - this.y.rawValue * b.x.rawValue);

            // Update the current vector with the scaled cross product
            this.x.rawValue = xRaw >> f32.FractionalBits;  // Scale the result
            this.y.rawValue = yRaw >> f32.FractionalBits;
            this.z.rawValue = zRaw >> f32.FractionalBits;
        }

        #endregion

        #region DISTANCE

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static f32 Distance(in Vector3S a, in Vector3S b) => (a - b).Magnitude();

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static f32 DistanceSquared(in Vector3S a, in Vector3S b) => (a - b).sqrMagnitude;

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
        public static Vector3S Abs(in Vector3S a) => new Vector3S(
            MathS.Abs(a.x),
            MathS.Abs(a.y),
            MathS.Abs(a.z)
        );

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

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Vector3S NormalizeInPlace()
        {
            f32 mag = Magnitude();
            if (mag > f32.zero)
            {
                this.Divide(mag);
            }
            return this;
        }

        #endregion

        #region CLAMPING

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3S Clamp(in Vector3S v, f32 min, f32 max) => new Vector3S(
            MathS.Clamp(v.x, min, max),
            MathS.Clamp(v.y, min, max),
            MathS.Clamp(v.z, min, max)
        );

        public void ClampEpsilon()
        {
            if (this.x <= f32.epsilon) this.x = f32.zero;
            if (this.y <= f32.epsilon) this.y = f32.zero;
            if (this.z <= f32.epsilon) this.z = f32.zero;
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Vector3S ClampMagnitude(f32 min, f32 max) => ClampMagnitude(this, min, max);

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
