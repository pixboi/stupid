using System;
using System.Runtime.CompilerServices;

namespace stupid.Maths
{
    public static class MathS
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static f32 Min(in f32 a, in f32 b) => a < b ? a : b;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static f32 Max(in f32 a, in f32 b) => a > b ? a : b;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static f32 Avg(in f32 a, in f32 b) => (a + b) * f32.half;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static f32 Clamp(in f32 value, in f32 min, f32 max) => Max(min, Min(max, value));

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static f32 Sign(in f32 value) => value < f32.zero ? f32.negativeOne : f32.one;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static f32 Floor(in f32 value)
        {
            long raw = value.ToRaw();
            long fractionalPart = raw & ((1L << f32.FractionalBits) - 1);
            return new f32(raw - fractionalPart);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static f32 Abs(in f32 value) => value < f32.zero ? -value : value;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static f32 FastAbs(in f32 value)
        {
            long mask = value.rawValue >> 63; // Creates a mask of all 1s if the value is negative, all 0s otherwise
            long absValue = (value.rawValue + mask) ^ mask; // If negative, it flips the sign bits
            return new f32(absValue);
        }


        // Fixed-point sine approximation using Taylor series expansion
        public static f32 Sin(in f32 value)
        {
            f32 result = value;
            f32 term = value;
            f32 square = value * value;

            // Taylor series terms
            term *= -square / new f32(6L << f32.FractionalBits);
            result += term;
            term *= -square / new f32(20L << f32.FractionalBits);
            result += term;
            term *= -square / new f32(42L << f32.FractionalBits);
            result += term;
            term *= -square / new f32(72L << f32.FractionalBits);
            result += term;

            return result;
        }

        // Fixed-point cosine approximation using Taylor series expansion
        public static f32 Cos(in f32 value)
        {
            f32 result = f32.one;
            f32 term = f32.one;
            f32 square = value * value;

            // Taylor series terms
            term *= -square / new f32(2L << f32.FractionalBits);
            result += term;
            term *= -square / new f32(12L << f32.FractionalBits);
            result += term;
            term *= -square / new f32(30L << f32.FractionalBits);
            result += term;
            term *= -square / new f32(56L << f32.FractionalBits);
            result += term;

            return result;
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static f32 Sqrt(in f32 value)
        {
            if (value < f32.zero)
                throw new ArgumentOutOfRangeException(nameof(value), "Cannot compute square root of a negative value.");
            if (value == f32.zero)
                return f32.zero;

            long rawValue = value.rawValue;
            long xRaw = rawValue > f32.one.rawValue ? rawValue : f32.one.rawValue; // Initial guess in raw form
            const int iterations = 16; // Number of iterations can be adjusted

            for (int i = 0; i < iterations; i++)
            {
                long div = (rawValue << f32.FractionalBits) / xRaw; // Properly grouped division
                xRaw = (xRaw + div) >> 1; // (x + value / x) / 2
            }

            return new f32(xRaw);
        }



        public static f32 Exp(in f32 value)
        {
            f32 result = f32.one;
            f32 term = f32.one;
            const int iterations = 10; // Adjust the number of iterations

            for (int i = 1; i <= iterations; i++)
            {
                term *= value / new f32(i << f32.FractionalBits);
                result += term;
            }

            return result;
        }

        public static f32 Log(in f32 value)
        {
            if (value <= f32.zero) throw new ArgumentOutOfRangeException(nameof(value), "Logarithm of non-positive value is undefined.");

            f32 result = f32.zero;
            f32 x = (value - f32.one) / (value + f32.one);
            f32 term = x;
            f32 xSquared = x * x;
            const int iterations = 10; // Adjust the number of iterations

            for (int i = 1; i <= iterations; i += 2)
            {
                result += term / new f32(i << f32.FractionalBits);
                term *= xSquared;
            }

            return result * f32.two;
        }

        public static f32 Pow(in f32 baseValue, in f32 exponent)
        {
            return Exp(exponent * Log(baseValue));
        }
    }
}
