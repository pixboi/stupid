using System;

namespace stupid.Maths
{
    public static class MathS
    {
        public static f32 Min(f32 a, f32 b) => a < b ? a : b;
        public static f32 Max(f32 a, f32 b) => a > b ? a : b;

        public static f32 Clamp(f32 value, f32 min, f32 max)
        {
            return Max(min, Min(max, value));
        }

        public static f32 Floor(f32 value)
        {
            long raw = value.ToRaw();
            long fractionalPart = raw & ((1L << f32.FractionalBits) - 1);
            return f32.FromRaw(raw - fractionalPart);
        }

        public static f32 Abs(f32 value)
        {
            return value < f32.zero ? -value : value;
        }

        // Fixed-point sine approximation using Taylor series expansion
        public static f32 Sin(f32 value)
        {
            f32 result = value;
            f32 term = value;
            f32 square = value * value;
            const int iterations = 3; // Number of iterations can be adjusted

            for (int i = 3; i <= 2 * iterations + 1; i += 2)
            {
                term *= -square / f32.FromRaw((i - 1) << f32.FractionalBits);
                result += term;
            }

            return result;
        }

        // Fixed-point cosine approximation using Taylor series expansion
        public static f32 Cos(f32 value)
        {
            f32 result = f32.one;
            f32 term = f32.one;
            f32 square = value * value;
            const int iterations = 3; // Number of iterations can be adjusted

            for (int i = 2; i <= 2 * iterations; i += 2)
            {
                term *= -square / f32.FromRaw((i - 1) << f32.FractionalBits);
                result += term;
            }

            return result;
        }

        public static f32 Sqrt(f32 value)
        {
            if (value < f32.zero) throw new ArgumentOutOfRangeException(nameof(value), "Cannot compute square root of a negative value.");
            if (value == f32.zero) return f32.zero;

            f32 x = value > f32.one ? value : f32.one; // Initial guess
            const int iterations = 10; // Number of iterations can be adjusted

            for (int i = 0; i < iterations; i++)
            {
                x = (x + value / x) / f32.two;
            }

            return x;
        }

        public static f32 Exp(f32 value)
        {
            f32 result = f32.one;
            f32 term = f32.one;
            const int iterations = 3; // Number of iterations can be adjusted

            for (int i = 1; i <= iterations; i++)
            {
                term *= value / f32.FromRaw(i << f32.FractionalBits);
                result += term;
            }

            return result;
        }

        public static f32 Log(f32 value)
        {
            if (value <= f32.zero) throw new ArgumentOutOfRangeException(nameof(value), "Logarithm of non-positive value is undefined.");

            f32 result = f32.zero;
            f32 x = (value - f32.one) / (value + f32.one);
            f32 term = x;
            f32 xSquared = x * x;
            const int iterations = 3; // Number of iterations can be adjusted

            for (int i = 1; i <= iterations; i += 2)
            {
                result += term / f32.FromRaw(i << f32.FractionalBits);
                term *= xSquared;
            }

            return result * f32.two;
        }

        public static f32 Pow(f32 baseValue, f32 exponent)
        {
            return Exp(exponent * Log(baseValue));
        }
    }
}