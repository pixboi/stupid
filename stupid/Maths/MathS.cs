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
            long fractionalPart = raw & ((1L << 16) - 1);
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

            term *= -square / f32.FromRaw(6L << 16);  // -value^3 / 3!
            result += term;
            term *= -square / f32.FromRaw(20L << 16); // +value^5 / 5!
            result += term;
            term *= -square / f32.FromRaw(42L << 16); // -value^7 / 7!
            result += term;

            return result;
        }

        // Fixed-point cosine approximation using Taylor series expansion
        public static f32 Cos(f32 value)
        {
            f32 result = f32.one;
            f32 term = f32.one;
            f32 square = value * value;

            term *= -square / f32.FromRaw(2L << 16);  // -value^2 / 2!
            result += term;
            term *= -square / f32.FromRaw(24L << 16); // +value^4 / 4!
            result += term;
            term *= -square / f32.FromRaw(120L << 16); // -value^6 / 6!
            result += term;

            return result;
        }

        public static f32 Sqrt(f32 value)
        {
            if (value < f32.zero) throw new ArgumentOutOfRangeException("Cannot compute square root of a negative value.");
            if (value == f32.zero) return f32.zero;

            f32 min = f32.zero;
            f32 max = value > f32.one ? value : f32.one;
            f32 mid;

            for (int i = 0; i < 8; i++) // Number of iterations can be adjusted
            {
                mid = (min + max) / f32.two;
                f32 midSquared = mid * mid;

                if (midSquared == value)
                {
                    return mid;
                }
                else if (midSquared < value)
                {
                    min = mid;
                }
                else
                {
                    max = mid;
                }
            }

            return (min + max) / f32.two;
        }
    }
}
