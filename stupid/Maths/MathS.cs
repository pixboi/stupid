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
            int raw = value.ToRaw();
            int fractionalPart = raw & ((1 << 16) - 1);
            return f32.FromRaw(raw - fractionalPart);
        }

        public static f32 Ceiling(f32 value)
        {
            int raw = value.ToRaw();
            int fractionalPart = raw & ((1 << 16) - 1);
            if (fractionalPart == 0)
                return value;
            return f32.FromRaw(raw + ((1 << 16) - fractionalPart));
        }

        public static f32 Round(f32 value)
        {
            int raw = value.ToRaw();
            int fractionalPart = raw & ((1 << 16) - 1);
            int integerPart = raw >> 16;
            if (fractionalPart >= 32768)
                integerPart += 1;
            return f32.FromRaw(integerPart << 16);
        }

        public static f32 Abs(f32 value)
        {
            return value < f32.zero ? -value : value;
        }

        // Simple square root approximation using the Newton-Raphson method
        public static f32 Sqrt(f32 value)
        {
            if (value < f32.zero) throw new ArgumentOutOfRangeException("Cannot compute square root of a negative value.");
            f32 x = value;
            f32 y = value / f32.FromRaw(2 << 16); // Start with an estimate of half the value
            int iterations = 10;
            for (int i = 0; i < iterations; i++)
            {
                y = (y + x / y) / f32.FromRaw(2 << 16);
            }
            return y;
        }

        // Trigonometric functions using Taylor series approximations
        public static f32 Sin(f32 value)
        {
            f32 x = value;
            f32 x2 = x * x;
            f32 term = x;
            f32 result = term;
            term *= -x2 / f32.FromRaw(6 << 16); result += term;
            term *= -x2 / f32.FromRaw(20 << 16); result += term;
            term *= -x2 / f32.FromRaw(42 << 16); result += term;
            term *= -x2 / f32.FromRaw(72 << 16); result += term;
            return result;
        }

        public static f32 Cos(f32 value)
        {
            f32 x = value;
            f32 x2 = x * x;
            f32 term = f32.one;
            f32 result = term;
            term *= -x2 / f32.FromRaw(2 << 16); result += term;
            term *= -x2 / f32.FromRaw(24 << 16); result += term;
            term *= -x2 / f32.FromRaw(120 << 16); result += term;
            term *= -x2 / f32.FromRaw(720 << 16); result += term;
            term *= -x2 / f32.FromRaw(5040 << 16); result += term;
            return result;
        }

        public static f32 Tan(f32 value)
        {
            return Sin(value) / Cos(value);
        }

        // Exponential function using Taylor series approximation
        public static f32 Exp(f32 value)
        {
            f32 x = value;
            f32 term = f32.one;
            f32 result = term;
            term *= x / f32.FromRaw(1 << 16); result += term;
            term *= x / f32.FromRaw(2 << 16); result += term;
            term *= x / f32.FromRaw(6 << 16); result += term;
            term *= x / f32.FromRaw(24 << 16); result += term;
            term *= x / f32.FromRaw(120 << 16); result += term;
            term *= x / f32.FromRaw(720 << 16); result += term;
            return result;
        }

        // Natural logarithm using approximation (not accurate)
        public static f32 Log(f32 value)
        {
            if (value <= f32.zero) throw new ArgumentOutOfRangeException("Logarithm of non-positive value is undefined.");
            f32 result = f32.zero;
            f32 x = (value - f32.one) / (value + f32.one);
            f32 term = x;
            f32 x2 = x * x;
            for (int i = 1; i < 10; i += 2)
            {
                result += term / f32.FromRaw(i << 16);
                term *= x2;
            }
            return result * f32.FromRaw(2 << 16);
        }
    }
}
