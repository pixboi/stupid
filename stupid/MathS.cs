using SoftFloat;

namespace stupid
{
    public static class MathS
    {
        public static sfloat Min(sfloat a, sfloat b) => a < b ? a : b;
        public static sfloat Max(sfloat a, sfloat b) => a > b ? a : b;

        public static sfloat Clamp(sfloat value, sfloat min, sfloat max)
        {
            return Max(min, Min(max, value));
        }

        public static sfloat Floor(sfloat value)
        {
            return libm.floorf(value);
        }

        public static sfloat Abs(sfloat value)
        {
            return value < sfloat.zero ? -value : value;

        }
    }
}
