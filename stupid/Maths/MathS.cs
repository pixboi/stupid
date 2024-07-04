using SoftFloat;
using System.Security.Cryptography;
using System.Text;
using System;

namespace stupid.Maths
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

        public static string CalculateStateHash(World world)
        {
            using (var sha256 = SHA256.Create())
            {
                var sb = new StringBuilder();
                foreach (var rb in world.Rigidbodies)
                {
                    sb.Append(rb.position.x.ToString());
                    sb.Append(rb.position.y.ToString());
                    sb.Append(rb.position.z.ToString());
                    sb.Append(rb.velocity.x.ToString());
                    sb.Append(rb.velocity.y.ToString());
                    sb.Append(rb.velocity.z.ToString());
                    sb.Append(rb.angularVelocity.x.ToString());
                    sb.Append(rb.angularVelocity.y.ToString());
                    sb.Append(rb.angularVelocity.z.ToString());
                    sb.Append(rb.rotation.x.ToString());
                    sb.Append(rb.rotation.y.ToString());
                    sb.Append(rb.rotation.z.ToString());
                    sb.Append(rb.rotation.w.ToString());
                }

                var hashBytes = sha256.ComputeHash(Encoding.UTF8.GetBytes(sb.ToString()));
                return BitConverter.ToString(hashBytes).Replace("-", "").ToLower();
            }
        }

    }
}
