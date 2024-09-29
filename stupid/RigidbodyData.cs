using stupid.Maths;
using System.Runtime.CompilerServices;

namespace stupid
{
    public struct RigidbodyData
    {
        public readonly Matrix3S inertiaWorld;
        public readonly Vector3S position;
        public Vector3S velocity, angularVelocity;
        public readonly f32 inverseMass;
        public readonly int index;
        public readonly bool isDynamic;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public RigidbodyData(Collidable c)
        {
            this.index = c.index;
            this.position = c.transform.position;
            this.velocity = c.velocity;
            this.angularVelocity = c.angularVelocity;
            this.inverseMass = c.inverseMass;
            this.inertiaWorld = c.tensor.inertiaWorld;
            this.isDynamic = c.isDynamic;
        }
    }
}
