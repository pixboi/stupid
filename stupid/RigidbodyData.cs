using stupid.Maths;
using System.Runtime.CompilerServices;

namespace stupid
{
    public struct RigidbodyData
    {
        //Quat = 32, vector 24 = 56
        public readonly Matrix3S inertiaWorld; // 8*9
        public readonly Vector3S position; // 24
        public Vector3S velocity, angularVelocity; //48
        public readonly f32 inverseMass; // 8
        public readonly int index; //4
        public readonly bool isDynamic; //1

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
