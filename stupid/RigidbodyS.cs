using stupid.Colliders;
using stupid.Maths;
using System;

namespace stupid
{

    public class RigidbodyS : Collidable
    {
        // Integration
        public Vector3S velocity;
        public Vector3S angularVelocity;

        // Settings
        public f32 mass = f32.one;
        public bool useGravity = true;
        public bool isKinematic = false;

        public Tensor tensor;

        public RigidbodyS(int index, IShape collider, bool isDynamic = true, TransformS transform = default,
            Vector3S velocity = default, Vector3S angularVelocity = default, f32 mass = default, bool useGravity = true, bool isKinematic = false) : base(index, collider, isDynamic, transform)
        {

            this.mass = mass;
            if (this.mass == f32.zero) this.mass = f32.one;

            if (collider != null)
            {
                this.tensor = new Tensor();
                tensor.inertia = collider.CalculateInertiaTensor(this.mass);
                tensor.inertiaInverseLocal = tensor.inertia.Inverse();
                tensor.inertiaWorld = tensor.inertiaInverseLocal;
            }

            this.velocity = velocity;
            this.angularVelocity = angularVelocity;

            this.useGravity = useGravity;
            this.isKinematic = isKinematic;
        }

    }
}
