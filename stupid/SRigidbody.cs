using SoftFloat;
using stupid.Colliders;
using stupid.Maths;

namespace stupid
{
    public class SRigidbody
    {
        // Runtime
        public readonly int index;
        public override int GetHashCode() => index;

        // Transform
        public Vector3S position;
        public SQuaternion rotation;

        public Vector3S velocity;
        public Vector3S angularVelocity;

        public ICollider collider;

        // Settings
        public sfloat mass = sfloat.one;
        public Vector3S centerOfMass = Vector3S.zero;
        public Vector3S inertiaTensor = new Vector3S(1, 1, 1);
        public SQuaternion inertiaTensorRotation = SQuaternion.Identity;

        public bool useGravity = true;
        public bool isKinematic = false;

        public SRigidbody(int index)
        {
            this.index = index;
            this.rotation = SQuaternion.Identity;
        }

        public SRigidbody(int index, Vector3S position = default, Vector3S velocity = default, Vector3S angularVelocity = default)
        {
            this.index = index;
            this.position = position;
            this.velocity = velocity;
            this.angularVelocity = angularVelocity;
            this.rotation = SQuaternion.Identity;
            this.inertiaTensor = new Vector3S(1, 1, 1);
        }

        public void Attach(ICollider collider)
        {
            this.collider = collider;
            collider.Attach(this);
        }

        public Matrix3S GetInverseInertiaTensorWorld()
        {
            // Compute the local space inverse inertia tensor
            Matrix3S inverseInertiaTensorLocal = new Matrix3S(
                new Vector3S(inertiaTensor.x != sfloat.zero ? sfloat.one / inertiaTensor.x : sfloat.zero, sfloat.zero, sfloat.zero),
                new Vector3S(sfloat.zero, inertiaTensor.y != sfloat.zero ? sfloat.one / inertiaTensor.y : sfloat.zero, sfloat.zero),
                new Vector3S(sfloat.zero, sfloat.zero, inertiaTensor.z != sfloat.zero ? sfloat.one / inertiaTensor.z : sfloat.zero)
            );

            // Convert the inertia tensor to world space using the body's rotation
            Matrix3S rotationMatrix = Matrix3S.Rotate(rotation);
            Matrix3S inverseInertiaTensorWorld = rotationMatrix * inverseInertiaTensorLocal * rotationMatrix.Transpose();

            return inverseInertiaTensorWorld;
        }
    }
}
