using stupid.Maths;
using stupid.Colliders;
using SoftFloat;

namespace stupid
{
    public class SRigidbody
    {
        // Runtime
        public readonly int index;
        public override int GetHashCode() => index;

        public Vector3S position;
        public SQuaternion rotation;

        public Vector3S velocity;
        public Vector3S angularVelocity;

        public ICollider collider;

        // Settings
        public sfloat mass = sfloat.one;
        public Matrix3S InertiaTensor { get; private set; }
        public Matrix3S InverseInertiaTensor { get; private set; }

        public bool useGravity = true;
        public bool isKinematic = false;

        public SRigidbody(int index)
        {
            this.index = index;
            this.rotation = SQuaternion.Identity;
            InitializeInertiaTensor();
        }

        public SRigidbody(int index, Vector3S position = default, Vector3S velocity = default, Vector3S angularVelocity = default)
        {
            this.index = index;
            this.position = position;
            this.velocity = velocity;
            this.angularVelocity = angularVelocity;
            this.rotation = SQuaternion.Identity;
            InitializeInertiaTensor();
        }

        private void InitializeInertiaTensor()
        {
            // This should be adjusted according to the actual shape and mass distribution of the rigidbody
            sfloat oneFifth = (sfloat)0.2f;
            InertiaTensor = new Matrix3S(new sfloat[,] {
                { oneFifth * mass, sfloat.zero, sfloat.zero },
                { sfloat.zero, oneFifth * mass, sfloat.zero },
                { sfloat.zero, sfloat.zero, oneFifth * mass }
            });
            InverseInertiaTensor = Matrix3S.Inverse(InertiaTensor);
        }

        public void Attach(ICollider collider)
        {
            this.collider = collider;
            collider.Attach(this);
        }

        public void UpdateRotation(sfloat deltaTime)
        {
            if (angularVelocity.Magnitude() > sfloat.zero)
            {
                sfloat angle = angularVelocity.Magnitude() * deltaTime;
                Vector3S axis = angularVelocity.Normalize();
                sfloat halfAngle = angle / (sfloat)2.0f;
                sfloat sinHalfAngle = libm.sinf(halfAngle);

                SQuaternion deltaRotation = new SQuaternion(
                    libm.cosf(halfAngle),
                    axis.x * sinHalfAngle,
                    axis.y * sinHalfAngle,
                    axis.z * sinHalfAngle
                );

                rotation = deltaRotation * rotation;
                angularVelocity *= (sfloat)0.99f;
            }
        }
    }
}
