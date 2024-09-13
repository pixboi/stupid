using stupid.Maths;

namespace stupid
{
    public struct TransformS
    {
        // Transform properties
        public Vector3S position, deltaPosition;
        public Vector3S transientPosition => position + deltaPosition;
        public QuaternionS rotation;
        public Vector3S localScale;
        public Matrix3S rotationMatrix;

        // Constructor
        public TransformS(Vector3S position, QuaternionS rotation, Vector3S localScale)
        {
            this.position = position;
            this.rotation = rotation;
            this.deltaPosition = Vector3S.zero;
            this.localScale = localScale;
            this.rotationMatrix = Matrix3S.Rotate(this.rotation);
        }

        // Updates rotation matrix
        public void UpdateRotationMatrix()
        {
            this.rotationMatrix = Matrix3S.Rotate(this.rotation);
        }

        // Converts world point to local point
        public Vector3S ToLocalPoint(in Vector3S worldPoint) => this.rotationMatrix.Transpose() * (worldPoint - position);
        public Vector3S ToWorldPoint(in Vector3S localPoint) => (rotationMatrix * localPoint) + position;
        public Vector3S InverseTransformDirection(in Vector3S worldDirection) => this.rotationMatrix.Transpose() * worldDirection;
        public Vector3S TransformDirection(in Vector3S localDirection) => rotationMatrix * localDirection;

    }
}
