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
            this.deltaPosition = Vector3S.zero;
            this.rotation = rotation;
            this.localScale = localScale;
            this.rotationMatrix = Matrix3S.Rotate(this.rotation);
        }

        // Updates rotation matrix
        public void UpdateRotationMatrix()
        {
            this.rotationMatrix = Matrix3S.Rotate(this.rotation);
        }

        // Converts world point to local point
        public Vector3S ToLocalPoint(in Vector3S worldPoint)
        {
            return this.rotationMatrix.Transpose() * (worldPoint - position); // Assuming rotationMatrix is orthogonal
        }

        // Converts local point to world point
        public Vector3S ToWorldPoint(in Vector3S localPoint)
        {
            return (rotationMatrix * localPoint) + position;
        }

        // Converts world direction to local direction
        public Vector3S InverseTransformDirection(in Vector3S worldDirection)
        {
            return this.rotationMatrix.Transpose() * worldDirection; // Assuming rotationMatrix is orthogonal
        }

        // Converts local direction to world direction
        public Vector3S TransformDirection(in Vector3S localDirection)
        {
            return rotationMatrix * localDirection;
        }

    }
}
