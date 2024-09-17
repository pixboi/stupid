using stupid.Maths;

namespace stupid
{
    public struct TransformS
    {
        // Transform properties
        public Vector3S position, deltaPosition, transientPosition;
        public QuaternionS rotation;
        public Vector3S localScale;
        public Matrix3S rotationMatrix;

        // Constructor
        public TransformS(Vector3S position, QuaternionS rotation, Vector3S localScale)
        {
            this.position = position;
            this.rotation = rotation;
            this.deltaPosition = Vector3S.zero;
            this.transientPosition = position;
            this.localScale = localScale;
            this.rotationMatrix = Matrix3S.Rotate(this.rotation);
        }

        public void AddDelta(Vector3S amount)
        {
            this.deltaPosition += amount;
            this.transientPosition = position + deltaPosition;
        }

        public void ActuateDelta()
        {
            this.position += this.deltaPosition;
            this.deltaPosition.Reset();
            this.transientPosition = this.position;
        }

        // Updates rotation matrix
        public void UpdateRotationMatrix()
        {
            this.rotationMatrix = Matrix3S.Rotate(this.rotation);
        }

        public void Rotate(QuaternionS delta)
        {
            this.rotation = (delta * rotation).Normalize();
            UpdateRotationMatrix();
        }

        // Converts world point to local point
        public Vector3S ToLocalPoint(in Vector3S worldPoint) => this.rotationMatrix.Transpose() * (worldPoint - position);
        public Vector3S ToWorldPoint(in Vector3S localPoint) => (rotationMatrix * localPoint) + position;
        public Vector3S InverseTransformDirection(in Vector3S worldDirection) => this.rotationMatrix.Transpose() * worldDirection;
        public Vector3S TransformDirection(in Vector3S localDirection) => rotationMatrix * localDirection;

    }
}
