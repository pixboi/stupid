using stupid.Maths;

namespace stupid
{
    public class TransformS
    {
        // Transform properties
        public Vector3S position, deltaPosition, transientPosition;
        public QuaternionS rotation;
        public Vector3S localScale;
        public Matrix3S rotationMatrix, rotationMatrixTranspose;

        // Constructor
        public TransformS(in Vector3S position, in QuaternionS rotation, in Vector3S localScale)
        {
            this.position = position;
            this.rotation = rotation;
            this.deltaPosition = Vector3S.zero;
            this.transientPosition = position;
            this.localScale = localScale;
            UpdateRotationMatrix();
        }

        public void UpdateRotationMatrix()
        {
            this.rotationMatrix = Matrix3S.Rotate(this.rotation);
            this.rotationMatrixTranspose = rotationMatrix.Transpose();
        }

        public void AddDelta(in Vector3S amount)
        {
            this.deltaPosition += amount;
            this.transientPosition = position + deltaPosition;
        }

        public void ActuateDelta()
        {
            this.position += this.deltaPosition;
            this.transientPosition = this.position;

            this.deltaPosition = Vector3S.zero;
        }

        // Updates rotation matrix
        public void Rotate(in QuaternionS delta)
        {
            this.rotation = (delta * rotation).Normalize();
            UpdateRotationMatrix();
        }

        // Converts world point to local point
        public Vector3S ToLocalPoint(in Vector3S worldPoint) => this.rotationMatrixTranspose * (worldPoint - position);
        public Vector3S ToWorldPoint(in Vector3S localPoint) => (rotationMatrix * localPoint) + position;
        public Vector3S InverseTransformDirection(in Vector3S worldDirection) => this.rotationMatrixTranspose * worldDirection;
        public Vector3S TransformDirection(in Vector3S localDirection) => rotationMatrix * localDirection;

    }
}
