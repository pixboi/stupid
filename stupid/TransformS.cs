using stupid.Maths;

namespace stupid
{
    public struct TransformS
    {
        // Transform properties
        public Vector3S position, deltaPosition, transientPosition;

        public QuaternionS rotation;

        public Vector3S localScale;

        public Matrix3S rotationMatrix, rotationMatrixTransposed;

        // Constructor
        public TransformS(Vector3S position, QuaternionS rotation, Vector3S localScale)
        {
            this.position = position;
            this.deltaPosition = Vector3S.zero;
            this.transientPosition = Vector3S.zero;
            this.rotation = rotation;
            this.localScale = localScale;
            this.rotationMatrix = Matrix3S.Rotate(this.rotation);
            this.rotationMatrixTransposed = rotationMatrix.Transpose();
        }

        public void MoveDelta(Vector3S amount)
        {
            this.deltaPosition.AddInPlace(amount);
            this.transientPosition = position + deltaPosition;
        }

        public void ActuateDelta()
        {
            this.position.AddInPlace(this.deltaPosition);
            this.deltaPosition.Reset();
            //this.transientPosition = position + deltaPosition;
        }

        // Updates rotation matrix
        public void UpdateRotationMatrix()
        {
            this.rotationMatrix = Matrix3S.Rotate(this.rotation);
            this.rotationMatrixTransposed = this.rotationMatrix.Transpose();
        }

        // Converts world point to local point
        public Vector3S ToLocalPoint(in Vector3S worldPoint)
        {
            return rotationMatrixTransposed * (worldPoint - position); // Assuming rotationMatrix is orthogonal
        }

        public void ToLocalPointFast(ref Vector3S worldPoint)
        {
            worldPoint.SubtractInPlace(position);
            Matrix3S.MultiplyInPlace(rotationMatrixTransposed, ref worldPoint);
        }

        // Converts local point to world point
        public Vector3S ToWorldPoint(in Vector3S localPoint)
        {
            return (rotationMatrix * localPoint) + position;
        }



        // Converts world direction to local direction
        public Vector3S InverseTransformDirection(in Vector3S worldDirection)
        {
            return rotationMatrixTransposed * worldDirection; // Assuming rotationMatrix is orthogonal
        }

        // Converts local direction to world direction
        public Vector3S TransformDirection(in Vector3S localDirection)
        {
            return rotationMatrix * localDirection;
        }

    }
}
