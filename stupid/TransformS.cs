using stupid.Maths;

namespace stupid
{
    public struct TransformS
    {
        // Transform properties
        public Vector3S position;

        public QuaternionS rotation;

        public Vector3S localScale;

        public Matrix3S rotationMatrix;

        // Constructor
        public TransformS(Vector3S position = default, QuaternionS rotation = default, Vector3S localScale = default)
        {
            this.position = position;
            this.rotation = rotation == QuaternionS.zero ? QuaternionS.identity : rotation;
            this.localScale = localScale == Vector3S.zero ? Vector3S.one : localScale;
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
            return rotationMatrix.Transpose() * (worldPoint - position); // Assuming rotationMatrix is orthogonal
        }

        //After this, the worldpoint should in be local
        public void ToLocalPointInPlace(in Vector3S worldPoint, out Vector3S localPoint)
        {
            localPoint = worldPoint;
            localPoint.SubtractInPlace(position);
            localPoint.MultiplyInPlace(rotationMatrix.Transpose());
        }

        // Converts local point to world point
        public Vector3S ToWorldPoint(in Vector3S localPoint)
        {
            return (rotationMatrix * localPoint) + position;
        }

        // Converts world direction to local direction
        public Vector3S InverseTransformDirection(in Vector3S worldDirection)
        {
            return rotationMatrix.Transpose() * worldDirection; // Assuming rotationMatrix is orthogonal
        }

        // Converts local direction to world direction
        public Vector3S TransformDirection(in Vector3S localDirection)
        {
            return rotationMatrix * localDirection;
        }

        // Converts world vector to local vector
        public Vector3S InverseTransformVector(in Vector3S worldVector)
        {
            return rotationMatrix.Transpose() * worldVector; // Assuming rotationMatrix is orthogonal
        }

        // Converts local vector to world vector
        public Vector3S TransformVector(in Vector3S localVector)
        {
            return rotationMatrix * localVector;
        }

        // Translates the transform by a given vector
        public void Translate(in Vector3S translation)
        {
            position += translation;
        }

        // Rotates the transform by a given quaternion
        public void Rotate(in QuaternionS deltaRotation)
        {
            rotation = (deltaRotation * rotation).Normalize();
            UpdateRotationMatrix();
        }

        // Sets the position of the transform
        public void SetPosition(in Vector3S newPosition)
        {
            position = newPosition;
        }

        // Sets the rotation of the transform
        public void SetRotation(in QuaternionS newRotation)
        {
            rotation = newRotation;
            UpdateRotationMatrix();
        }

        // Sets the local scale of the transform
        public void SetLocalScale(in Vector3S newScale)
        {
            localScale = newScale;
        }

        // Looks at a target point
        public void LookAt(in Vector3S target)
        {
            Vector3S direction = (target - position).Normalize();
            rotation = QuaternionS.LookRotation(direction);
            UpdateRotationMatrix();
        }
    }
}
