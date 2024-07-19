using stupid.Maths;

namespace stupid
{
    public struct TransformS
    {
        // Transform properties
        public Vector3S position;
        public QuaternionS rotation;
        public Vector3S localScale;
        public Matrix3S rotationMatrix { get; private set; }

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
        public Vector3S ToLocalPoint(Vector3S worldPoint)
        {
            Vector3S translatedPoint = worldPoint - position;
            return rotationMatrix.Transpose() * translatedPoint; // Assuming rotationMatrix is orthogonal
        }

        // Converts local point to world point
        public Vector3S ToWorldPoint(Vector3S localPoint)
        {
            return (rotationMatrix * localPoint) + position;
        }

        // Translates the transform by a given vector
        public void Translate(Vector3S translation)
        {
            position += translation;
        }

        // Rotates the transform by a given quaternion
        public void Rotate(QuaternionS deltaRotation)
        {
            rotation = deltaRotation * rotation;
            UpdateRotationMatrix();
        }

        // Sets the position of the transform
        public void SetPosition(Vector3S newPosition)
        {
            position = newPosition;
        }

        // Sets the rotation of the transform
        public void SetRotation(QuaternionS newRotation)
        {
            rotation = newRotation;
            UpdateRotationMatrix();
        }

        // Sets the local scale of the transform
        public void SetLocalScale(Vector3S newScale)
        {
            localScale = newScale;
        }

        // Looks at a target point
        public void LookAt(Vector3S target)
        {
            Vector3S direction = (target - position).Normalize();
            rotation = QuaternionS.LookRotation(direction);
            UpdateRotationMatrix();
        }
    }
}
