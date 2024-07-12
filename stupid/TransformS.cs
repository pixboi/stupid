using stupid.Maths;

namespace stupid
{
    public struct TransformS
    {
        //Matrices?

        // Transform
        public Vector3S position;
        public QuaternionS rotation;
        public Vector3S localScale;

        public Matrix3S rotationMatrix { get; private set; }

        public TransformS(Vector3S position = default, QuaternionS rotation = default, Vector3S localScale = default)
        {
            this.position = position;

            this.rotation = rotation;
            if (this.rotation == QuaternionS.zero) this.rotation = QuaternionS.identity;

            this.localScale = localScale;
            if (this.localScale == Vector3S.zero) this.localScale = Vector3S.one;

            this.rotationMatrix = Matrix3S.Rotate(this.rotation);
        }

        public void UpdateRotationMatrix()
        {
            this.rotationMatrix = Matrix3S.Rotate(this.rotation);
        }
    }
}
