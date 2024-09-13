using System.Runtime.CompilerServices;

namespace stupid.Maths
{
    public struct Tensor
    {
        // Only the inverse of the local inertia tensor is stored
        public readonly Matrix3S inertiaInverse;
        public Matrix3S inertiaWorld;

        public Tensor(in Matrix3S inertia, in TransformS t)
        {
            // Precompute the inverse of the local inertia tensor
            this.inertiaInverse = inertia.Inverse();
            this.inertiaWorld = Matrix3S.identity; // Initial value, updated in the next step
            CalculateInverseInertiaTensor(t);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void CalculateInverseInertiaTensor(in TransformS t)
        {
            // Use the transform's rotation matrix to calculate the world space inertia tensor
            // Perform R * I (rotation matrix multiplied by local inertia tensor)
            var ri = t.rotationMatrix * inertiaInverse;
            // Perform the final multiplication with the transpose of R
            inertiaWorld = ri * t.rotationMatrix.Transpose();
        }

    }
}
