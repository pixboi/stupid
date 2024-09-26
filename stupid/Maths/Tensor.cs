using System.Runtime.CompilerServices;

namespace stupid.Maths
{
    //The life time of this object => were never creating a new tensor, just updating it
    public struct Tensor
    {
        // Only the inverse of the local inertia tensor is stored
        public readonly Matrix3S inertiaLocal;
        public Matrix3S inertiaWorld;

        public Tensor(in Matrix3S inertia, in TransformS t)
        {
            // Precompute the inverse of the local inertia tensor
            this.inertiaLocal = Matrix3S.Inverse(inertia);
            this.inertiaWorld = default;
            UpdateInertiaTensor(t);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void UpdateInertiaTensor(in TransformS t)
        {
            // Use the transform's rotation matrix to calculate the world space inertia tensor
            // Perform R * I (rotation matrix multiplied by local inertia tensor)
            var ri = t.rotationMatrix * inertiaLocal;
            // Perform the final multiplication with the transpose of R

            //Make sure the ROTATION MATRIX IS UPDATED!!!
            inertiaWorld = ri * t.rotationMatrix.Transpose(); //t.rotationMatrix.Transpose();
        }

    }
}
