using stupid.Maths;
using System.Runtime.CompilerServices;

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
        // public Vector3S ToLocalPoint(in Vector3S worldPoint) => this.rotationMatrixTranspose * (worldPoint - position);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Vector3S ToLocalPoint(in Vector3S worldPoint)
        {
            long dx = worldPoint.x.rawValue - position.x.rawValue;
            long dy = worldPoint.y.rawValue - position.y.rawValue;
            long dz = worldPoint.z.rawValue - position.z.rawValue;

            long xRaw = (rotationMatrixTranspose.m00.rawValue * dx + rotationMatrixTranspose.m01.rawValue * dy + rotationMatrixTranspose.m02.rawValue * dz) >> f32.FractionalBits;
            long yRaw = (rotationMatrixTranspose.m10.rawValue * dx + rotationMatrixTranspose.m11.rawValue * dy + rotationMatrixTranspose.m12.rawValue * dz) >> f32.FractionalBits;
            long zRaw = (rotationMatrixTranspose.m20.rawValue * dx + rotationMatrixTranspose.m21.rawValue * dy + rotationMatrixTranspose.m22.rawValue * dz) >> f32.FractionalBits;

            return new Vector3S(new f32(xRaw), new f32(yRaw), new f32(zRaw));
        }

        // public Vector3S ToWorldPoint(in Vector3S localPoint) => (rotationMatrix * localPoint) + position;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Vector3S ToWorldPoint(in Vector3S localPoint)
        {
            long xRaw = ((rotationMatrix.m00.rawValue * localPoint.x.rawValue + rotationMatrix.m01.rawValue * localPoint.y.rawValue + rotationMatrix.m02.rawValue * localPoint.z.rawValue) >> f32.FractionalBits) + position.x.rawValue;
            long yRaw = ((rotationMatrix.m10.rawValue * localPoint.x.rawValue + rotationMatrix.m11.rawValue * localPoint.y.rawValue + rotationMatrix.m12.rawValue * localPoint.z.rawValue) >> f32.FractionalBits) + position.y.rawValue;
            long zRaw = ((rotationMatrix.m20.rawValue * localPoint.x.rawValue + rotationMatrix.m21.rawValue * localPoint.y.rawValue + rotationMatrix.m22.rawValue * localPoint.z.rawValue) >> f32.FractionalBits) + position.z.rawValue;

            return new Vector3S(new f32(xRaw), new f32(yRaw), new f32(zRaw));
        }

        //public Vector3S InverseTransformDirection(in Vector3S worldDirection) => this.rotationMatrixTranspose * worldDirection;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Vector3S InverseTransformDirection(in Vector3S worldDirection)
        {
            long xRaw = (rotationMatrixTranspose.m00.rawValue * worldDirection.x.rawValue + rotationMatrixTranspose.m01.rawValue * worldDirection.y.rawValue + rotationMatrixTranspose.m02.rawValue * worldDirection.z.rawValue) >> f32.FractionalBits;
            long yRaw = (rotationMatrixTranspose.m10.rawValue * worldDirection.x.rawValue + rotationMatrixTranspose.m11.rawValue * worldDirection.y.rawValue + rotationMatrixTranspose.m12.rawValue * worldDirection.z.rawValue) >> f32.FractionalBits;
            long zRaw = (rotationMatrixTranspose.m20.rawValue * worldDirection.x.rawValue + rotationMatrixTranspose.m21.rawValue * worldDirection.y.rawValue + rotationMatrixTranspose.m22.rawValue * worldDirection.z.rawValue) >> f32.FractionalBits;

            return new Vector3S(new f32(xRaw), new f32(yRaw), new f32(zRaw));
        }

        //public Vector3S TransformDirection(in Vector3S localDirection) => rotationMatrix * localDirection;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Vector3S TransformDirection(in Vector3S localDirection)
        {
            long xRaw = (rotationMatrix.m00.rawValue * localDirection.x.rawValue + rotationMatrix.m01.rawValue * localDirection.y.rawValue + rotationMatrix.m02.rawValue * localDirection.z.rawValue) >> f32.FractionalBits;
            long yRaw = (rotationMatrix.m10.rawValue * localDirection.x.rawValue + rotationMatrix.m11.rawValue * localDirection.y.rawValue + rotationMatrix.m12.rawValue * localDirection.z.rawValue) >> f32.FractionalBits;
            long zRaw = (rotationMatrix.m20.rawValue * localDirection.x.rawValue + rotationMatrix.m21.rawValue * localDirection.y.rawValue + rotationMatrix.m22.rawValue * localDirection.z.rawValue) >> f32.FractionalBits;

            return new Vector3S(new f32(xRaw), new f32(yRaw), new f32(zRaw));
        }


    }
}
