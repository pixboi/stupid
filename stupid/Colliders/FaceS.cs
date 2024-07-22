using stupid.Maths;

namespace stupid.Colliders
{
    public readonly struct FaceS
    {
        public readonly Vector3S vertex1;
        public readonly Vector3S vertex2;
        public readonly Vector3S vertex3;
        public readonly Vector3S vertex4;
        public readonly Vector3S normal;

        public FaceS(Vector3S vertex1, Vector3S vertex2, Vector3S vertex3, Vector3S vertex4, Vector3S normal)
        {
            this.vertex1 = vertex1;
            this.vertex2 = vertex2;
            this.vertex3 = vertex3;
            this.vertex4 = vertex4;
            this.normal = normal;
        }
    }
}
