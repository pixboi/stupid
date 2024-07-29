using stupid.Maths;

namespace stupid.Colliders
{
    public readonly struct TriangleS
    {
        public readonly Vector3S a, b, c;
        public readonly Vector3S normal;

        public TriangleS(Vector3S a, Vector3S b, Vector3S c, Vector3S normal)
        {
            this.a = a;
            this.b = b;
            this.c = c;
            this.normal = normal;
        }
    }
}
