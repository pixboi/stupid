using stupid.Maths;

namespace stupid.Colliders
{
    public readonly struct EdgeS
    {
        public readonly Vector3S start;
        public readonly Vector3S end;

        public EdgeS(Vector3S start, Vector3S end)
        {
            this.start = start;
            this.end = end;
        }
    }
}
