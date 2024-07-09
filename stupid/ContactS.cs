
using stupid.Maths;

namespace stupid
{
    public struct ContactManifoldS
    {
        public Collidable a;
        public Collidable b;
        public ContactS contact;
    }

    public struct ContactS
    {
        public Vector3S point;
        public Vector3S normal;
        public f32 penetrationDepth;
    }
}
