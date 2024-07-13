
using stupid.Maths;

namespace stupid
{
    public readonly struct ContactManifoldS
    {
        public readonly Collidable a;
        public readonly Collidable b;
        public readonly ContactS contact;

        public ContactManifoldS(Collidable a, Collidable b, ContactS contact)
        {
            this.a = a;
            this.b = b;
            this.contact = contact;
        }
    }

    public readonly struct ContactS
    {
        public readonly Vector3S point;
        public readonly Vector3S normal;
        public readonly f32 penetrationDepth;

        public ContactS(Vector3S point, Vector3S normal, f32 penetrationDepth)
        {
            this.point = point;
            this.normal = normal;
            this.penetrationDepth = penetrationDepth;
        }
    }
}
