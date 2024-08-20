using stupid.Maths;

namespace stupid.Colliders
{
    public readonly struct ContactS
    {
        public readonly Collidable a, b;
        public readonly Vector3S point, normal, ra, rb;
        public readonly f32 penetrationDepth, effectiveMass;

        public ContactS(Vector3S point, Vector3S normal, f32 penetrationDepth, Collidable a, Collidable b)
        {
            this.a = a;
            this.b = b;
            this.point = point;
            this.normal = normal;
            this.penetrationDepth = penetrationDepth;
            this.ra = this.point - a.transform.position;
            this.rb = this.point - b.transform.position;

            var AB = (RigidbodyS)this.a;
            var BB = b.isDynamic ? (RigidbodyS)this.b : null;

            f32 invMassA = AB.inverseMass;
            f32 invMassB = BB != null ? BB.inverseMass : f32.zero;

            // Linear effective mass
            f32 effectiveMass = invMassA + invMassB;

            // Angular effective mass for body A
            Vector3S raCrossNormal = Vector3S.Cross(this.ra, this.normal);
            f32 angularMassA = Vector3S.Dot(raCrossNormal, AB.tensor.inertiaWorld * raCrossNormal);
            effectiveMass += angularMassA;

            // Angular effective mass for body B
            if (BB != null)
            {
                Vector3S rbCrossNormal = Vector3S.Cross(this.rb, this.normal);
                f32 angularMassB = Vector3S.Dot(rbCrossNormal, BB.tensor.inertiaWorld * rbCrossNormal);
                effectiveMass += angularMassB;
            }

            // Invert to get effective mass
            this.effectiveMass = effectiveMass > f32.zero ? f32.one / effectiveMass : f32.zero;
        }
    }
}
