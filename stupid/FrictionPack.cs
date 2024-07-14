using stupid.Maths;

namespace stupid
{

    public readonly struct FrictionPack
    {
        public readonly RigidbodyS a, b;
        public readonly ContactS contact;
        public readonly f32 impulseScalar, velocityAlongNormal;
        public readonly Vector3S relativeVelocityAtContact, ra, rb;

        public FrictionPack(RigidbodyS a, RigidbodyS b, ContactS contact, f32 impulseScalar, f32 velocityAlongNormal, Vector3S relativeVelocityAtContact, Vector3S ra, Vector3S rb)
        {
            this.a = a;
            this.b = b;
            this.contact = contact;
            this.impulseScalar = impulseScalar;
            this.velocityAlongNormal = velocityAlongNormal;
            this.relativeVelocityAtContact = relativeVelocityAtContact;
            this.ra = ra;
            this.rb = rb;
        }
    }

}
