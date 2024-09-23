using stupid.Maths;

namespace stupid
{
    public struct RigidbodySData
    {
        public readonly Vector3S position;
        public readonly Matrix3S inertiaWorld;
        public readonly f32 inverseMass;
        public readonly bool isDynamic;

        public Vector3S velocity;
        public Vector3S angularVelocity;

        public RigidbodySData(RigidbodyS body)
        {
            if (body != null)
            {
                this.position = body.transform.position;
                this.velocity = body.velocity;
                this.angularVelocity = body.angularVelocity;
                this.inertiaWorld = body.tensor.inertiaWorld;
                this.inverseMass = body.inverseMass;
                this.isDynamic = body.isDynamic;
            }
            else
            {
                this.position = Vector3S.zero;
                this.velocity = Vector3S.zero;
                this.angularVelocity = Vector3S.zero;
                this.inertiaWorld = Matrix3S.identity;
                this.inverseMass = f32.zero;
                this.isDynamic = false;
            }
        }

        public void Actuate(RigidbodyS body)
        {
            body.velocity = this.velocity;
            body.angularVelocity = this.angularVelocity;
        }
    }
}
