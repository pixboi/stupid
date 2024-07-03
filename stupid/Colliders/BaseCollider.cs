using SoftFloat;
using stupid.Maths;

namespace stupid.Colliders
{
    public abstract class BaseCollider : ICollider
    {
        protected SRigidbody attachedRigidbody;
        public virtual SRigidbody GetRigidbody() => attachedRigidbody;

        public void Attach(SRigidbody rigidbody)
        {
            attachedRigidbody = rigidbody;
            CalculateBounds(rigidbody.position);
        }

        public abstract SBounds CalculateBounds(Vector3S position);
        public abstract SBounds GetBounds();
        public abstract bool Intersects(Vector3S positionA, Vector3S positionB, ICollider other, out Contact contact);

    }
}
