using stupid.Colliders;
using stupid.Maths;

namespace stupid.Colliders
{
    public interface ICollider
    {
        SRigidbody GetRigidbody();
        SBounds CalculateBounds(Vector3S position);
        SBounds GetBounds();        //cached version
        bool Intersects(Vector3S positionA, Vector3S positionB, ICollider other, out Contact contact); // Add this method
    }
}
