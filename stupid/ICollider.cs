namespace stupid
{
    public interface ICollider
    {
        Rigidbody GetRigidbody();
        Bounds CalculateBounds(Vector3S position);

        //cached version
        Bounds GetBounds();
        bool Intersects(Vector3S positionA, Vector3S positionB, ICollider other, out Contact contact); // Add this method
    }
}
