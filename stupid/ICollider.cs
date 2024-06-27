namespace stupid
{
    public interface ICollider
    {
        Bounds GetBounds(Vector3S position);
        bool Intersects(Vector3S positionA, Vector3S positionB, ICollider other, out Contact contact); // Add this method
    }
}
