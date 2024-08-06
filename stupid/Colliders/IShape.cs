using stupid.Maths;

namespace stupid.Colliders
{
    public interface IShape
    {
        void Attach(Collidable body);
        Collidable collidable { get; }
        BoundsS CalculateAABB(in Vector3S position, in QuaternionS rotation);
        BoundsS bounds { get; }
        int Intersects(Collidable other, ref ContactS contact); // Add this method
        Matrix3S CalculateInertiaTensor(f32 mass);
        bool NeedsRotationUpdate { get; }
        void OnRotationUpdate();
    }
}
