using stupid.Maths;

namespace stupid.Colliders
{
    public interface IShape
    {
        void Attach(Collidable body);
        Collidable collidable { get; }
        BoundsS CalculateAABB(in TransformS transform);
        BoundsS bounds { get; }
        int Intersects(Collidable other, ref ContactS[] contacts); // Add this method
        Matrix3S CalculateInertiaTensor(f32 mass);
        bool NeedsRotationUpdate { get; }
        void OnRotationUpdate();
    }
}
