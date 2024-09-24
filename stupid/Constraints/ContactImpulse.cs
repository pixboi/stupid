using stupid.Constraints;
using stupid.Maths;
using stupid;
using System.Runtime.CompilerServices;

public struct ContactImpulse
{
    public readonly Vector3S point;//24
    public f32 normalMass;//8
    public f32 accumulatedImpulse; //8
    public readonly int featureId; //4

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public ContactImpulse(in ContactData data)
    {
        point = data.point;
        featureId = data.featureId;
        accumulatedImpulse = f32.zero;
        normalMass = f32.zero;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void CalculatePrestep(RigidbodyS a, RigidbodyS b, in Vector3S ra, in Vector3S rb, in Vector3S normal)
    {
        Vector3S raCrossNormal = Vector3S.Cross(ra, normal);
        f32 angularMassA = Vector3S.Dot(raCrossNormal, a.tensor.inertiaWorld * raCrossNormal);
        f32 effectiveMass = a.inverseMass + angularMassA;

        if (b != null)
        {
            Vector3S rbCrossNormal = Vector3S.Cross(rb, normal);
            f32 angularMassB = Vector3S.Dot(rbCrossNormal, b.tensor.inertiaWorld * rbCrossNormal);
            effectiveMass += b.inverseMass + angularMassB;
        }

        normalMass = effectiveMass > f32.zero ? f32.one / effectiveMass : f32.zero;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void SolveImpulse(RigidbodyS a, RigidbodyS b, in Vector3S ra, in Vector3S rb, in f32 inverseDt, in f32 penetrationDepth, in Vector3S normal, in WorldSettings settings, bool useBias)
    {
        f32 bias = f32.zero;
        if (penetrationDepth > f32.zero)
        {
            bias = penetrationDepth * inverseDt;
        }
        else if (useBias)
        {
            var separation = MathS.Min(f32.zero, penetrationDepth + settings.DefaultContactOffset);
            bias = MathS.Max(settings.Baumgartner * separation * inverseDt, -settings.DefaultMaxDepenetrationVelocity);
        }

        var contactVelocity = ContactSlim.CalculateContactVelocity(a, b, ra, rb);
        var vn = Vector3S.Dot(contactVelocity, normal);

        var incremental = -normalMass * (vn + bias);
        var newImpulse = MathS.Max(incremental + accumulatedImpulse, f32.zero);
        incremental = newImpulse - accumulatedImpulse;
        accumulatedImpulse = newImpulse;

        var impulse = normal * incremental;
        ContactSlim.ApplyImpulse(a, b, impulse, ra, rb);
    }
}