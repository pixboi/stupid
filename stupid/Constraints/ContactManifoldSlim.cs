using stupid.Maths;
using System.Runtime.CompilerServices;
using stupid;
using stupid.Constraints;
using stupid.Broadphase;

public struct ContactManifoldSlim
{
    public readonly RigidbodyS a;
    public readonly Collidable b;
    public readonly Vector3S normal, avgPoint;
    public readonly f32 penetrationDepth;
    public readonly int contactCount;
    public ContactSlim c0, c1, c2, c3;

    public Vector3S tangent;
    public f32 tangentMass, accumulatedFriction;

    // Twist-related properties
    public f32 twistMass, accumulatedTwist;

    public ContactManifoldSlim(RigidbodyS a, Collidable b, in ContactData[] data, int contactCount)
    {
        this.a = a;
        this.b = b;
        this.normal = data[0].normal;
        this.penetrationDepth = data[0].penetrationDepth;
        this.contactCount = contactCount;

        if (contactCount < 1)
        {
            throw new System.ArgumentException("ZERO CONTACTS?");
        }

        var bb = b.isDynamic ? (RigidbodyS)b : null;

        c0 = new ContactSlim(a, bb, this.normal, data[0]);
        c1 = new ContactSlim(a, bb, this.normal, data[1]);
        c2 = new ContactSlim(a, bb, this.normal, data[2]);
        c3 = new ContactSlim(a, bb, this.normal, data[3]);

        this.accumulatedFriction = f32.zero;

        Vector3S avg = Vector3S.zero;
        if (contactCount >= 1) avg += data[0].point;
        if (contactCount >= 2) avg += data[1].point;
        if (contactCount >= 3) avg += data[2].point;
        if (contactCount >= 4) avg += data[3].point;

        this.avgPoint = avg / (f32)this.contactCount;

        this.tangent = Vector3S.zero;
        this.tangentMass = f32.zero;
        this.twistMass = f32.zero;
        this.accumulatedTwist = f32.zero;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void CalculateMassTangent(in Vector3S prevTangent)
    {
        var bb = b.isDynamic ? (RigidbodyS)b : null;

        // Calculate relative velocity at the contact point
        var ra = avgPoint - a.transform.position;
        var rb = bb != null ? avgPoint - b.transform.position : Vector3S.zero;

        var aVel = a.velocity + Vector3S.Cross(a.angularVelocity, ra);
        var bVel = bb != null ? bb.velocity + Vector3S.Cross(bb.angularVelocity, rb) : Vector3S.zero;

        var contactVelocity = bVel - aVel;

        // Project the contact velocity onto the plane perpendicular to the normal (get the tangential velocity)
        Vector3S normalVelocity = this.normal * Vector3S.Dot(contactVelocity, this.normal);
        Vector3S tangentialVelocity = contactVelocity - normalVelocity;

        // Precompute tangential velocity magnitude and small threshold comparison
        var tangentialVelocitySqMag = tangentialVelocity.sqrMagnitude;
        bool usePrevTangent = tangentialVelocitySqMag < f32.small && prevTangent != Vector3S.zero;

        if (usePrevTangent)
        {
            // Blend factor between previous tangent and current tangential velocity
            f32 blendFactor = tangentialVelocitySqMag / (tangentialVelocitySqMag + f32.small);

            // Use a linear interpolation between the previous tangent and the normalized tangential velocity
            this.tangent = Vector3S.Lerp(prevTangent, tangentialVelocity.Normalize(), blendFactor);
        }
        else
        {
            this.tangent = tangentialVelocity.Normalize();
        }

        // Precompute cross products for mass calculation
        var raCrossTangent = Vector3S.Cross(ra, this.tangent);
        var tm = a.inverseMass + Vector3S.Dot(Vector3S.Cross(a.tensor.inertiaWorld * raCrossTangent, ra), this.tangent);

        if (bb != null)
        {
            var rbCrossTangent = Vector3S.Cross(rb, this.tangent);
            tm += bb.inverseMass + Vector3S.Dot(Vector3S.Cross(bb.tensor.inertiaWorld * rbCrossTangent, rb), this.tangent);
        }

        this.tangentMass = tm > f32.zero ? f32.one / tm : f32.zero;

        // --- Twist Mass Calculation ---
        var raCrossN = Vector3S.Cross(ra, this.normal);
        var twistMass = a.inverseMass + Vector3S.Dot(Vector3S.Cross(a.tensor.inertiaWorld * raCrossN, ra), this.normal);

        if (bb != null)
        {
            var rbCrossN = Vector3S.Cross(rb, this.normal);
            twistMass += bb.inverseMass + Vector3S.Dot(Vector3S.Cross(bb.tensor.inertiaWorld * rbCrossN, rb), this.normal);
        }

        this.twistMass = twistMass > f32.zero ? f32.one / twistMass : f32.zero;
    }

    public IntPair ToPair => new IntPair(this.a.index, this.b.index);

    // Indexer with a direct array-like access pattern using unsafe pointers
    public ref ContactSlim this[int i]
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get
        {
            if (i < 0 || i >= 4) throw new System.ArgumentOutOfRangeException();
            unsafe
            {
                fixed (ContactSlim* ptr = &c0)
                {
                    return ref *(ptr + i);
                }
            }
        }
    }

    // Retain impulse data from the old manifold
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void RetainData(in ContactManifoldSlim old)
    {
        //if (old.contactCount == this.contactCount)
        this.accumulatedFriction = old.accumulatedFriction;
        this.accumulatedTwist = old.accumulatedTwist;

        for (int i = 0; i < contactCount; i++)
        {
            ref var c = ref this[i];

            for (int j = 0; j < old.contactCount; j++)
            {
                var o = old[j];
                if (Transfer(ref c, o))
                {
                    break; // Exit the inner loop once a match is found
                }
            }
        }
    }

    // Transfer impulse data from an old contact to a new one
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private bool Transfer(ref ContactSlim c, in ContactSlim old)
    {
        if (c.featureId == old.featureId)
        {
            c.accumulatedImpulse = old.accumulatedImpulse;
            return true;
        }

        return false;
    }

    // Warmup for iterative solvers
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void Warmup()
    {
        var bb = b.isDynamic ? (RigidbodyS)b : null;

        var tangentImpulse = this.tangent * this.accumulatedFriction;
        var ra = this.avgPoint - a.transform.position;
        var rb = this.avgPoint - b.transform.position;
        ApplyImpulse(a, bb, tangentImpulse, ra, rb);

        var twistImpulse = this.normal * this.accumulatedTwist;
        ApplyTwistImpulse(a, bb, twistImpulse);

        for (int i = 0; i < contactCount; i++)
        {
            this[i].WarmStart(this.normal, a, bb);
        }
    }

    public void Resolve(f32 inverseDt, WorldSettings settings, bool bias)
    {
        f32 sumAccum = f32.zero;
        var bb = b.isDynamic ? (RigidbodyS)b : null;

        if (contactCount >= 1)
        {
            c0.SolveImpulse(a, bb, inverseDt, settings, this.penetrationDepth, this.normal, bias);
            sumAccum += c0.accumulatedImpulse;
        }

        if (contactCount >= 2)
        {
            c1.SolveImpulse(a, bb, inverseDt, settings, this.penetrationDepth, this.normal, bias);
            sumAccum += c1.accumulatedImpulse;
        }

        if (contactCount >= 3)
        {
            c2.SolveImpulse(a, bb, inverseDt, settings, this.penetrationDepth, this.normal, bias);
            sumAccum += c2.accumulatedImpulse;
        }

        if (contactCount >= 4)
        {
            c3.SolveImpulse(a, bb, inverseDt, settings, this.penetrationDepth, this.normal, bias);
            sumAccum += c3.accumulatedImpulse;
        }

        var friction = MathS.Sqrt(a.material.staticFriction * b.material.staticFriction);

        SolveFriction(sumAccum, friction);
    }

    public void SolveFriction(in f32 sumAccum, in f32 friction)
    {
        var bb = b.isDynamic ? (RigidbodyS)b : null;

        var ra = this.avgPoint - a.transform.position;
        var rb = bb != null ? this.avgPoint - b.transform.position : Vector3S.zero;

        // Linear friction (already present)
        var av = a.velocity + Vector3S.Cross(a.angularVelocity, ra);
        var bv = bb != null ? bb.velocity + Vector3S.Cross(bb.angularVelocity, rb) : Vector3S.zero;
        var contactVelocity = bv - av;

        // Linear friction impulse calculation
        var vt = Vector3S.Dot(contactVelocity, this.tangent);
        var incrementalFriction = -this.tangentMass * vt;

        var coulombMax = sumAccum * friction;
        var newImpulse = MathS.Clamp(this.accumulatedFriction + incrementalFriction, -coulombMax, coulombMax);
        incrementalFriction = newImpulse - this.accumulatedFriction;
        this.accumulatedFriction = newImpulse;

        var tangentImpulse = this.tangent * incrementalFriction;
        ApplyImpulse(a, bb, tangentImpulse, ra, rb);


        // --- Twist Friction ---
        // Calculate the relative angular velocity along the contact normal
        var relativeAngularVelocity = a.angularVelocity - (bb != null ? bb.angularVelocity : Vector3S.zero);
        var twistAngularVelocity = Vector3S.Dot(relativeAngularVelocity, this.normal);  // Twist around the normal

        // Calculate the twist friction impulse
        var incrementalTwist = -this.twistMass * twistAngularVelocity * (f32)this.contactCount;

        var twistFrictionMax = this.accumulatedFriction;
        var newTwistImpulse = MathS.Clamp(this.accumulatedTwist + incrementalTwist, -twistFrictionMax, twistFrictionMax);
        incrementalTwist = newTwistImpulse - this.accumulatedTwist;
        this.accumulatedTwist = newTwistImpulse;

        var twistImpulse = this.normal * incrementalTwist;  // Twist impulse is along the normal
        ApplyTwistImpulse(a, bb, twistImpulse);

    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    static void ApplyImpulse(in RigidbodyS a, in RigidbodyS b, in Vector3S impulse, in Vector3S ra, in Vector3S rb)
    {
        a.velocity -= impulse * a.inverseMass; // A moves away
        a.angularVelocity -= a.tensor.inertiaWorld * Vector3S.Cross(ra, impulse);

        if (b != null)
        {
            b.velocity += impulse * b.inverseMass; // B moves along normal
            b.angularVelocity += b.tensor.inertiaWorld * Vector3S.Cross(rb, impulse);
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    static void ApplyTwistImpulse(in RigidbodyS a, in RigidbodyS b, in Vector3S twistImpulse)
    {
        // Apply twist friction to angular velocity (rotational effect)
        a.angularVelocity -= a.tensor.inertiaWorld * twistImpulse;

        if (b != null)
        {
            b.angularVelocity += b.tensor.inertiaWorld * twistImpulse;
        }
    }


}
