﻿using stupid.Maths;
using System.Runtime.CompilerServices;

namespace stupid.Constraints
{
    //The only thing different really, between contacts that share the same manifold, are the offset FROM A, and feature ID
    //We could grealt reduce the memory footprint with this... i think
    public readonly struct ContactData
    {
        //Init
        public readonly Vector3S point, normal;//48
        public readonly f32 penetrationDepth; //8
        public readonly int featureId; //4

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ContactData(Vector3S point, Vector3S normal, f32 penetrationDepth, int featureId = -1)
        {
            this.point = point;
            this.normal = normal;
            this.penetrationDepth = penetrationDepth;
            this.featureId = featureId;
        }
    }
}