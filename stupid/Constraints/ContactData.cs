﻿using stupid.Maths;

namespace stupid.Constraints
{
    //The only thing different really, between contacts that share the same manifold, are the offset FROM A, and feature ID
    //We could grealt reduce the memory footprint with this... i think

    public readonly struct ContactData
    {
        //Init
        public readonly Vector3S point, normal;
        public readonly f32 penetrationDepth;
        public readonly byte featureId;

        public ContactData(Vector3S point, Vector3S normal, f32 penetrationDepth, byte featureId)
        {
            this.point = point;
            this.normal = normal;
            this.penetrationDepth = penetrationDepth;
            this.featureId = featureId;
        }
    }
}