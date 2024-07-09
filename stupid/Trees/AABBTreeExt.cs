using stupid.Colliders;
using stupid.Maths;
using System.Collections.Generic;

namespace stupid.Trees
{
    public static class AABBTreeExt
    {
        /*
        public static List<RaycastHit> RaycastAll(this AABBTree tree, Ray ray, ref List<RaycastHit> hits)
        {
            RaycastAll(tree.Root, ray, ref hits);
            return hits;
        }

        private static void RaycastAll(AABBNode node, Ray ray, ref List<RaycastHit> hits)
        {
            if (node == null) return;

            f32 distance;
            Vector3S point, normal;

            if (!ray.Intersects(node.Box, out distance, out point, out normal)) return;

            if (node.IsLeaf)
            {
                hits.Add(new RaycastHit(node.Data, distance, point, normal));
            }
            else
            {
                RaycastAll(node.Left, ray, ref hits);
                RaycastAll(node.Right, ray, ref hits);
            }
        }

        public static RaycastHit? Raycast(this AABBTree tree, Ray ray)
        {
            return Raycast(tree.Root, ray);
        }

        private static RaycastHit? Raycast(AABBNode node, Ray ray)
        {
            if (node == null) return null;

            f32 distance;
            Vector3S point, normal;

            if (!ray.Intersects(node.Box, out distance, out point, out normal)) return null;

            if (node.IsLeaf)
            {
                return new RaycastHit(node.Data, distance, point, normal);
            }
            else
            {
                var leftHit = Raycast(node.Left, ray);
                if (leftHit.HasValue)
                {
                    return leftHit;
                }

                var rightHit = Raycast(node.Right, ray);
                if (rightHit.HasValue)
                {
                    return rightHit;
                }
            }

            return null;
        }
        */

        public static List<BoundsS> CollectBounds(this AABBTree tree)
        {
            List<BoundsS> boundsList = new List<BoundsS>();
            CollectBounds(tree.Root, boundsList);
            return boundsList;
        }

        private static void CollectBounds(AABBNode node, List<BoundsS> boundsList)
        {
            if (node == null) return;

            boundsList.Add(node.Bounds);

            if (!node.IsLeaf)
            {
                CollectBounds(node.Left, boundsList);
                CollectBounds(node.Right, boundsList);
            }
        }
    }
}
