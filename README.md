# stupid (and lets keep it so)

* uses a 48/16 fixed point type for deterministic maths
* pretty good broadphase (sort and sweep on xyz)
* supports warm starting and relaxing (for stable resting contacts, somewhat)
* OOBB and sphere shapes
* api design and naming so that its easy to put into unity UNITY_THING+"S"
* easily build world through script or by converting from unity types (might add the current integration later)

# what would be nice
* better box v box, now its kinda messy
* redesign the constraints and collections so that they fit the 64 bit cache line, now most structs are way over 64 (matrices are huge)
* more tests
* more shapes

# thanks
https://box2d.org/posts/2024/02/solver2d/
https://leanrada.com/notes/sweep-and-prune/
