# stupid

* uses a 48/16 fixed point type for deterministic maths
* pretty good broadphase (sort and sweep on xyz)
* supports warm starting and relaxing (for stable resting contacts, somewhat)
* OOBB and sphere shapes
* api design and naming so that its easy to put into unity UNITY_THING+"S"
* easily build world through script or by converting from unity types (might add the current integration later)

# todo'ish
* better box v box, now its kinda messy
* redesign the constraints and collections so that they fit the 64 bit cache line, now most structs are way over 64
* more tests
