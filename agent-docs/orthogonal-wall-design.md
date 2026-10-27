# Orthogonal wall and undercut survey design

`Verify` with `WithMinWallThickness` reads the thinnest wall of a faceted solid every face of which squares to a world
axis exactly, from the grid of its coordinates (`internal/orthoshell/wall.go`, called by
`internal/verifyrun/faceted_wall.go`). The ray survey of `agent-docs/mesh-wall-survey-design.md` can prove a wall too
thin and never thick enough; this reading proves both. The undercut survey of such a solid against a pull along a world axis is read from the same grid (§3).

## 1. Contract

| Decision | Rule |
|---|---|
| E1 | The reading applies to a faceted solid whose held vertices all carry a zero bound and every triangle of which squares to an axis exactly (a mesh solid, a result of the orthogonal Boolean, `MoveFace`, `Stretch`, `Offset` or an orthogonal `Shell`). Any other faceted body takes the rays. |
| E2 | The thinnest wall is the shortest run of material along any of the three axes: over every line of the solid's grid parallel to an axis, each maximal stretch of occupied cells is a run, and its length is the sum of the cell gaps. For faces that all square to an axis that is the least distance between two opposite faces whose footprints overlap, which is what a wall is. A fin, a neck or a floor is a run, wherever the rays' centre samples fall. |
| E3 | The reading is that length with a bound of `1e-12` of the larger of 1 mm and the solid's extent, for the rounding of the sums. Its interval has a lower end as well as an upper one, so an interval wholly above the tool is `Sound` (no wall finding), one wholly below is `DiagWallTooThin`, `Violating`, and one that reaches the tool is `DiagUndecidedWall`, `Suspect`. A solid with no material is not read this way. |
| E4 | The tests read an L whose legs are 4 mm thick against 2, 4 and 5 mm tools (`Sound` with no finding, undecided, proven too thin with the exact 4 mm), a cup with 1 mm walls against 2 and 0.5 mm tools and a thick block with a 0.5 mm fin. A survey that falls back to the rays leaves the thick-enough case undecided and fails them. |

## 3. Undercuts against an axis pull

| Decision | Rule |
|---|---|
| E5 | For a pull exactly along a world axis, `±X`, `±Y` or `±Z`, on a body that meets E1, a face square to the pull is released along its own outward normal and is blocked when material lies beyond it, in that direction, on any line over any grid cell it covers with positive area (`orthoshell.Undercuts`). A face square to the pull's axis is tested; one square to another axis is edge on and is never blocked, as U1 of `agent-docs/mesh-wall-survey-design.md`. |
| E6 | The test is made on every line over every covered cell, so the survey decides each face: the blocked faces are listed and are `DiagUndercut`, `Violating`, and a body with none has a proven all-clear, an empty listing with no finding and no undecided mark, so a part free to pull can be `Sound`. A pull that is not exactly an axis, or a body outside E1, takes the rays and their refusal to prove a pull free. |
| E7 | The tests pull a block along four axis directions to `Sound`, pull a T-shaped part with a cap along z (the cap's underside and the base's top are the two blocked faces) and free along x and y, and find a closed cavity blocked. A survey that falls back to the rays leaves the free pulls undecided and fails them. |

## 2. Not covered

- A faceted body with a slanted or curved face, or with a vertex bound, which keeps the ray survey and its refusal to prove a wall thick enough.
- A pull that is not along a world axis, and the concave-radius survey of an axis-square solid.
