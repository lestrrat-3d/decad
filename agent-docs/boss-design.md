# Boss design

`Boss` raises a round boss on a planar or cylindrical face, with an optional blind pilot hole. It builds a cylinder tool, unions it
to the body and cuts the pilot with the tools `Hole` builds, all in one atomic commit. It adds no geometry kernel.

## 1. Contract

```go
type BossBore struct {
    Diameter, Depth units.Value
    Head            HoleHead
    Thread          *HoleThread
    Draft           units.Value
}

type BossSpec struct {
    Face     FaceSelector
    At       r3.Vec
    Diameter units.Value
    Height   units.Value
    Bore       *BossBore
    Draft      units.Value
    RootFillet units.Value
    Shape      HoleShape
}

func Boss(ctx context.Context, b *Body, spec BossSpec) (*Body, error)
```

| Decision | Rule |
|---|---|
| B1 | `Face` resolves to exactly one analytic plane of a live solid `b`, as `Hole`'s does (`agent-docs/hole-design.md` H1), or one cylinder (§8). `At` is a world point within `holeplan.OnPlaneTol` (1e-6 mm) of the face plane, and is never moved onto it. |
| B2 | The boss runs `Height` along the face's outward normal, a positive length. `Diameter` is positive. Zero is `ErrDegenerate` and a wrong unit kind is `ErrUnitKind`. |
| B3 | The cylinder tool starts `embed` behind the face, where `embed` is `holeplan.MouthMargin` (1 mm) or half the body's depth behind the face, whichever is less, so the union crosses the face and nothing lies in its plane. A body with no depth behind the face is `ErrDegenerate`. |
| B4 | The pilot hole is `Hole`'s plain blind hole, started on the boss's top face at the boss axis: the same tools, `holeplan.Compute` and gates. `Bore.Diameter` is less than `Diameter`, and `Bore.Depth` is positive and less than `Height` plus the body's depth behind the face, so the pilot never leaves through the back. A violation is `ErrDegenerate`. |
| B5 | One `combineToolSteps` chain applies the union and then each pilot tool as a cut. The history holds one `OperationBoss` entry with `b` consumed, and the tools stay private. A failure in any step leaves `b` live and the document unchanged. |

## 2. Exactness

The union of a round boss with a planar face and the cut of the pilot are the prism Booleans `Hole` and `Union`
already run, so the result carries their exactness: a round boss is `Approximate` because pi is never exact, and its
faces are planes and cylinders where the Booleans admit them.

## 3. Not covered

- Ribs other than the triangular gussets of §9, and gussets on a shaped, drafted, filleted or cylinder boss. Add others with `Rib` and `Web`.
- A pilot thread whose major diameter exceeds the boss's.
- A draft that closes the boss or breaks its circle before its top: `Extrude`'s refusal.
- A boss that stands on a curved face other than a cylinder's (§8), a boss on a cylinder with a root fillet or a draft, and a shaped boss with a root fillet.
- A boss whose axis is tilted from the face normal.

## 4. A pilot with a head or a thread

`BossBore.Head` and `BossBore.Thread` are `HoleSpec`'s `Head` and `Thread` (`agent-docs/hole-design.md` §8) for the pilot.

| Decision | Rule |
|---|---|
| W1 | `Head` is a `Counterbore` or a `Countersink` at the pilot's mouth on the boss's top face. The gates are `Hole`'s: the head is wider than the pilot, a counterbore is shallower than the pilot, and a countersink's cone is shallower than the pilot. |
| W2 | `Thread` taps the pilot as `Hole` does: `Diameter` is the tap drill, and `Length` is measured from where the pilot proper starts, which is the top face or a head's floor. The zero `Length` threads the pilot's whole depth and stops at its bottom, since a pilot is blind. |
| W3 | The pilot's tools (the bore, a head and the coil) are united with one another and cut from the boss in one step. A second cut of the boss would meet a target that already holds the first cut's coarse mesh, which the evaluator refuses when the cut's own chord tolerance is finer. |
| W4 | One `OperationBoss` entry holds the whole. A thread whose major diameter would pass the boss's wall leaves a wall thinner than its groove: choose a boss wider than the thread's major diameter. |

## 5. A drafted boss

`BossSpec.Draft` leans the boss's wall. It passes `WithTaper` to the `Extrude` that builds the pillar.

| Decision | Rule |
|---|---|
| Y1 | `Diameter` is the boss's diameter at `Face`. A positive `Draft` makes the boss narrower toward its top, `Diameter − 2·Height·tan(Draft)` across there, and a negative one wider. The zero `Value` is a straight wall. |
| Y2 | The pillar's footing starts `embed` behind the face (B3), where the taper begins, so its circle there is wider by `embed·tan(Draft)`: the wall is then the stated diameter at the face. |
| Y3 | `Draft` is read as a signed angle. A wrong unit kind is `ErrUnitKind`. A draft whose magnitude is at or past a right angle is `ErrDegenerate`, and one that closes or breaks the circle before the top is `Extrude`'s own refusal, `ErrUnsupported`. |
| Y4 | The pilot opens on the boss's top face, so its diameter MUST be less than the boss's diameter there as well as at the face: a pilot that would break through the narrowed wall is `ErrDegenerate`. |
| Y5 | `BossBore.Draft` leans the pilot's wall as `HoleSpec.Draft` does (`agent-docs/hole-design.md` §9): narrower toward its bottom for a positive angle. A tapped pilot takes none, `ErrDegenerate`. |

## 6. A root fillet

`RootFillet` rounds the concave circle where the boss meets its face, with the evaluator's `Fillet` of that edge.

| Decision | Rule |
|---|---|
| R1 | The zero `Value` leaves the corner sharp. A positive length is the fillet's radius. A negative one is `ErrNegativeMagnitude` and a wrong unit kind `ErrUnitKind`. |
| R2 | The boss is built as before. The fillet then takes the one circular edge of the result that lies in the face's plane, about the boss's axis, with the boss's radius at the face (a drafted boss's radius at the face is its `Diameter`). The result is a mesh-free torus patch where `Fillet` admits one, and carries the fillet's own bounds. |
| R3 | A boss and fillet are tried on a copy of `b` first: the copy is built, filleted and retired. A fillet the evaluator refuses, or a root circle not found in the result, therefore leaves `b` live and the document unchanged. The copy and its trial leave history entries. |
| R4 | The result is `Boss`'s union and pilot cut followed by `Fillet`, so the history holds `OperationBoss` then `OperationFillet`. A fillet too large for the face, for the boss's height or for the body's other edges is `Fillet`'s refusal. |

## 7. A shaped boss

`BossSpec.Shape` replaces the round section with a `RectangularHole`, `SlotHole` or `PolygonHole`, the shapes of `Hole`
(`agent-docs/hole-design.md` §5, §6, §10), so a rectangular rib block, a stadium lug or a hexagonal post stands on the face.

| Decision | Rule |
|---|---|
| Q1 | The section is centred on `At` and turned as the hole shape is: a rectangle's `Width` along the face frame's U axis before `Angle` turns it, a slot's long axis along U, and a polygon with a flat toward U. The shape's size is the boss's size at the face. |
| Q2 | `Diameter` must be the zero value (`ErrDegenerate` otherwise) and the shape's own gates are the hole's: a polygon of 3 to 64 sides, a slot longer than it is wide, positive lengths (`ErrUnitKind`, `ErrDegenerate`). |
| Q3 | The pillar is the shape extruded out of the face, starting `embed` behind it, from a sketch of fixed points joined by lines and arcs, as the round pillar is from a circle. |
| Q4 | A `Draft` moves every wall of the section by `h·tan(draft)` over a height `h`, which is the rectangle's width and height each smaller by `2h·tan(draft)`, the polygon's width across flats and the slot's width and length the same. The footing is wider by `embed·tan(draft)` on each side so the wall has the stated size at the face. A taller draft than the section can close is `Extrude`'s refusal. |
| Q5 | A pilot opens in the top face and must be narrower than the shape's smallest width there: the rectangle's shorter side, the slot's width or the polygon's width across flats, less `2·height·tan(draft)`. Its other gates are §1's. |
| Q6 | A `RootFillet` with a shape is `ErrUnsupported`: the root loop of §6 is a circle. |
| Q7 | The volume above the face is the section's area times the height, which the tests compute independently: `w·l·h − (w+l)·t·h² + 4/3·t²·h³` for a drafted rectangle, `(n·(a/2)²·tan(π/n))·h` for a polygon across flats `a`, and `(w·(l−w) + π·w²/4)·h` for a slot. |

## 8. A boss on a cylindrical face

`Face` may resolve to one analytic cylindrical face, a rod's wall or a bore's, as `Hole`'s may
(`agent-docs/hole-design.md`). The boss stands on the plane tangent to the cylinder at `At` and runs along the radial line
through it, out of the body's material.

| Decision | Rule |
|---|---|
| C1 | `At` is a world point within `holeplan.OnPlaneTol` of the cylinder, off its axis, and is the boss's centre: the tangent plane, its frame and the boss's local axis come from `Hole`'s `cylinderHoleSite`. The boss runs `Height` along the face's outward normal at `At`, which points away from the axis for a rod and toward it for a bore, measured from the tangent plane. |
| C2 | A convex wall falls away from the tangent plane by the sagitta `R − √(R² − r²)` at distance `r`, so the footing starts `embed = MouthMargin + R − √(R² − r²)` behind it, `r` being half the boss's `Diameter`, and lies inside the body across its whole section. A boss as wide as the cylinder (`r ≥ R`), or `embed` past the body's depth behind the face, is `ErrDegenerate`. A concave wall rises into the material the footing already overlaps, so `embed` stays `MouthMargin`. |
| C3 | A boss on a cylindrical face takes no `RootFillet` and no `Draft`: both are `ErrUnsupported`. A pilot `Bore`, its head, its thread and the shape of §7 follow the tangent frame as on a plane. Everything else of §1 holds, and the result is the union of the pillar with the body, a mesh Boolean where the evaluator has no analytic route, with its own proven volume bound. |
| C4 | The pillar's footprint along the cylinder's axis is the caller's to keep on the wall: a boss wider than the rod is long overhangs its ends and adds the overhang. |
| C5 | The tests state the volume of a rod with a boss and of a bore with a boss from the integral of the pillar over its footprint, `∫ 2√(r² − y²)·(R + h − √(R² − y²)) dy` for a rod and `∫ 2√(r² − y²)·(√(R² − y²) − (R − h)) dy` for a bore, and require it inside the result's own volume bound; the sagitta embed is checked by a boss wide enough that dropping it leaves a gap the volume shows. |

## 9. Gusset ribs at the foot

`BossSpec.Ribs` braces the boss's root with `BossRibs.Count` triangular gusset ribs spaced evenly round its axis.

| Decision | Rule |
|---|---|
| G1 | `Count` is from 1 to 16 and `Thickness`, `Length` and `Height` are positive lengths (`ErrDegenerate` for zero or a count out of range, `ErrNegativeMagnitude`, `ErrUnitKind`). `Height` is at most the boss's `Height`, and `Thickness` is less than the boss's diameter, each otherwise `ErrDegenerate`. With several ribs, half the thickness is less than `r·sin(π/Count)`, `r` the boss's radius, so neighbouring ribs do not meet at the wall: otherwise `ErrDegenerate`. |
| G2 | Rib `k` lies in the plane through the boss's axis at the angle `Angle + 2πk/Count` from the face frame's U axis, `Angle` read as an angle (the zero Value is zero) and `ErrUnitKind` for any other kind. It is `Thickness` thick, half each way from that plane. |
| G3 | Its section in that plane, with a the distance from the axis and z the height above the face, is the polygon `(r/2, −e/2)`, `(r + Length, −e/2)`, `(r + Length, 0)`, `(r, Height)`, `(r/2, Height)`, `e` the footing depth `embed` of B3: it is flat across the top inside the boss, slopes down to the face at its outer end, and is buried in the boss and the plate, but lies above the pillar's footing plane so no two facets of the tools overlap in one plane (the evaluator refuses that). |
| G4 | The pillar and the ribs are united into one tool and that tool into the body, and the pilot and its head are cut after, so a rib never closes a pilot. A boss with ribs takes no `Draft` (the wall leans from the rib's top vertex), no `RootFillet`, no `Shape`, and no cylindrical face, each `ErrUnsupported`. |
| G5 | The volume the ribs add outside the boss and above the face is, for each rib, `Height · (Thickness·Length/2 + ∫ (r − √(r² − y²)) dy)` over `y` across its thickness: the flat top before the wall's radius and the slope after it. The tests state it independently, require it inside the result's volume bound, and check a point in a rib's foot and a point beside it. |
