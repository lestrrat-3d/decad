# Boss design

`Boss` raises a round boss on a planar face, with an optional blind pilot hole. It builds a cylinder tool, unions it
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
}

func Boss(ctx context.Context, b *Body, spec BossSpec) (*Body, error)
```

| Decision | Rule |
|---|---|
| B1 | `Face` resolves to exactly one analytic plane of a live solid `b`, as `Hole`'s does (`agent-docs/hole-design.md` H1). `At` is a world point within `holeplan.OnPlaneTol` (1e-6 mm) of the face plane, and is never moved onto it. |
| B2 | The boss runs `Height` along the face's outward normal, a positive length. `Diameter` is positive. Zero is `ErrDegenerate` and a wrong unit kind is `ErrUnitKind`. |
| B3 | The cylinder tool starts `embed` behind the face, where `embed` is `holeplan.MouthMargin` (1 mm) or half the body's depth behind the face, whichever is less, so the union crosses the face and nothing lies in its plane. A body with no depth behind the face is `ErrDegenerate`. |
| B4 | The pilot hole is `Hole`'s plain blind hole, started on the boss's top face at the boss axis: the same tools, `holeplan.Compute` and gates. `Bore.Diameter` is less than `Diameter`, and `Bore.Depth` is positive and less than `Height` plus the body's depth behind the face, so the pilot never leaves through the back. A violation is `ErrDegenerate`. |
| B5 | One `combineToolSteps` chain applies the union and then each pilot tool as a cut. The history holds one `OperationBoss` entry with `b` consumed, and the tools stay private. A failure in any step leaves `b` live and the document unchanged. |

## 2. Exactness

The union of a round boss with a planar face and the cut of the pilot are the prism Booleans `Hole` and `Union`
already run, so the result carries their exactness: a round boss is `Approximate` because pi is never exact, and its
faces are planes and cylinders where the Booleans admit them.

## 3. Not covered

- Ribs and gussets at the boss's foot. Add them with `Rib` and `Web`.
- A pilot thread whose major diameter exceeds the boss's.
- A draft that closes the boss or breaks its circle before its top: `Extrude`'s refusal.
- A boss that is not round, or one that stands on a curved face.
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
