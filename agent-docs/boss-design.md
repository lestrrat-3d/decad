# Boss design

`Boss` raises a round boss on a planar face, with an optional blind pilot hole. It builds a cylinder tool, unions it
to the body and cuts the pilot with the tools `Hole` builds, all in one atomic commit. It adds no geometry kernel.

## 1. Contract

```go
type BossBore struct {
    Diameter, Depth units.Value
    Head            HoleHead
    Thread          *HoleThread
}

type BossSpec struct {
    Face     FaceSelector
    At       r3.Vec
    Diameter units.Value
    Height   units.Value
    Bore     *BossBore
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

- Ribs, gussets and a root fillet at the boss's foot. Add them with `Rib`, `Web` and `Fillet`.
- A pilot with a draft, and a pilot thread whose major diameter exceeds the boss's.
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

