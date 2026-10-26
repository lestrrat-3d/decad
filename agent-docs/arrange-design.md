# Arrange design

`Arrange` lays bodies out side by side on a flat bed, for printing or machining several parts at once. It moves
bodies by translation with `Transform` and adds no geometry kernel. The placement itself is plain-data packing in
`internal/arrangeplan/`.

## 1. Contract

```go
type ArrangeBed struct {
    Origin       r3.Vec
    Width, Depth units.Value
    Gap          units.Value
    Height       units.Value
}

func Arrange(ctx context.Context, bodies []*Body, bed ArrangeBed, opts ...ArrangeOption) ([]*Body, error)
func WithArrangeTurns() ArrangeOption
func WithArrangeAngle(step units.Value) ArrangeOption
```

| Decision | Rule |
|---|---|
| G1 | `Origin` is the world point of the bed's minimum corner. `Width` runs along X and `Depth` along Y from it. `Origin.Z` is the bed's surface. |
| G2 | `Width` and `Depth` are positive lengths and `Gap` is a non-negative length. A zero size is `ErrDegenerate`, a negative value is `ErrNegativeMagnitude`, and a wrong unit kind is `ErrUnitKind`. |
| G3 | `bodies` is non-empty, holds no nil, no repeat and no retired body, and every body belongs to one document. A violation is `ErrDegenerate`, `ErrRetiredBody` or `ErrForeignBody`. |
| G4 | Each body's footprint is its `Bounds` box grown by the box's own `Bound` on every side, so two bodies whose proven boxes are `Gap` apart are never closer. The footprint is the extent along X and Y. |
| G5 | Each body moves by one translation. Its minimum Z lands on `Origin.Z`, and its footprint lands at the position `arrangeplan.Pack` gives. No body turns unless `WithArrangeTurns` is given (§5). |
| G6 | The result holds the moved bodies in the order given, and the inputs are retired through `Transform`, so each move is an `OperationTransform` history entry. |
| G7 | Bodies that do not all fit are `ErrDegenerate` and no body moves. The plan is complete before the first move. |

## 2. Packing

`arrangeplan.Pack` places footprints deepest first, in input order among equals. An item goes to the right of the
previous one on the current shelf. One that would cross the bed's right edge starts a shelf above the last, and one
that would cross the top edge ends the search with no placement. A shelf is as deep as its deepest item, and a gap
separates items from each other and from the bed's edges.

This is a first-fit shelf packer on bounding rectangles. It does not search for a tighter layout, so a set that a
cleverer packer would fit can still be refused.

## 3. Failure after the first move

A canceled context, or a `Transform` failure, after some bodies have moved leaves those bodies at their new places
and the rest where they were. Every refusal G2, G3 and G7 name is raised before the first move.

## 4. Not covered

- Turning bodies about any axis but +Z, or following their outlines. Packing is by bounding rectangle.
- Packing in three dimensions beyond whole layers, and stacking one body on another that it fits above.
- A bed that is not a rectangle aligned with X and Y.

## 5. Quarter turns

`WithArrangeTurns` lets `Arrange` turn bodies a quarter turn about +Z. `arrangeplan.PackTurning` chooses which.

| Decision | Rule |
|---|---|
| T1 | `PackTurning` first lays every body deeper than it is wide with its long side along X, so shelves stay shallow, and packs that. If that does not fit it packs the bodies as given, so allowing turns never refuses a set that `Pack` fits. |
| T2 | A body that turns is rotated a quarter turn counter-clockwise about +Z through its bounding box's minimum corner. Its bounds are then read again and one translation places it, as G5 says. |
| T3 | A turned body costs two `OperationTransform` entries, the turn and the move. An unturned body costs one. |
| T4 | A turn changes no volume. The footprint is read from the turned body's own bounds, grown by the bound of that box. |
| T5 | A nil option is `ErrDegenerate`. Bodies that fit neither way are `ErrDegenerate` and none moves. |

## 6. Free angles

`WithArrangeAngle` turns each body about +Z to the angle that makes its bounding-box footprint smallest. It reads
each trial's footprint from a copy of the body, so it needs no geometry of its own.

| Decision | Rule |
|---|---|
| A1 | The angles tried are the multiples of `step` from zero below a quarter turn. A further quarter turn swaps a footprint's width and depth and keeps its area, so nothing past it is tried. |
| A2 | The footprint of an angle is the bounding box of a `Copy` turned by it about the body's bounding-box centre, grown on every side by that box's own bound as in G4. The copy is retired once read. Angle zero reads the body itself. |
| A3 | The least footprint area wins, and the least angle among equals, so a body square to the axes does not turn. A step that misses the best angle gets the best multiple, which can leave a set refused that a finer step fits. |
| A4 | A chosen angle is a `Transform` about the bounding-box centre before the move, so a turned body costs two `OperationTransform` entries as in T3. The probe copies add `Copy` entries to the history. |
| A5 | The option combines with `WithArrangeTurns`: the quarter-turn packer then works on the turned footprints. |
| A6 | `step` is an angle (`ErrUnitKind`), finite (`ErrNotFinite`), not negative (`ErrNegativeMagnitude`), and nonzero with at most 360 multiples below a quarter turn (`ErrDegenerate`). The last option wins. A body that cannot be copied turned carries `Copy`'s refusal. |
| A7 | Refusals A6 names are raised before any trial, and a set that does not fit is `ErrDegenerate` with no body moved, as G7 says. |

## 7. Layers

`ArrangeBed.Height` lets bodies that the bed's surface cannot hold stack in layers.

| Decision | Rule |
|---|---|
| L1 | The zero `Value` is a single layer, G7's refusal included. A positive `Height` is the most the layers may stand above the bed's surface. A negative one is `ErrNegativeMagnitude`, a wrong unit kind `ErrUnitKind`, and one that is not a length `ErrUnitKind`. |
| L2 | `arrangeplan.PackLayers` takes the bodies in order. Each joins the current layer if that layer still packs with it, by `Pack` or by `PackTurning` when turns are allowed, and otherwise waits for a later layer; a body that fits no empty layer is refused. So a narrow body can join an early layer that a wide one before it could not. |
| L3 | A layer's base rests `Gap` above the top of the tallest body of the layer below. A body's height is its box's extent along Z grown by the bound of the box, so the bound never lets two layers touch. The first layer's base is the bed's surface. |
| L4 | The layers together, the gaps between them included, must stand no higher than `Height`, or the arrangement is `ErrDegenerate` and no body moves. |
| L5 | Each body moves by one translation as G5 says, its minimum Z landing on its layer's base. A quarter turn or an angle of `WithArrangeAngle` changes no body's extent along Z. |
| L6 | The tests state each layer's base from the bodies' own heights and compare the moved boxes with it, and check the limit on both sides. |
