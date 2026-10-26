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
}

func Arrange(ctx context.Context, bodies []*Body, bed ArrangeBed, opts ...ArrangeOption) ([]*Body, error)
func WithArrangeTurns() ArrangeOption
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

- Turning bodies by any angle but a quarter turn about +Z, or following their outlines. Packing is by bounding rectangle.
- Packing in three dimensions or stacking bodies.
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

