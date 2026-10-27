# Sheet metal design

`SheetMetal` builds a bent strip of sheet metal as one solid and states its flat pattern. It is the part of a
sheet-metal environment that a verification oracle needs first: the bent shape and the blank it is cut from.

## 1. Contract

```go
func SheetMetal(ctx context.Context, d *Document, spec SheetMetalSpec) (*Body, SheetMetalReport, error)
```

| Decision | Rule |
|---|---|
| M1 | The spec is `Thickness`, `InnerRadius` and `Width` (positive lengths), a `KFactor` in (0, 1] (the zero Value takes 0.44), `Legs` (positive lengths, the flat portions) and `Bends` (nonzero angles under a half turn, one fewer than the legs; positive turns counter-clockwise in the strip's plane, negative clockwise). A wrong unit kind is `ErrUnitKind`, a negative length `ErrNegativeMagnitude`, a non-finite one `ErrNotFinite`, and every other violation `ErrDegenerate`. |
| M2 | The strip lies in the XY plane, its first leg from the origin along +X, and is `Width` deep along +Z; `Transform` places it. The centreline is the legs joined by arcs of radius `InnerRadius + Thickness/2` about a centre on the side each bend turns toward, so a leg meets its bend tangentially. |
| M3 | The outline is the centreline offset `Thickness/2` each side: lines along the legs and arcs about the bends' centres, the inner face of radius `InnerRadius` and the outer `InnerRadius + Thickness`, closed by a line across each end. It is extruded by `Extrude`, so the volume is `Thickness · Width · (Σ legs + Σ |angle| · (InnerRadius + Thickness/2))`. |
| M4 | An outline that is not one simple loop, as a strip whose legs come back over themselves, is refused by the sketch or by the profile record and builds nothing. |
| M5 | `SheetMetalReport` states the flat pattern: each bend's allowance, `|angle| · (InnerRadius + KFactor · Thickness)`, in the order of the bends, and `FlatLength`, the legs plus the allowances. The K-factor places the neutral axis and so changes the flat pattern only, never the shape. |
| M6 | The tests build an L bracket, a channel and a Z-bend, and bends of 37, -53, 135 and a pair of 120/-120 degrees; read volumes from the formula of M3, bounds written out by hand, point containment, and the flat pattern against the formula of M5; and refuse each bad input and a strip that closes on itself. A flat length that ignores the K-factor, or an arc drawn the wrong way for a right turn, fails them. |

## 2. Not covered

- Flanges on a body's edges, hems, tabs, reliefs, corner treatments, forms and a flat pattern as geometry (the blank's outline).
- Bends whose sections change along the bend line, conical bends, and K-factor tables by material.
