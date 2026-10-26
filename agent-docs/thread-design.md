# Thread design

`Thread` cuts an ISO metric thread into a cylindrical face. It sketches the groove's axial section, `Coil`s it and
`Cut`s the coil from the body, as `agent-docs/helix-design.md` §9 states the thread use case. It adds a tool-building
wrapper and no geometry kernel. The groove arithmetic is plain-data code in `internal/threadplan/`.

## 1. Contract

```go
type ThreadSpec struct {
    Face     FaceSelector
    Pitch    units.Value
    LeftHand bool
    Length   units.Value
}

func Thread(ctx context.Context, b *Body, spec ThreadSpec) (*Body, error)
```

| Decision | Rule |
|---|---|
| T1 | `Face` resolves to exactly one face of a live solid `b`, and that face's surface is a `Cylinder`. A selector that matches other than one face is `ErrDegenerate`, and a face of another surface is `ErrUnsupported`. |
| T2 | A face whose outward normal points away from the axis is external: a rod, whose radius is the thread's major radius. A face whose outward normal points toward the axis is internal: a bore, whose radius is the thread's minor (drilled) radius. |
| T3 | `Pitch` is a positive finite length, the axial advance per turn. Zero is `ErrDegenerate`, a negative value is `ErrNegativeMagnitude` and a wrong unit kind is `ErrUnitKind`. |
| T4 | The thread is right-handed unless `LeftHand` is set, which gives `Coil` its `WithLeftHand` option (§5). |
| T5 | The groove profile is the ISO 68-1 basic profile restated as a four-corner polygon: flanks 30 degrees off radial, an external root flat of `P/8` at depth `0.61343·P` below the major radius, and an internal root flat of `P/4` at depth `0.54127·P` beyond the minor radius. Each groove's mouth opens `0.05·P` past the surface so the cut crosses it transversally. |
| T6 | A groove too wide for the pitch (its axial width at or above `0.95·P`), or one whose root reaches the axis, is `ErrDegenerate`. |
| T7 | The coil starts with its first groove wholly before the face's start plane and carries past the face's far end, so the groove runs out of both ends and every point of the face's length is cut at every angle. |
| T8 | The coil is cut from `b` by one `Cut` with a private tool. The result is a mesh Boolean's, faceted, and the history entry is `OperationThread`. A failure leaves `b` live and the document unchanged. |

## 2. Frame

The groove profile is sketched on the plane that holds the cylinder's axis. Its origin is the axis point at the face's
start, found as the least projection of the face's vertices on the axis. Its V axis is the cylinder's axis and its U
axis a radial direction perpendicular to it. `Coil` advances along V and turns right-handed about it, and `LeftHand` turns it the other way.

## 3. What the result reads

A groove that runs out of both ends removes, at each radius `ρ` between the root and the major radius `R`, the fraction
`w(ρ)/P` of the length `L` at every angle, where `w(ρ)` is the groove's axial width at `ρ`. The removed volume is

`∫ 2π·ρ·L·w(ρ)/P dρ` over the root to `R` (external) or the minor radius to the root (internal).

The tests compute this integral independently and require the published volume interval to enclose `V₀ − removed`.
The interval comes from the mesh Boolean's composed bounds, so it is wider than the closed form and does not state it
exactly.

## 4. Not covered

- A thread on a face beside a larger face or a shoulder. The groove runs out of the face's ends and cuts the
  neighbour. Thread a rod that ends at the thread, or cut the thread before adding the shoulder.
- A lead-in chamfer and a runout that tapers the groove out.
- Tapered (pipe) and trapezoidal forms, and the UN inch series.
- A thread table lookup. The caller states the pitch.
- Cosmetic threads that mark a face and cut nothing.
- Fit classes and tolerances: the profile is the basic profile with no allowance.

## 5. Hand and length

| Decision | Rule |
|---|---|
| H1 | `LeftHand` cuts the same groove with the coil turning the other way. Both hands remove the same volume from a whole rod, and a quarter turn on the groove is a quarter pitch lower instead of higher. |
| H2 | `Length` is the axial length to thread, measured from the end of the face with the least position along the axis (§2). The zero `Value` threads the whole face. |
| H3 | A `Length` that is not a positive length is `ErrDegenerate`, `ErrNegativeMagnitude` or `ErrUnitKind`. One longer than the face is `ErrDegenerate`. |
| H4 | With a `Length` the coil stops where its last groove's far flank meets the length, so the groove never reaches past it and the thread ends with the coil's flat end. The first groove still starts wholly before the start plane (T7). |
| H5 | Past the threaded length the face is whole at every angle, and `threadplan.Compute` states the turns with `partial` set. |

The volume a partial thread removes has no closed form in one line: the tests integrate the groove's axial overlap with
`[0, length]` over radius and angle, and require the published interval to enclose `V₀` less that.

