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
    Form     ThreadForm
    LeadIn   units.Value
}

type ThreadForm struct {
    Angle, Depth, Flat    units.Value
    FlankLead, FlankTrail units.Value
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
| T9 | A private coil built by `Thread` uses 128 stations per turn when the §5.4 departure estimate is at most `min(0.003 mm, 0.0006·ρ_max)` and each radial profile step is at most the inner radius; otherwise it uses 256. `Hole` and `Boss` thread tools use 256 stations per turn for their combined bore cuts. The shell's recorded displacement remains the bound for the result. |

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
- A runout that tapers the groove out, and a lead-in of another angle than 45 degrees.
- Tapered (pipe) forms and the UN inch series' own tables: a UN thread is the ISO profile at a pitch of `25.4 / threads per inch`.
- Tables beyond the metric coarse series, explicit metric pitches and the Unified fractional sizes of §9, such as ISO fine series by size, tapered pipe threads and trapezoidal tables. The caller states any other pitch.
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

## 6. Trapezoidal forms

`ThreadSpec.Form` replaces the ISO basic profile with a symmetric trapezoidal groove, for Acme, trapezoidal and
Whitworth-style threads. `threadplan.ComputeForm` takes the form, and `Compute` is `ComputeForm` with the zero form.

| Decision | Rule |
|---|---|
| F1 | The zero `ThreadForm` is the ISO profile of T5. A form that sets one or two of `Angle`, `Depth` and `Flat` is `ErrDegenerate`. |
| F2 | `Angle` is the included angle between the flanks, above zero and at most 120 degrees. Each flank widens the groove by `tan(Angle/2)` per unit of depth. A wrong unit kind is `ErrUnitKind`, a negative value `ErrNegativeMagnitude`, a non-finite one `ErrNotFinite`, and zero or past 120 degrees `ErrDegenerate`. |
| F3 | `Depth` is the root's distance from the face, below the major radius for a rod and beyond the minor radius for a bore, and `Flat` is the root's width. Both are positive lengths, read as `Pitch` is. |
| F4 | The groove is the four-corner polygon of T5 with these three numbers: root at `Depth`, mouth `0.05·P` past the face, and the width at the mouth `Flat + 2·tan(Angle/2)·(Depth + 0.05·P)`. T6 holds for it: a groove whose width reaches `0.95·P` or whose root reaches the axis is `ErrDegenerate`. |
| F5 | The removed volume of a groove that runs out of both ends is the integral of §3 with `w(ρ) = Flat + 2·tan(Angle/2)·(ρ − root)` for a rod, and the tests compute it independently. |
| F6 | A form with the ISO angle, depth and flat of the pitch gives the ISO thread's volume, so `ThreadForm` restates the profile and adds no second path. |

## 7. A lead-in chamfer

`ThreadSpec.LeadIn` bevels the threaded face's ends before the groove is cut, so a screw or a nut starts without a sharp
rim.

| Decision | Rule |
|---|---|
| L1 | The zero `Value` cuts no lead-in. A positive length is the distance of a 45 degree `Chamfer` on each convex circular edge of the threaded face: the rims at the ends of a rod, and the mouths of a bore. A negative value is `ErrNegativeMagnitude`, a wrong unit kind `ErrUnitKind`, and a face with no convex circular edge `ErrDegenerate`. |
| L2 | The chamfer comes first and the groove is cut from the chamfered body. The groove's frame and plan are those of the unchamfered face, so the thread is the same helix whatever the lead-in. A shoulder's concave edge is not chamfered. |
| L3 | The chamfer and thread run on a copy of `b`. The successful thread commit consumes `b` and the chamfered copy together. A refusal leaves `b` live. The copy leaves history entries. |
| L4 | The history holds `OperationChamfer` then `OperationThread`. |
| L5 | The removed volume of a rod threaded after a lead-in `d` is the chamfered rod's less `∫ 2π·ρ·w(ρ)/P·ℓ(ρ) dρ` over the groove's radii, where `ℓ(ρ)` is the length 20 less `2·max(0, d − (R − ρ))` the chamfer leaves at radius `ρ`. A groove covers `w(ρ)/P` of any axial interval at every radius on average over the turn, so the integral is exact. The tests compute it independently. |

## 8. Asymmetric grooves

`ThreadForm.FlankLead` and `FlankTrail` set the two flanks of the groove apart, for a buttress thread: a flank steep to the load and a shallow
one behind it. They replace `Angle`.

| Decision | Rule |
|---|---|
| B1 | Each flank is an angle above zero and under 90 degrees, its lean from the radial direction: `FlankLead` for the flank toward the start of the face (its least position along the axis) and `FlankTrail` for the one toward its far end. Both are set and `Angle` is not, or `ThreadForm` is symmetric as §6 says. Any other mix is `ErrDegenerate`. `Depth` and `Flat` are required as in F1. |
| B2 | The groove polygon is T5's with the mouth reaching `Flat/2 + h·tan(FlankLead)` from the middle of the root toward the start and `Flat/2 + h·tan(FlankTrail)` toward the far end, `h` the root's distance from the mouth. `threadplan.Form.BackSlope` is the far flank's slope, zero meaning the start flank's. |
| B3 | The first groove lies wholly before the start plane, so its middle sits the far reach and the margin before it, and the coil carries on until the start reach clears the far end: `(length + lead + trail + 2·margin)/pitch` turns. A groove whose two reaches together reach `0.95·P` is `ErrDegenerate`. |
| B4 | The volume of a groove that runs out of both ends depends on the sum of the two slopes alone, so it is F5's with `tan` of the mean: the tests compute that independently, and the polygon's own corners are pinned in `threadplan`. Which flank is which is read from the polygon, not from the volume. |

## 9. Looking a thread up

`LookupThread(designation)` reads a thread's major diameter and pitch from its name, so a caller writes `"M10"` for the pitch of 1.5 mm and
the 10 mm diameter instead of stating them.

| Decision | Rule |
|---|---|
| K1 | `M<d>` is an ISO metric thread at its ISO 261 coarse pitch, for a preferred nominal diameter `d` from M1 to M64 (1, 1.2, 1.4, 1.6, 1.8, 2, 2.5, 3, 3.5, 4, 5, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 27, 30, 33, 36, 39, 42, 45, 48, 52, 56, 60, 64). Another size has no coarse pitch and is `ErrDegenerate`. |
| K2 | `M<d>x<p>` states the pitch: any positive finite `p` below `d`, for any positive `d`. The letters M and x are read in either case. |
| K3 | `<n>/<m>-<t>`, or `<n>-<t>` for a whole number of inches, is a Unified thread of nominal diameter `n/m` inch, a multiple of 1/64 inch in the table from 1/4 to 1 inch, at `t` threads per inch, which must be that size's UNC or UNF value (20 or 28 for 1/4, 13 or 20 for 1/2, 8 or 12 for 1, and so on). The pitch is `25.4/t` mm and the diameter `n/m · 25.4` mm. |
| K6 | `#<n>-<t>` is a Unified numbered size: the nominal diameter of `#n` is `0.060 + 0.013 n` inch, for `n` in 0 to 6, 8, 10 and 12, at `t` threads per inch, which must be that size's UNC or UNF value (24 or 32 for `#10`, 32 or 40 for `#6`, and `#0` has only 80). The pitch is `25.4/t` mm. Any other number, a rate the size has not, or text after the rate is `ErrDegenerate`; a number without the `#` is no designation. The tests pin all nineteen sizes against the standard's table and refuse the near misses. |
| K7 | `G<size>` is an ISO 228 parallel pipe thread (BSPP) by its nominal bore in inches, one of 1/8, 1/4, 3/8, 1/2, 5/8, 3/4, 7/8, 1, 1-1/4, 1-1/2 and 2, written as the standard writes it: its major diameter is the standard's in millimetres (9.728 for G1/8 to 59.614 for G2) and its pitch is `25.4/t` mm at 28, 19, 14 or 11 threads per inch. The G or g is read in either case. Any other size is `ErrDegenerate`. The size states the diameter and pitch only: the 55 degree Whitworth profile and the tapered NPT and BSPT threads are not read, and `Thread` cuts the form its spec names. The test pins the eleven sizes and refuses near misses. |
| K4 | The result is a `ThreadSize` of two lengths. It names no tolerance class, allowance or minor diameter: a bore's drill diameter is the caller's to state from the thread's own tables. Any other designation, an empty one, one with a space, or a size not in the tables is `ErrDegenerate`. |
| K5 | The tests pin each table value against the standard's, a stated pitch and a Unified size against the converted inch, every refusal, and cut a rod with the size `"M10"` names to the volume of a 1.5 mm pitch thread. |

## 10. A lead-in at another angle

`ThreadSpec.LeadInAngle` sets the lead-in bevel's slope, for a start steeper or shallower than 45 degrees.

| Decision | Rule |
|---|---|
| A1 | `LeadInAngle` is an angle strictly between 0 and 90 degrees and needs `LeadIn`: without a distance it is `ErrDegenerate`, as are zero as an angle, 90 degrees or more, and a negative angle `ErrNegativeMagnitude`, a wrong unit kind `ErrUnitKind`. The zero `Value` is 45 degrees and the chamfer of §7. |
| A2 | The bevel is `ChamferAngle` on the same edges as L1, with `LeadIn` the distance along the end face, the end face the reference and `LeadInAngle` the angle between bevel and end face, so the bevel runs `LeadIn·tan(angle)` along the threaded face. Its receivers, routes and refusals are those of `ChamferAngle`. |
| A3 | Order, copying and history are those of L2 to L4: `OperationChamfer` then `OperationThread`. |
| A4 | The volume is L5's with the end face receding `tan(angle)·(ρ − (R − d))` at radius `ρ` in place of `ρ − (R − d)`, and the tests compute it for 30 and 60 degrees independently, and require 45 degrees to equal the default. |
