# Emboss design

`Emboss` raises or engraves closed sketch profiles on a planar face of a solid. It extrudes each profile with
`Extrude` and joins the tools with `Union` (raised) or `Cut` (engraved). It adds the face-derived direction,
validation and one atomic commit, and no new geometry kernel. The public contract is in `agent-docs/api-design.md`
§8; this document owns the decisions behind it.

## 1. Contract

```go
func Emboss(ctx context.Context, b *Body, s *sketch.Sketch, profiles []*sketch.Profile, face FaceSelector, mode EmbossMode, height units.Value, opts ...EmbossOption) (*Body, error)
func WithEmbossDraft(angle units.Value) EmbossOption
func WithEmbossRimFillet(radius units.Value) EmbossOption
```

`EmbossMode` is `EmbossRaised` or `EmbossEngraved`. `Emboss` retires `b` and returns the result.

| Decision | Rule |
|---|---|
| E1 | `face` resolves to exactly one face with an analytic `Plane` surface, through `resolvePlanarFace`, which `Hole` shares. Any other surface is `ErrUnsupported`. |
| E2 | The sketch plane is the face plane. `embossplan.Plan` refuses a sketch origin more than `embossplan.PlaneTol` (1e-6 mm) off the face plane, and a sketch normal whose cross product with the outward normal is longer than `embossplan.ParallelTol` (1e-9). Both are `ErrDegenerate`. The sketch is never moved onto the plane. |
| E3 | `height` is a positive finite length. A raised relief stands `height` out of the face. An engraving is cut `height` into the body. |
| E4 | Each tool sweeps a `TwoSided` extent from the sketch plane. The relief side is `height` long and the other side is `embossplan.Margin` (1 mm). The tool therefore overlaps the body by the margin and never lies in the face plane, where a coplanar contact is `BooleanUnsupportedContact`. |
| E5 | The outward normal is `Face.NormalAt` at the plane origin. The sketch normal decides which `TwoSided` side takes the height: the `Along` side when it points outward, the `Against` side when it points inward. |
| E6 | A profile whose tool has a bounding box certifiably disjoint from the body's is `ErrDegenerate`. The test grows both boxes by their own bounds, so it can refuse and never admits. |
| E7 | A raised result whose volume is certifiably no larger than the body's, and an engraved result whose volume is certifiably no smaller, are `ErrDegenerate`. The tests compare readings with their bounds, so they can refuse and never admit. |
| E8 | `profiles` MUST NOT be empty or hold nil. Profiles keep their holes, so a ring relief leaves its island untouched. The profiles of one call MUST NOT touch each other in a shared plane. |

## 2. Joining

- Each tool is `Extrude(s, profile, TwoSided{…})`. A profile that `Extrude` refuses is refused by `Emboss` with the
  same error.
- The tools join `b` in profile order through `combineTools` (`tool_combine.go`) with no pre-merge. No body is
  registered until every step and the volume check succeed. A failure removes the tools and leaves `b` live and the
  document unchanged. The tools are live in the document between their build and the removal.

## 3. Exactness

`Emboss` returns what `Union` or `Cut` returns for the same bodies. The relief of a straight-walled profile on a
plate reads `Approximate` with a bound near 1e-11 mm³, because the join takes the faceted path
(`apitest/emboss_test.go`). A circular profile reads `Approximate` as it does in `Hole`.

## 4. Selecting a face after a relief

A face that a relief joined or cut loses the `CapEnd` and `CapStart` roles of the original extrusion, so
`FaceCreatedBy(CapEnd(b))` finds nothing on the result. `Facing` selects by outward normal. On a body with two
reliefs of different height on one side, `Facing` matches several faces and `Emboss` refuses the selection
(`TestEmbossRaisedOnTop` and `Example_emboss` select the face before the first relief).

## 5. Not covered

- A relief on a face that is not an analytic plane or cylinder, and a relief wrapped round a cylinder (§8).
- Text from a font. The caller supplies closed profiles; `agent-docs/missing-features.md` lists text profiles.
- A height that varies across the profile.

## 6. Draft

`WithEmbossDraft` leans the relief's walls. It passes `WithTaper` to the `Extrude` that builds each tool, so the lean,
its sign and its refusals are `Extrude`'s (`agent-docs/draft-design.md` §2).

| Decision | Rule |
|---|---|
| D1 | A positive angle narrows a raised relief toward its top and an engraving toward its floor: the wall at depth `t` from the face has moved in by `t·tan(angle)`. A negative angle widens them. The zero value is a straight wall. |
| D2 | The angle is `WithTaper`'s: an angle is required, a wrong unit kind is `ErrUnitKind`, and one at or past a right angle is `ErrDegenerate`. A nil option is `ErrDegenerate`. |
| D3 | The tool's one millimetre on the other side of the face (E4) leans the same way and lies inside the body, so the join sees the relief's own walls where they meet the face. |
| D4 | The result is what `Union` or `Cut` gives for the drafted tool. The tests compare the volume against `∫ (L − 2t·tan(angle))² dt` for a square relief within a relative `1e-9` and probe a point a drafted wall leaves as material. |

## 7. A rim fillet

`WithEmbossRimFillet` rounds the relief's rim with the evaluator's `Fillet`.

| Decision | Rule |
|---|---|
| R1 | The radius is a positive length: zero is `ErrDegenerate`, a negative value `ErrNegativeMagnitude` and a wrong unit kind `ErrUnitKind`. The last option wins. |
| R2 | The rim of a raised relief is every edge of the planar faces that face outward in the plane `height` out of the embossed face. The rim of an engraving is the edges of the inner loops of the planar faces that face outward in the embossed face's own plane, which is where the engraving's wall meets that face. Both are convex edges. |
| R3 | The relief is built, and its rim rounded, on a copy of `b` first, retired once read, so a fillet the evaluator refuses, or a rim not found in the result, leaves `b` live and the document unchanged. The copy leaves history entries. The face is resolved once, on `b`, and the copy is embossed by that plane, so a provenance selector need not match it. |
| R4 | The result is the emboss followed by `Fillet`, so the history holds `OperationEmboss` then `OperationFillet`. A round too large for the relief is `Fillet`'s refusal. |
| R5 | A circular relief's rim round removes the volume of `∫ 2π(R ∓ x)(r − √(r² − (r − x)²)) dx` over `x` from 0 to `r`, minus for a raised relief and plus for an engraving, which the tests compute independently. |

## 8. A relief on a cylindrical face

`face` may resolve to one analytic cylindrical face, a rod's wall or a bore's. The sketch's plane is then tangent to the
cylinder, and the profiles are projected square to that plane, not wrapped round the cylinder: a relief of straight
sides on a curved wall.

| Decision | Rule |
|---|---|
| E1 | The sketch plane's normal is square to the cylinder's axis, and the plane lies at the cylinder's radius from it, within `embossplan.PlaneTol`. Any other plane is `ErrDegenerate`, naming "not parallel" or "not tangent". The relief's height is measured from that plane, outward along the face's outward normal at the line of contact: away from the axis for a rod and toward it for a bore. |
| E2 | The plane touches the cylinder along a line, and the wall falls away from the plane on a rod and rises toward it in a bore by the sagitta `R − √(R² − w²)` at distance `w` from that line. The side of the tool that is left open reaches that far past the usual 1 mm margin, so the tool overlaps the body across the widest profile: a raised relief on a rod reaches in, an engraving in a bore reaches out, and the other two need nothing. `w` is read from the bounds of a private probe of each profile. A profile as wide as the cylinder (`w ≥ R`) is `ErrDegenerate`. |
| E3 | A relief on a cylindrical face takes `WithEmbossDraft` as on a plane and takes no rim fillet: `WithEmbossRimFillet` is `ErrUnsupported`. The result is `Union` or `Cut`'s for the same bodies, a mesh Boolean where the evaluator has no analytic route, with its own volume bound. |
| E4 | The evaluator refuses some axis-aligned pocket tools on a rod from its brep topology check ("a brep edge must bound exactly two distinct faces"); that is its own `ErrUnsupported` and leaves the body live. The engraving tests tilt the plane to 45 degrees about the axis, which it takes. |
| E5 | The tests state the added or removed volume from the integral of the relief's height over its width, `∫ (R + h − √(R² − y²)) dy` times the length for a raised relief on a rod, the wall above the floor for an engraving, and `∫ (√(R² − y²) − (R − h)) dy` for one in a bore, and require it inside the result's own bound. A relief wide enough that the sagitta exceeds the margin checks E2. |
