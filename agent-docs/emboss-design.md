# Emboss design

`Emboss` raises or engraves closed sketch profiles on a planar face of a solid. It extrudes each profile with
`Extrude` and joins the tools with `Union` (raised) or `Cut` (engraved). It adds the face-derived direction,
validation and one atomic commit, and no new geometry kernel. The public contract is in `agent-docs/api-design.md`
§8; this document owns the decisions behind it.

## 1. Contract

```go
func Emboss(ctx context.Context, b *Body, s *sketch.Sketch, profiles []*sketch.Profile, face FaceSelector, mode EmbossMode, height units.Value) (*Body, error)
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
(`TestEmbossRaisedOnTop` and `Example_decad_emboss` select the face before the first relief).

## 5. Not covered

- A relief on a face that is not an analytic plane, such as a cylinder wall.
- Text from a font. The caller supplies closed profiles; `agent-docs/missing-features.md` lists text profiles.
- A relief with a draft angle, a fillet on its rim, or a height that varies across the profile.
