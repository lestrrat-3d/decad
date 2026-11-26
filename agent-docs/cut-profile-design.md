# Cut from a sketch profile

`CutExtrude` and `CutRevolve` cut a solid by sketch profiles and an extent,
with no tool body in the caller's code. Each maps to one feature in CAD tools
that cut directly from a sketch (an extrude or revolve with the cut operation).

## 1. Behavior

- Input: a live solid target, a sketch, a non-empty list of profiles of that
  sketch, and an `Extent` (`CutExtrude`) or an `Axis` and `AngularExtent`
  (`CutRevolve`). `CutExtrude` has the `Taper` method that `Extrude` has, and
  neither cut has `SurfaceResult` (command-api-design M11).
- Each profile is swept by the ordinary `Extrude` or `Revolve`, so every gate,
  extent rule and refusal of those calls applies unchanged. Body-relative stops
  read live bodies, the target included.
- The tools are built under a private construction context: they are never
  public history and are retired before the call returns.
- With more than one profile, the tools are united in order and the target is
  cut once (`combineTools` with `merge`). Profiles whose tools overlap are one
  removed volume; the many-tool union avoids cutting a target that already
  holds a coarse mesh from an earlier cut.
- The result retires the target and records one `OperationCutExtrude` or
  `OperationCutRevolve` whose consumed input is the target.

## 2. Errors

A nil context, target or sketch, an empty profile list or a nil profile is
`ErrDegenerate`. A non-solid target is `ErrNotSolid`. A cut that removes
everything is `ErrBooleanFailed`. All other refusals are those of `Extrude`,
`Revolve` and `Cut`. Every failure leaves the target live and the document
unchanged.

## 3. Tests

`apitest/cut_profile_test.go` checks volumes and bounds for several disjoint
profiles, overlapping profiles, a revolved groove, the recorded operation kind,
the live-body set after the call, and the refusal cases.
