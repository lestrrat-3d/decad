# Revolve operands past the facet ceilings

A boolean with a revolve operand whose mesh at the pair tolerance exceeds a work ceiling requests both operands
coarser, up to eight times the pair tolerance. `agent-docs/tessellation-design.md` §3 owns the ceilings;
this document owns the retry.

## 1. Problem

A boolean chords every operand at the pair tolerance, `meshbool.ChordFactor` (2e-5) times the pair diameter
(`internal/meshbool/chord.go`). A revolve mesh is one meridian chording times one global angular count, so a long
arc of a thick tube asks for more facets than `maxFacetsPerMesh` (65 536) allows, or more facet-contact work than
`maxFacetPairTestsPerCall` allows. Facets scale as 1/tolerance.

A torus of major radius 4 and minor radius 1, unioned with a radius-1 stem ending on its tube centreline, at the
pair tolerance (`apitest/revolve_local_refinement_test.go`):

| Sweep | Angular stations | Facets | Refusal |
|---|---|---|---|
| 300° | 347 | 129 452 | per-mesh facet ceiling |
| 180° | 228 | 93 886 | per-mesh facet ceiling |
| 60° | 84 | 38 246 | facet-pair audit ceiling |
| 20° | 30 | 14 814 | none |

## 2. The retry

`booleanBodyWith` runs the pair at the pair tolerance first. When a revolve operand's mesh is refused with a
work-limit error (`proofbound.WorkLimitError`: the facet ceiling or the facet-pair audit ceiling),
`booloperand.CoarsensOperand` says so and the pair is retried on rungs 1, 2 and 3. Rung `r` requests BOTH operands
at `2^r` times their pair request, so neither is far finer than its partner. The first rung that answers wins.
When no rung answers, the call reports the refusal the pair tolerance hit.

- The request is at most eight times the pair tolerance. `booloperand.MaxOperandCoarsenings` (3) is a constant;
  no caller sets it.
- Only a boolean's own operand request coarsens. `Body.Tessellate` never does, and `agent-docs/tessellation-design.md`
  §3's "refinement may never coarsen" is unchanged for it.
- A held operand (`booloperand.HeldFloor`) keeps its floor: its request is `max(request, floor)` on every rung.
- Each mesh publishes the chording it actually took, so its face bounds, the rim bound, the hidden-tangency slack,
  the result's vertex bounds and `Mesh.Bound()` all read that figure. The result of a retried pair reports bounds
  up to eight times the pair tolerance on its walls. No path other than this retry does.
- A rung after the first may fail for a reason other than a ceiling (an undecided contact, a float fold). The
  retry continues to the next rung regardless. A pair-tolerance failure that was not a ceiling is returned at
  once and never retried.
- A facet-pair audit that exceeds its work ceiling inside one revolve tessellation is final for that request: a
  finer angular count only adds facets, so `tessellateRevolve` returns the audit's own refusal rather than
  refining and charging every attempt's mesh to the cumulative facet ceiling.
- No ceiling or budget changes.

## 3. Measured

The same torus-plus-stem pair succeeds at 60°, 180° and 300° with the retry. Each union volume matches the arc
volume plus the stem volume outside the arc, the latter by midpoint integration, within the reported bound.

## 4. Why not a finer mesh near the contact

A mesh refined only near the contact (non-uniform angular stations, a per-interval area and volume proof) was
designed first and not built. A revolve mesh publishes no per-vertex bound, so a face's published bound is its
coarsest chording, and a finer patch tightens no bound any proof reads. The retry gives the same bounds and
changes none of the revolve proofs.

## 5. Test obligations

- The 60°, 180° and 300° torus-plus-stem unions succeed, and each volume matches the analytic value within the
  reported bound and stays under two percent of the volume.
- Each case is shown to fail first: remove the retry and all three refuse.
- No test pins a `Bound` literal; bounds differ between amd64 and arm64.
