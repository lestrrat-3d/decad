# Revolve local angular refinement

Extends `agent-docs/tessellation-design.md` tier T5 ("deterministic local meridian refinement and global
angular density improvements"). This document owns non-uniform angular stations for a revolve mesh.
`agent-docs/tessellation-design.md` §3 (facet ceilings), §8 (sampling), §9 (cells), §10 (boundary proofs) and
§11 (angular homotopy) stay authoritative for everything not changed here.

## 1. Problem

A revolve mesh is one meridian chording times ONE global angular count `nPhi`
(`revolvetess.buildRevolveMesh`, `revolvemesh.AngularSequence`). A boolean chords every operand at the pair
tolerance, `meshbool.ChordFactor` (2e-5) times the pair diameter. A long arc of a thick tube then needs more
facets than `maxFacetsPerMesh` (65 536) allows.

Measured on a torus of major radius 4 and minor radius 1 unioned with a radius-1 stem ending on the tube
centreline (`apitest/revolve_local_refinement_test.go`):

| Sweep | nPhi | Facets at the pair tolerance | Result |
|---|---|---|---|
| 300° | 347 | 129 452 | refused, per-mesh ceiling |
| 180° | 228 | 93 886 | refused, per-mesh ceiling |
| 60° | 84 | 38 246 | refused, facet-pair audit ceiling (`maxFacetPairTestsPerCall`) |
| 20° | 30 | 14 814 | succeeds |

Facets scale as 1/tolerance, so the 300° arc at the pair tolerance is about twice the ceiling before any
refinement near a contact.

## 2. Decision that changes a guarantee

Non-uniform stations cannot reduce the facet count of a region that must keep the pair tolerance. To fit the
ceiling, a revolve meshed as a BOOLEAN OPERAND is chorded in two regions:

- the **contact window**, where the operand may touch its partner, at the pair tolerance `τ_pair`;
- the **rest**, at `τ_far = 2^k · τ_pair`, for the smallest `k` whose facet count fits the ceilings.

Scope and limits:

1. The coarser chording applies only to a boolean's internal operand chord (`meshbool.OperandChord` and its
   callers). `Body.Tessellate(tol)` keeps `agent-docs/tessellation-design.md` §1 and §3 unchanged, including
   "a refinement may NEVER coarsen". §3 gains the words "boolean operand chording" where it states that a
   requested tolerance bounds chording.
2. `k` is capped by a fixed constant, `k ≤ 3` (so `τ_far ≤ 8 · τ_pair`). A plan needing more refuses with
   `ErrUnsupported`. A caller cannot set the cap.
3. The window test inflates both boxes by `τ_far`, so a facet whose chording could reach across the gap is
   inside the window.
4. A revolve mesh publishes no per-vertex bound. The face bound `sourceBound(face)` of a revolve wall is
   therefore `τ_far` over the WHOLE face, window included, and every downstream figure that reads it — rim
   bound, hidden-tangency slack, result vertex bounds, `Mesh.Bound()` — reads that figure. This document does
   not claim window facets carry `τ_pair` in any published bound. Publishing a per-vertex revolve bound is a
   separate increment, outside S1–S5.

The guarantee that changes is therefore: a boolean over a revolve whose pair tolerance would exceed the ceilings
reports bounds up to 8x the pair tolerance on that operand's walls, and no other path does. This decision needs
the user's confirmation before S4 lands. S1–S3 change no guarantee.

## 3. Shared angular stations

A revolve emits one ring of vertices per angular station and one cell per meridian segment per angular
interval. Rings must share stations across ALL meridian samples of ALL loops, or the walls crack at a
station one ring has and its neighbour lacks. Refinement is therefore in angle only and applies to every
meridian segment at once: the angular plan is a list of stations, not a count.

A plan is `n0` uniform base intervals over the sweep. Each base interval may be bisected, recursively, to a
depth of at most `D`. Every station is a rational fraction `l / (n0 · 2^D)` of the sweep, so
`AngularSequence` encloses `sin` and `cos` for each station exactly as it does today, from the same exact
fraction. A uniform plan (no bisection) is the special case `D = 0` and MUST produce bit-identical meshes and
proof records to today's output.

A full turn closes its last interval onto station 0 and emits no seam ring, exactly as today.

## 4. What becomes per-interval

Each of these reads `nPhi` or one `Step` today and becomes a sum over intervals `l`, each with its own step
`s_l`:

| Quantity | Today | After |
|---|---|---|
| Wall triangles | `EmitCellTriangles(lo, hi, nPhi, …)` loops `l < nPhi` | loops over the interval list; the diagonal choice per cell is unchanged |
| Area slack | `nPhi · CellSlack(angular.Step)` | `Σ_l CellSlack(s_l)`; intervals with equal step are summed once and multiplied |
| Swept volume | `nPhi · Σ_cells RevolveCellSweptVolume(…, factor(Step))` | `Σ_l Σ_cells RevolveCellSweptVolume(…, factor(s_l))` |
| Angular homotopy factor | one `RevolveAngularHomotopyFactor(Step)` | one per distinct step, memoised on the exact step as today |
| Angular sagitta in the chord check | `ChordSagitta(rho, Sweep, nPhi)` | `ChordSagitta` at the largest step in the plan |
| Cap `last` index | `angular.Samples - 1` | last station index |
| Facet preflight | `per · nPhi` per wall segment | `per · (number of intervals)` |

Rotating a cell about the axis is an isometry, so the cell reading depends on the interval only through its
step. That is why each reading is a pure function of `s_l` and the sums above are sound with no new argument.
The first test obligation is that, for a plan whose intervals all have one step, every sum equals today's
product to the last bit.

## 5. Choosing the plan

The planner runs before any mesh exists and uses only the pair's inflated bounding boxes
(`meshbool.ChordOperand`).

1. The contact window of an operand is the intersection of its own inflated box with the partner's inflated
   box, grown by the proof allowance already charged for placement and chording.
2. A base interval needs the fine step when the operand's swept cells over it can meet the window: the
   interval's angular extent, swept over the operand's meridian radius range, intersects the window. This is a
   conservative test; a missed window can only cost facets, never soundness, and an interval it flags
   wrongly is merely refined.
3. Flagged base intervals are bisected until their step meets the pair tolerance. All other intervals keep the
   step `τ_far` gives.
4. `τ_far` is the smallest `2^k · τ_pair` for which `PreflightFacets` fits both facet ceilings.

The plan is a deterministic function of the operand, the partner's box and the tolerance. It never reads a
mesh. A refinement retry (§3 of the tessellation design) bisects the intervals around the first failing cell
instead of incrementing one global count.

## 6. Refinement work

A facet-pair audit that exceeds its work ceiling is final. Refining on it only adds facets and pair tests, and
each retry charges its whole mesh to the cumulative `maxFacetWorkPerCall` again; the refusal a caller then read
was that facet ceiling, not the audit's. `tessellateRevolve` returns the audit's own refusal at once.

An angular failure the audit can still answer (a crossing or an undecided contact) bisects the failing base
interval once §3's plan exists, rather than adding one global station.

## 7. Contact audit

The facet-contact audit (`revolvemesh.revolve_contact_audit.go`) scans spatial candidates among the facets.
Facets in the fine window are the only ones near the partner, so candidate counts fall with the plan. The
audit's predicates and required-contact rules do not change; only the facet set it reads does.

## 8. Increments

Each increment passes its own tests and relaxes no ceiling.

| Stage | Lands | Guarantee changed |
|---|---|---|
| S1 | An audit work-ceiling refusal is returned at once (§6). The 60° repro reports the audit ceiling. | none |
| S2 | Station-list angular plan; uniform plan bit-identical (§3, §4 rows for triangles, caps, preflight). | none |
| S3 | Bisecting angular retries (§6). Per-interval area slack, swept volume and homotopy factor (§4). Tests compare a hand-built non-uniform plan against the exact volume and area of the true solid. | none |
| S4 | Planner with a contact window and `τ_far` (§5), wired into the boolean's operand chord; face bounds per §2.4. Needs S3. | §2 |
| S5 | Contact audit reads the windowed facet set (§7). The 300° repro and the Gallery yarn hook succeed. | none |

## 9. Test obligations

- A uniform plan reproduces today's mesh, proof record and volume bound exactly.
- A non-uniform plan over a sphere, a torus arc and a cone gives a volume within the proof record's own bound
  of the analytic volume, and a watertight mesh with no cracks at a station joining a fine and a coarse
  interval.
- Every fixture is shown to fail first: remove the per-interval step from one sum and the volume test goes
  red.
- The torus-plus-stem repro at 300° succeeds, and its union volume matches the analytic value within the
  reported bound. S3's per-interval proofs land and pass before S4 is wired in.
- No test pins a `Bound` literal; bounds differ between amd64 and arm64.
