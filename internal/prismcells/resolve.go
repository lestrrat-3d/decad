package prismcells

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/sketch"
)

// BuildScene is §4.1's scene construction: one private sketch.Sketch
// holding both operands' recorded entities — Outer AND every Holes[i] loop of
// each, so a Cut target's own carried-through holes and a nested Intersect
// operand's own boundary both reach the arrangement (Union's own admitted
// pairs are hole-free by G6, so this is a no-op widening for that path).
// Operand A's segments are created verbatim (A's frame is the reference);
// operand B's are re-expressed into A's frame first (Reexpression.MapPoint) —
// the one new rounding this design introduces. When that map is a
// reflection, B's record is mapped and re-wound first
// (Reexpression.Rewind) and its entities are built from that record. Entities are deduplicated
// WITHIN each operand (the same dedup key discipline internal/momentinput/reconstruct.go's
// momentRecordScene already uses for one record) but NEVER across operands: a
// coincident carrier is handed to sketch as two separate, numerically
// matching entities, and sketch's own coincident-carrier resolution decides
// whether they merge.
//
// The second return value is §4.1's tag map: which operand and which loop
// each created entity traces to (record.go's own "outer loops CCW, holes CW"
// convention is unaffected — this is provenance, not orientation). Union's
// resolution ignores it; Cut/Intersect's clean-nesting match
// (payload_nesting.go, §4.2) reads it to prove the nesting relation
// structurally.
//
// Every entity is built from the segment's own WALKED geometry (walkOf),
// never from a Partial segment's recorded Center/Start/End as if it named a
// whole curve. A Partial segment's Start/End/Center are its SOURCE entity's
// own defining data, verbatim (record.go), which is exactly what a whole
// curve's fields hold too — nothing in a CurveSegment's own shape marks a
// fragment as partial. Recreating that source curve WHOLE is correct for an
// operand recorded straight from a live sketch profile (a genuine curve that
// may cross the OTHER operand beyond this operand's own walked range, which
// is exactly why momentRecordScene does the same for a single record's own
// self-consistency check). It stops being correct once an operand is itself
// a prior prism-boolean result: a Partial segment surviving THAT merge
// traces to one of the ORIGINAL pre-merge operands' own walls, and the
// portion beyond its walked range is now genuinely INTERIOR material, not a
// boundary of anything — recreating it whole resurrects a wall this
// operand's own solid does not have, which can silently misclassify a
// region in whatever new arrangement it enters. Every segment's OWN walked
// portion, and nothing more, is what this operand's boundary IS, whole or
// partial alike — so that is what gets built.
//
// A segment whose recorded range narrows its own natural domain enters the
// scene at that WALKED endpoint — a coordinate this evaluator computed, not
// one the record states verbatim — and WalkChargeOf's allowance for it is
// accumulated into the returned SceneDelta, the largest such charge over
// each operand's own consumed segments (§7's δ_walk).
func BuildScene(budget *proofbound.WorkBudget, pa, pb momentinput.Profile, reexpress *Reexpression) (*sketch.Sketch, map[sketch.Entity]Origin, SceneDelta, error) {
	return BuildProfileScene(budget, []momentinput.Profile{pa}, []momentinput.Profile{pb}, reexpress)
}

// BuildProfileScene adapts root profile records and the composed
// placement to prismcells' private scene builder.
func BuildProfileScene(budget *proofbound.WorkBudget, regionsA, regionsB []momentinput.Profile, reexpress *Reexpression) (*sketch.Sketch, map[sketch.Entity]Origin, SceneDelta, error) {
	s, tags, charge, err := BuildSceneRegions(budget, SceneProfiles(regionsA), SceneProfiles(regionsB), reexpress)
	return s, tags, SceneDelta{A: charge.A, B: charge.B}, err
}

func SceneProfiles(regions []momentinput.Profile) []SceneProfile {
	out := make([]SceneProfile, len(regions))
	for i, region := range regions {
		out[i] = SceneProfile{Outer: region.Outer, Holes: region.Holes}
	}
	return out
}

// ResolveUnion is §4.2's hole-free select-all/merge/chain path. It
// returns the merged section, BuildScene's own per-operand §7 walk
// charge (sceneDelta), and §7's cut displacement over the merge — the largest
// allowance any surviving fragment's own cut parameters owe, zero when every
// survivor is a whole edge.
//
// sceneDelta also carries the crossing charge (SceneDelta.crossing) for
// every cut the arrangement made.
//
// resolved=false (err always nil in that case) means the pair's topology is
// unresolved (§4.4), or a cut carries an amplified displacement the crossing
// charge cannot bound or the merge then fails on: the caller falls back to
// the mesh path with no error. A non-nil error — including ctx cancellation
// surfacing through budget — is genuine and must propagate.
func ResolveUnion(ctx context.Context, budget *proofbound.WorkBudget, pa, pb momentinput.Profile, deltaA, deltaB float64, reexpress *Reexpression) (momentinput.Profile, SceneDelta, float64, bool, error) {
	s, tags, sceneDelta, err := BuildScene(budget, pa, pb, reexpress)
	if err != nil {
		return momentinput.Profile{}, SceneDelta{}, 0, false, err
	}
	if err := budget.Err(); err != nil {
		return momentinput.Profile{}, SceneDelta{}, 0, false, err
	}
	profiles, err := CellProfiles(ctx, budget, s)
	if err != nil {
		return momentinput.Profile{}, SceneDelta{}, 0, false, err
	}
	if err := budget.Err(); err != nil {
		return momentinput.Profile{}, SceneDelta{}, 0, false, err
	}
	if err := budget.Step(); err != nil {
		return momentinput.Profile{}, SceneDelta{}, 0, false, err
	}
	if len(profiles) == 0 {
		return momentinput.Profile{}, SceneDelta{}, 0, false, nil // §4.4: the scene holds no bounded cell at all
	}
	// agent-docs/general-boolean-design.md §3 A6: every crossing the arrangement
	// cut is charged the input displacement it amplifies; a crossing too
	// close to tangent for that charge sends the pair to the mesh path.
	if ok, err := sceneDelta.ChargeCrossings(budget, tags, profiles, deltaA, deltaB, reexpress.Delta); err != nil || !ok {
		return momentinput.Profile{}, SceneDelta{}, 0, false, err
	}

	// Union, hole-free operands (G6): select every returned cell, which is
	// the union once no cell is material of neither operand. Two hole-free
	// operands can still enclose such a cell between them, so a scene with
	// one is unresolved (§4.2, §4.4). Merge is the shared
	// merge/chain tail the crossing sub-case (internal/prismcells/payload_resolve.go)
	// reuses over its OWN, narrower selected cell set.
	voidFree, err := CellsHaveNoVoid(budget, tags, profiles)
	if err != nil {
		return momentinput.Profile{}, SceneDelta{}, 0, false, err
	}
	if !voidFree {
		return momentinput.Profile{}, SceneDelta{}, 0, false, nil
	}
	if ok, err := sceneDelta.SharedSpansBounded(budget, profiles); err != nil || !ok {
		return momentinput.Profile{}, SceneDelta{}, 0, false, err
	}
	merged, cutDelta, resolved, err := Merge(budget, profiles, "union")
	if fallBack, err := AmplifiedFallback(sceneDelta.Amplified, err); fallBack || err != nil {
		return momentinput.Profile{}, SceneDelta{}, 0, false, err
	}
	if !resolved {
		return momentinput.Profile{}, SceneDelta{}, 0, false, nil // §4.4: not a shape this increment covers
	}
	return merged, sceneDelta, cutDelta, true, nil
}

// ResolveCrossing is the shared resolution shape behind both builders
// above: ResolveCrossingCells's selection (scene, classification,
// per-op keep, crossing charge), then Merge's assembly.
// resolved=false (err always nil in that case) is silent
// fallback, which includes a near-tangent crossing A6's charge cannot bound
// and a merge failure on cuts that carry an amplified displacement. opName
// feeds Merge's own RB1 message.
func ResolveCrossing(ctx context.Context, budget *proofbound.WorkBudget, pa, pb momentinput.Profile, deltaA, deltaB float64, reexpress *Reexpression, keep func(a, b bool) bool, opName string) (momentinput.Profile, SceneDelta, float64, bool, error) {
	selected, sceneDelta, resolved, err := ResolveCrossingCells(ctx, budget, pa, pb, deltaA, deltaB, reexpress, keep)
	if err != nil {
		return momentinput.Profile{}, SceneDelta{}, 0, false, err
	}
	if !resolved {
		return momentinput.Profile{}, SceneDelta{}, 0, false, nil
	}

	merged, cutDelta, mergedResolved, err := Merge(budget, selected, opName)
	if fallBack, err := AmplifiedFallback(sceneDelta.Amplified, err); fallBack || err != nil {
		return momentinput.Profile{}, SceneDelta{}, 0, false, err
	}
	if !mergedResolved {
		return momentinput.Profile{}, SceneDelta{}, 0, false, nil
	}
	return merged, sceneDelta, cutDelta, true, nil
}

// ResolveCrossingCells is §4.2's crossing sub-case selection alone, the
// first half of ResolveCrossing's own shape: build the private scene,
// classify every cell (Classify), select the op's own subset
// (keep), and charge every crossing the operands' incoming displacement can
// move (A6, SceneDelta.ChargeCrossings) — stopping short of Merge's
// assembly tail, which requires the selected cells to chain into one closed
// loop and so cannot answer a multi-region selection at all
// (agent-docs/prism-boolean-design.md §4.4). ResolveCrossing above is this
// task's own caller; §4.5's overlap-area reading
// (prismcells.OverlapVolume) is the second, sharing this exact
// selection so it never re-derives §4.2's classification.
//
// resolved=false (err always nil in that case) is silent fallback (§4.4),
// including a crossing too close to tangent for A6's charge.
func ResolveCrossingCells(ctx context.Context, budget *proofbound.WorkBudget, pa, pb momentinput.Profile, deltaA, deltaB float64, reexpress *Reexpression, keep func(a, b bool) bool) (selected []*sketch.Profile, sceneDelta SceneDelta, resolved bool, err error) {
	s, tags, sceneDelta, err := BuildScene(budget, pa, pb, reexpress)
	if err != nil {
		return nil, SceneDelta{}, false, err
	}
	if err := budget.Err(); err != nil {
		return nil, SceneDelta{}, false, err
	}
	profiles, err := CellProfiles(ctx, budget, s)
	if err != nil {
		return nil, SceneDelta{}, false, err
	}
	if err := budget.Err(); err != nil {
		return nil, SceneDelta{}, false, err
	}
	if len(profiles) == 0 {
		return nil, SceneDelta{}, false, nil // §4.4: the scene holds no bounded cell at all
	}
	matterA, matterB, resolved, err := Classify(budget, tags, profiles)
	if err != nil {
		return nil, SceneDelta{}, false, err
	}
	if !resolved {
		return nil, SceneDelta{}, false, nil
	}

	selected, err = Select(budget, profiles, matterA, matterB, keep)
	if err != nil {
		return nil, SceneDelta{}, false, err
	}
	// agent-docs/general-boolean-design.md §3 A6, Union's own charge: every cut the
	// operands' incoming displacement can move is charged, and a crossing too
	// close to tangent for that charge sends the pair to the mesh path.
	if ok, err := sceneDelta.ChargeCrossings(budget, tags, profiles, deltaA, deltaB, reexpress.Delta); err != nil || !ok {
		return nil, SceneDelta{}, false, err
	}
	if ok, err := sceneDelta.SharedSpansBounded(budget, selected); err != nil || !ok {
		return nil, SceneDelta{}, false, err
	}
	return selected, sceneDelta, true, nil
}

// ResolveCut is §4.2's clean-nesting match for Cut(target, tool): when
// the tool's boundary does not touch the target's anywhere, the arrangement
// leaves both operands' original loops completely unmodified, and decad
// finds the one s.Profiles() result whose Outer structurally reproduces the
// target's own Outer and whose Holes structurally reproduce the target's own
// Holes plus EXACTLY ONE further hole reproducing the tool's own Outer (G6
// keeps the tool hole-free, so that one hole is the tool's whole solid).
// That profile IS the result, verbatim — no assembly.
//
// This is the discriminator the disjoint-footprint trap needs: a structural
// match on the tool's own cell alone would not by itself prove nesting, since
// two disjoint hole-free footprints also arrange into two untouched cells,
// each reporting its own Outer whole. Requiring the target's OWN cell to
// carry the tool's Outer as one of ITS holes is what a disjoint pair can
// never produce — the arrangement of two disjoint footprints yields two
// separate cells, neither with a hole at all.
//
// resolved=false (err always nil in that case) means the pair's topology is
// unresolved (§4.4) — the caller falls back to the mesh path with no error.
// A non-nil error is always genuine and must propagate. The returned
// *sketch.Sketch is the private scene the match was found in, needed to
// authenticate it through RecordProfile, and sceneDelta is BuildScene's
// own per-operand §7 walk charge.
func ResolveCut(ctx context.Context, budget *proofbound.WorkBudget, target, tool momentinput.Profile, reexpress *Reexpression) (*sketch.Sketch, *sketch.Profile, SceneDelta, bool, error) {
	s, match, _, delta, resolved, err := ResolveCutWithTags(ctx, budget, target, tool, reexpress)
	return s, match, delta, resolved, err
}

// ResolveCutWithTags also returns the scene's entity-origin map. A
// stacked result uses it to retain the target's already recorded whole loops
// while taking only the new tool hole from RecordProfile's authenticated cell.
func ResolveCutWithTags(ctx context.Context, budget *proofbound.WorkBudget, target, tool momentinput.Profile, reexpress *Reexpression) (*sketch.Sketch, *sketch.Profile, map[sketch.Entity]Origin, SceneDelta, bool, error) {
	s, profiles, tags, sceneDelta, err := CutCells(ctx, budget, target, tool, reexpress)
	if err != nil {
		return nil, nil, nil, SceneDelta{}, false, err
	}
	match, resolved, err := MatchCut(budget, tags, profiles, len(target.Holes))
	if err != nil || !resolved {
		return nil, nil, nil, SceneDelta{}, false, err
	}
	return s, match, tags, sceneDelta, true, nil
}

// CutCells keeps both the scene and sketch's arranged cells available to
// the clean and enclosing whole-loop matches without arranging a pair twice.
func CutCells(ctx context.Context, budget *proofbound.WorkBudget, target, tool momentinput.Profile,
	reexpress *Reexpression) (*sketch.Sketch, []*sketch.Profile,
	map[sketch.Entity]Origin, SceneDelta, error) {
	s, tags, sceneDelta, err := BuildScene(budget, target, tool, reexpress)
	if err != nil {
		return nil, nil, nil, SceneDelta{}, err
	}
	if err := budget.Err(); err != nil {
		return nil, nil, nil, SceneDelta{}, err
	}
	profiles, err := ProfilesContext(ctx, s.Profiles)
	if err != nil {
		return nil, nil, nil, SceneDelta{}, err
	}
	if err := budget.Err(); err != nil {
		return nil, nil, nil, SceneDelta{}, err
	}
	return s, profiles, tags, sceneDelta, nil
}

// ResolveIntersect is §4.2's clean-nesting match for Intersect(a, b):
// run the search in both directions (X=a,Y=b and X=b,Y=a) since Intersect's
// relation is symmetric (§3.2) — a caller may pass the smaller body first.
// Each direction asks the same question ResolveCut does for its own
// nesting proof: is X's own cell reported with Y's Outer as one further hole?
// That is what closes the disjoint-footprint trap here too. Once a direction
// proves nesting, the RESULT is a separate s.Profiles() candidate: the
// profile whose Outer reproduces the NESTED operand's own Outer, with its
// admitted hole if present — that operand's own cell, untouched. If both
// directions match,
// or neither does, the topology is unresolved (§4.4).
//
// resolved=false (err always nil in that case) means the pair's topology is
// unresolved. nestedIsB reports which operand the result traces to, for
// §7's displacement selection. The returned *sketch.Sketch is the private
// scene the match was found in.
func ResolveIntersect(ctx context.Context, budget *proofbound.WorkBudget, pa, pb momentinput.Profile, reexpress *Reexpression) (s *sketch.Sketch, match *sketch.Profile, sceneDelta SceneDelta, nestedIsB, resolved bool, err error) {
	s, tags, sceneDelta, err := BuildScene(budget, pa, pb, reexpress)
	if err != nil {
		return nil, nil, SceneDelta{}, false, false, err
	}
	if err := budget.Err(); err != nil {
		return nil, nil, SceneDelta{}, false, false, err
	}
	profiles, err := ProfilesContext(ctx, s.Profiles)
	if err != nil {
		return nil, nil, SceneDelta{}, false, false, err
	}
	if err := budget.Err(); err != nil {
		return nil, nil, SceneDelta{}, false, false, err
	}
	if len(profiles) == 0 {
		return nil, nil, SceneDelta{}, false, false, nil
	}

	match, nestedIsB, resolved, err = MatchIntersect(budget, tags, profiles, len(pb.Holes))
	if err != nil || !resolved {
		return nil, nil, SceneDelta{}, false, false, err
	}
	return s, match, sceneDelta, nestedIsB, true, nil
}

// RequireRegionsWithinCap refuses a two-region scene whose arranger segment charge exceeds
// MaxArrangementSegments.
func RequireRegionsWithinCap(budget *proofbound.WorkBudget, pa, pb momentinput.Profile) error {
	segments, withinCap, err := RegionsWithinWorkCap(budget, pa, pb)
	if err != nil {
		return err
	}
	if !withinCap {
		return WithWorkLimit(fmt.Errorf(
			`%w: the analytic union scene charges at least %d arranger segments against this evaluator's cap of %d (each circle or arc costs 256, each line 1)`,
			decaderr.ErrUnsupported, segments, MaxArrangementSegments), segments)
	}
	return nil
}
