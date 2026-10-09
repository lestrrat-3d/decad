package prismcells

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismplacement"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/sketch"
)

// GroupScene contains one multi-region arrangement and its charged section
// displacement. Callers retain their own payloads for result publication.
type GroupScene struct {
	Sketch            *sketch.Sketch
	Tags              map[sketch.Entity]Origin
	Profiles          []*sketch.Profile
	Delta             SceneDelta
	ReexpressionDelta float64
}

// AdmitGroupRegions applies G4, G6 and the trimmed-circular refusal to every
// recorded region after the root caller has admitted the operand placements.
func AdmitGroupRegions(budget *proofbound.WorkBudget, regions []momentinput.Profile, holeFree bool) (bool, error) {
	for _, region := range regions {
		if holeFree && len(region.Holes) != 0 {
			return false, nil
		}
		analytic, err := ProfileAnalytic(budget, region)
		if err != nil || !analytic {
			return false, err
		}
		trimmed, err := ProfileHasTrimmedCircularSource(budget, region.Outer, region.Holes)
		if err != nil || trimmed {
			return false, err
		}
	}
	return true, nil
}

func groupWithinCap(budget *proofbound.WorkBudget, op string, regions ...[]momentinput.Profile) error {
	var all []momentinput.Profile
	for _, rs := range regions {
		all = append(all, rs...)
	}
	segments, withinCap, err := RegionsWithinWorkCap(budget, all...)
	if err != nil {
		return err
	}
	if !withinCap {
		return fmt.Errorf(
			`%w: the analytic %s scene charges at least %d arranger segments against this evaluator's cap of %d (each circle or arc costs 256, each line 1)`,
			decaderr.ErrUnsupported, op, segments, MaxArrangementSegments)
	}
	return nil
}

func groupSceneProfiles(regions []momentinput.Profile) []SceneProfile {
	out := make([]SceneProfile, len(regions))
	for i, region := range regions {
		out[i] = SceneProfile{Outer: region.Outer, Holes: region.Holes}
	}
	return out
}

// BuildGroupScene arranges two groups, splits line runs at shared vertices,
// and charges cuts that can move with either operand's displacement. It
// returns ok=false for a topology this analytic path cannot resolve.
func BuildGroupScene(ctx context.Context, budget *proofbound.WorkBudget,
	regionsA, regionsB []momentinput.Profile, placementA, placementB prismplacement.Operand,
	sectionA, sectionB float64, op string) (GroupScene, bool, error) {
	if err := groupWithinCap(budget, op, regionsA, regionsB); err != nil {
		return GroupScene{}, false, err
	}
	reexpress, err := NewReexpression(placementA, placementB)
	if err != nil {
		return GroupScene{}, false, err
	}
	s, tags, charge, err := BuildSceneRegions(budget, groupSceneProfiles(regionsA), groupSceneProfiles(regionsB), reexpress)
	if err != nil {
		return GroupScene{}, false, err
	}
	if err := budget.Err(); err != nil {
		return GroupScene{}, false, err
	}
	profiles, err := ProfilesContext(ctx, s.Profiles)
	if err != nil {
		return GroupScene{}, false, err
	}
	profiles, resolved, err := SplitRuns(budget, profiles)
	if err != nil || !resolved {
		return GroupScene{}, false, err
	}
	if len(profiles) == 0 {
		return GroupScene{}, false, nil
	}
	delta := SceneDelta{A: charge.A, B: charge.B}
	if ok, err := delta.ChargeCrossings(budget, tags, profiles, sectionA, sectionB, reexpress.Delta); err != nil || !ok {
		return GroupScene{}, false, err
	}
	return GroupScene{Sketch: s, Tags: tags, Profiles: profiles,
		Delta: delta, ReexpressionDelta: reexpress.Delta}, true, nil
}

// GroupCutMatch finds the one cell whose whole outer reproduces the target
// and whose holes reproduce its holes plus every tool region's whole outer.
func GroupCutMatch(budget *proofbound.WorkBudget, profiles []*sketch.Profile,
	tags map[sketch.Entity]Origin, targetHoles, toolRegions int) (*sketch.Profile, bool, error) {
	targetOuter, err := RegionLoopEntitySet(budget, tags, false, 0, -1)
	if err != nil {
		return nil, false, err
	}
	wantHoles := make([]map[sketch.Entity]struct{}, 0, targetHoles+toolRegions)
	for i := range targetHoles {
		hole, err := RegionLoopEntitySet(budget, tags, false, 0, i)
		if err != nil {
			return nil, false, err
		}
		wantHoles = append(wantHoles, hole)
	}
	for r := range toolRegions {
		outer, err := RegionLoopEntitySet(budget, tags, true, r, -1)
		if err != nil {
			return nil, false, err
		}
		wantHoles = append(wantHoles, outer)
	}
	match, resolved, err := FindLoopMatch(budget, profiles, targetOuter, wantHoles)
	if err != nil || !resolved {
		return nil, false, err
	}
	if !match.Valid {
		return nil, false, InvalidRegionError("cut")
	}
	return match, true, nil
}

// GroupCutCells keeps the target's material outside every tool region. A
// displaced shared span must bound a selected cell or the answer is unresolved.
func GroupCutCells(budget *proofbound.WorkBudget, scene GroupScene) ([]*sketch.Profile, bool, error) {
	matterA, matterB, resolved, err := Classify(budget, scene.Tags, scene.Profiles)
	if err != nil || !resolved {
		return nil, false, err
	}
	selected, err := Select(budget, scene.Profiles, matterA, matterB, func(a, b bool) bool { return a && !b })
	if err != nil {
		return nil, false, err
	}
	if ok, err := scene.Delta.SharedSpansBounded(budget, selected); err != nil || !ok {
		return nil, false, err
	}
	return selected, true, nil
}

// GroupUnionLoops keeps every bounded cell after rejecting a void or an
// displaced shared span outside the selected boundary, then chains the
// surviving boundary into closed loops.
func GroupUnionLoops(budget *proofbound.WorkBudget, scene GroupScene) ([]sectionrecord.LoopRecord, float64, bool, error) {
	voidFree, err := CellsHaveNoVoid(budget, scene.Tags, scene.Profiles)
	if err != nil || !voidFree {
		return nil, 0, false, err
	}
	if ok, err := scene.Delta.SharedSpansBounded(budget, scene.Profiles); err != nil || !ok {
		return nil, 0, false, err
	}
	return MergeLoops(budget, scene.Profiles, "union")
}

// ProveGroupDisjoint accepts only one valid, whole outer cell per recorded
// region. Holes stay inside their own outer by the input's validity gate.
func ProveGroupDisjoint(ctx context.Context, budget *proofbound.WorkBudget,
	regions []momentinput.Profile) (bool, float64, error) {
	outers := make([]momentinput.Profile, len(regions))
	for i, region := range regions {
		outers[i] = momentinput.Profile{Outer: region.Outer}
	}
	if err := groupWithinCap(budget, "disjointness", outers); err != nil {
		return false, 0, err
	}
	s, tags, charge, err := BuildSceneRegions(budget, groupSceneProfiles(outers), nil, &Reexpression{Identity: true})
	if err != nil {
		return false, 0, err
	}
	profiles, err := ProfilesContext(ctx, s.Profiles)
	if err != nil {
		return false, 0, err
	}
	if len(profiles) != len(regions) {
		return false, 0, nil
	}
	claimed := make([]bool, len(profiles))
	for r := range regions {
		want, err := RegionLoopEntitySet(budget, tags, false, r, -1)
		if err != nil {
			return false, 0, err
		}
		match, resolved, err := FindLoopMatch(budget, profiles, want, nil)
		if err != nil {
			return false, 0, err
		}
		if !resolved || !match.Valid {
			return false, 0, nil
		}
		for i, p := range profiles {
			if p != match {
				continue
			}
			if claimed[i] {
				return false, 0, nil
			}
			claimed[i] = true
		}
	}
	return true, charge.A, nil
}
