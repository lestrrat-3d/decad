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
	"github.com/lestrrat-3d/sketch/geom"
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
		return WithWorkLimit(fmt.Errorf(
			`%w: the analytic %s scene charges at least %d arranger segments against this evaluator's cap of %d (each circle or arc costs 256, each line 1)`,
			decaderr.ErrUnsupported, op, segments, MaxArrangementSegments), segments)
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
	_, _, separate, charge, err := proveGroupCells(ctx, budget, regions)
	return separate, charge, err
}

// ProveGroupApart is ProveGroupDisjoint with the regions' closures apart as
// well, since a body that meets itself at a point is not a manifold. Two whole
// cells that share a walked vertex touch there. A curve touching another
// region's outline at an interior point, such as two tangent circles, is no
// vertex: sketch's arrangement neither cuts nor flags a clean tangency, so
// every pair of entities from two regions with a curved member is also put to
// sketch's own intersection query (geom.LineCircleIntersections and its
// circle and arc kin), which reports a tangency within its band as one point.
// Any point it reports touches. A curve kind the query does not cover refuses.
// The check only refuses: a reported point never admits anything.
func ProveGroupApart(ctx context.Context, budget *proofbound.WorkBudget,
	regions []momentinput.Profile) (bool, float64, error) {
	cells, tags, separate, charge, err := proveGroupCells(ctx, budget, regions)
	if err != nil || !separate {
		return false, charge, err
	}
	owner := map[[2]float64]int{}
	for i, cell := range cells {
		for _, loop := range append([][]sketch.BoundaryEdge{cell.Outer}, cell.Holes...) {
			for _, e := range loop {
				if err := budget.Step(); err != nil {
					return false, 0, err
				}
				if len(e.Polyline) == 0 {
					return false, charge, nil
				}
				for _, p := range [][2]float64{e.Polyline[0], e.Polyline[len(e.Polyline)-1]} {
					if j, ok := owner[p]; ok && j != i {
						return false, charge, nil
					}
					owner[p] = i
				}
			}
		}
	}
	touch, err := curvedContact(budget, tags)
	if err != nil || touch {
		return false, charge, err
	}
	return true, charge, nil
}

// curvedContact reports whether sketch's intersection query finds any point
// shared by two entities of different regions where at least one is curved.
// A line's reported point counts when it lies on the segment, read with the
// same band the query uses.
func curvedContact(budget *proofbound.WorkBudget, tags map[sketch.Entity]Origin) (bool, error) {
	entities := make([]sketch.Entity, 0, len(tags))
	for e := range tags {
		entities = append(entities, e)
	}
	for i, a := range entities {
		for _, b := range entities[i+1:] {
			if err := budget.Step(); err != nil {
				return false, err
			}
			if tags[a].IsB == tags[b].IsB && tags[a].Region == tags[b].Region {
				continue
			}
			touch, covered := curvedPairContact(a, b)
			if !covered || touch {
				return true, nil
			}
		}
	}
	return false, nil
}

// curvedPairContact asks sketch's geom query whether a and b meet. Two lines
// are the arrangement's to split and report covered with no contact.
func curvedPairContact(a, b sketch.Entity) (bool, bool) {
	if _, ok := a.(*sketch.Line); !ok {
		if _, ok := b.(*sketch.Line); ok {
			a, b = b, a
		}
	}
	onSegment := func(l *geom.Line, pts []*geom.Point) bool {
		const band = 1e-9
		for _, p := range pts {
			if _, t := geom.ClosestPointOnLine(l, p); t >= -band && t <= 1+band {
				return true
			}
		}
		return false
	}
	switch a := a.(type) {
	case *sketch.Line:
		switch b := b.(type) {
		case *sketch.Line:
			return false, true
		case *sketch.Circle:
			l := a.Geometry()
			return onSegment(l, geom.LineCircleIntersections(l, b.Geometry())), true
		case *sketch.Arc:
			l := a.Geometry()
			return onSegment(l, geom.LineArcIntersections(l, b.Geometry())), true
		}
	case *sketch.Circle:
		switch b := b.(type) {
		case *sketch.Circle:
			return len(geom.CircleCircleIntersections(a.Geometry(), b.Geometry())) != 0, true
		case *sketch.Arc:
			return len(geom.CircleArcIntersections(a.Geometry(), b.Geometry())) != 0, true
		}
	case *sketch.Arc:
		switch b := b.(type) {
		case *sketch.Circle:
			return len(geom.CircleArcIntersections(b.Geometry(), a.Geometry())) != 0, true
		case *sketch.Arc:
			return len(geom.ArcArcIntersections(a.Geometry(), b.Geometry())) != 0, true
		}
	}
	return false, false
}

// proveGroupCells returns the one matched cell per region, and the scene's
// tags, when every region matched a distinct valid, whole cell.
func proveGroupCells(ctx context.Context, budget *proofbound.WorkBudget,
	regions []momentinput.Profile) ([]*sketch.Profile, map[sketch.Entity]Origin, bool, float64, error) {
	outers := make([]momentinput.Profile, len(regions))
	for i, region := range regions {
		outers[i] = momentinput.Profile{Outer: region.Outer}
	}
	if err := groupWithinCap(budget, "disjointness", outers); err != nil {
		return nil, nil, false, 0, err
	}
	s, tags, charge, err := BuildSceneRegions(budget, groupSceneProfiles(outers), nil, &Reexpression{Identity: true})
	if err != nil {
		return nil, nil, false, 0, err
	}
	profiles, err := ProfilesContext(ctx, s.Profiles)
	if err != nil {
		return nil, nil, false, 0, err
	}
	if len(profiles) != len(regions) {
		return nil, nil, false, 0, nil
	}
	claimed := make([]bool, len(profiles))
	for r := range regions {
		want, err := RegionLoopEntitySet(budget, tags, false, r, -1)
		if err != nil {
			return nil, nil, false, 0, err
		}
		match, resolved, err := FindLoopMatch(budget, profiles, want, nil)
		if err != nil {
			return nil, nil, false, 0, err
		}
		if !resolved || !match.Valid {
			return nil, nil, false, 0, nil
		}
		for i, p := range profiles {
			if p != match {
				continue
			}
			if claimed[i] {
				return nil, nil, false, 0, nil
			}
			claimed[i] = true
		}
	}
	return profiles, tags, true, charge.A, nil
}
