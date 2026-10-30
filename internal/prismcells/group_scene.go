package prismcells

import (
	"context"
	"fmt"
	"math"

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
// A curve of one region touching the target or another region misses, as in
// MatchCut.
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
	if touch, err := RegionContact(budget, tags); err != nil || touch {
		return nil, false, err
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
// The arrangement does not cut a line at another line's endpoint lying on it
// either, so RegionContact also puts every two lines of two regions to
// sketch's line query. Any point either query reports touches. A curve kind
// the query does not cover refuses. The check only refuses: a reported point
// never admits anything.
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
	touch, err := RegionContact(budget, tags)
	if err != nil || touch {
		return false, charge, err
	}
	return true, charge, nil
}

// curvedContact reports whether sketch's intersection query finds any point
// shared by two entities of different regions where at least one is curved.
// Two lines are RegionContact's line reading.
// A line's reported point counts when its parameter along the segment lies
// within 1e-9 of [0, 1], a fraction of the segment's length, not the
// distance band the query itself uses; a wider reading only refuses more.
func curvedContact(budget *proofbound.WorkBudget, tags map[sketch.Entity]Origin) (bool, error) {
	entities := make([]sketch.Entity, 0, len(tags))
	for e := range tags {
		entities = append(entities, e)
	}
	for i, a := range entities {
		_, aLine := a.(*sketch.Line)
		for _, b := range entities[i+1:] {
			if _, bLine := b.(*sketch.Line); aLine && bLine {
				continue // curvedPairContact reports no contact between lines
			}
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

// CurvedCrossingsCut reports whether every point at which a curved entity of
// one operand meets an entity of the other is a transverse crossing the
// arrangement cut. Two operands whose boundaries cross elsewhere can still
// touch: sketch's arrangement neither cuts nor flags a clean tangency, so a
// circle tangent to the other operand's line or circle leaves no vertex for
// a caller's cell reading to see. Each such pair is put to sketch's own
// intersection query, as curvedContact puts it:
//
//   - two curved entities of different operands refuse on any point the
//     query reports, so a crossing of two circles refuses as well as a
//     tangency;
//   - a line and a curve refuse when the query reads one point on the line's
//     carrier (its tangent reading) lying on both entities;
//   - every other point the query reads on both entities must be a scene
//     vertex where an edge of each entity ends, within crossingMatchBand of
//     the reported point relative to the coordinates and the radius. A
//     crossing the arrangement read as a tangency, or did not place, has no
//     such vertex and refuses.
//
// A curve kind the query does not cover refuses. Two lines are the caller's
// to read. The check only refuses: a matched vertex never admits anything, it
// only leaves the arrangement's own cut standing for the caller's checks.
func CurvedCrossingsCut(budget *proofbound.WorkBudget, tags map[sketch.Entity]Origin,
	profiles []*sketch.Profile) (bool, error) {
	at := map[[2]float64]map[sketch.Entity]struct{}{}
	for _, cell := range profiles {
		for _, ring := range append([][]sketch.BoundaryEdge{cell.Outer}, cell.Holes...) {
			for _, e := range ring {
				if err := budget.Step(); err != nil {
					return false, err
				}
				if len(e.Polyline) == 0 {
					return false, nil
				}
				for _, p := range [][2]float64{e.Polyline[0], e.Polyline[len(e.Polyline)-1]} {
					if at[p] == nil {
						at[p] = map[sketch.Entity]struct{}{}
					}
					at[p][e.Entity] = struct{}{}
				}
			}
		}
	}
	entities := make([]sketch.Entity, 0, len(tags))
	for e := range tags {
		entities = append(entities, e)
	}
	for i, a := range entities {
		for _, b := range entities[i+1:] {
			if err := budget.Step(); err != nil {
				return false, err
			}
			if tags[a].IsB == tags[b].IsB {
				continue
			}
			line, aLine := a.(*sketch.Line)
			bl, bLine := b.(*sketch.Line)
			switch {
			case aLine && bLine:
				continue
			case !aLine && !bLine:
				if touch, covered := curvedPairContact(a, b); !covered || touch {
					return false, nil
				}
				continue
			}
			curve := b
			if bLine {
				line, curve = bl, a
			}
			points, tangent, covered := lineCurvePoints(line, curve)
			if !covered || tangent {
				return false, nil
			}
			for _, q := range points {
				cut, err := crossingVertex(budget, at, line, curve, q)
				if err != nil || !cut {
					return false, err
				}
			}
		}
	}
	return true, nil
}

// crossingMatchBand is CurvedCrossingsCut's relative distance between a point
// sketch's query reports and the arranged vertex standing for it. A narrower
// band only refuses more.
const crossingMatchBand = 1e-9

// lineCurvePoints is sketch's reading of where a line meets a circle or an
// arc: the points on both entities, and whether the query read the carriers
// as tangent (one point) at a point on both. covered is false for a curve
// kind the query does not cover.
func lineCurvePoints(line *sketch.Line, curve sketch.Entity) ([]*geom.Point, bool, bool) {
	l := line.Geometry()
	var circle *geom.Circle
	var arc *geom.Arc
	switch c := curve.(type) {
	case *sketch.Circle:
		circle = c.Geometry()
	case *sketch.Arc:
		arc = c.Geometry()
		circle = &geom.Circle{Center: arc.Center, Radius: arc.Radius()}
	default:
		return nil, false, false
	}
	raw := geom.LineCircleIntersections(l, circle)
	var on []*geom.Point
	for _, p := range raw {
		if !withinSegment(l, p) || (arc != nil && !arc.Contains(p)) {
			continue
		}
		on = append(on, p)
	}
	return on, len(raw) == 1 && len(on) == 1, true
}

// crossingVertex reports whether some arranged vertex where an edge of line
// and an edge of curve both end lies within crossingMatchBand of q.
func crossingVertex(budget *proofbound.WorkBudget, at map[[2]float64]map[sketch.Entity]struct{},
	line *sketch.Line, curve sketch.Entity, q *geom.Point) (bool, error) {
	scale := max(1, math.Abs(q.X), math.Abs(q.Y))
	switch c := curve.(type) {
	case *sketch.Circle:
		scale = max(scale, c.Geometry().Radius)
	case *sketch.Arc:
		scale = max(scale, c.Geometry().Radius())
	}
	for p, incident := range at {
		if err := budget.Step(); err != nil {
			return false, err
		}
		if _, ok := incident[line]; !ok {
			continue
		}
		if _, ok := incident[curve]; !ok {
			continue
		}
		if math.Hypot(p[0]-q.X, p[1]-q.Y) <= crossingMatchBand*scale {
			return true, nil
		}
	}
	return false, nil
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

// RegionContact reports whether two entities of different regions meet at a
// point sketch's arrangement leaves uncut. Two entities belong to different
// regions when their operand side or region index differs. It asks
// curvedContact for every pair with a curved member. The arrangement cuts two
// lines that cross, but not a line at another line's endpoint lying on it,
// such as a triangle whose apex touches a square's side. So every pair of
// lines is also put to sketch's own query: geom.LineLineIntersection gives
// the carriers' common point, and geom.ClosestPointOnLine reads its parameter
// along each segment. A point within lineContactBand of [0, 1] on both
// segments touches. Parallel carriers report no point. Collinear segments
// share a carrier, which the arrangement cuts where they overlap or meet. The
// answer only refuses: a reported point never admits anything.
func RegionContact(budget *proofbound.WorkBudget, tags map[sketch.Entity]Origin) (bool, error) {
	touch, err := curvedContact(budget, tags)
	if err != nil || touch {
		return touch, err
	}
	var lines []*sketch.Line
	for e := range tags {
		if l, ok := e.(*sketch.Line); ok {
			lines = append(lines, l)
		}
	}
	for i, a := range lines {
		for _, b := range lines[i+1:] {
			if err := budget.Step(); err != nil {
				return false, err
			}
			if tags[a].IsB == tags[b].IsB && tags[a].Region == tags[b].Region {
				continue
			}
			la, lb := a.Geometry(), b.Geometry()
			p, ok := geom.LineLineIntersection(la, lb)
			if !ok {
				continue
			}
			if withinSegment(la, p) && withinSegment(lb, p) {
				return true, nil
			}
		}
	}
	return false, nil
}

// RingContact reports whether the rings of one arranged cell meet: its outer
// and a hole, two holes, or one ring passing a point twice. Any such point
// pinches the face there. It reads three ways, each only refusing:
//
//   - a walked vertex (an edge's Polyline start, as ChainClosedLoops keys it)
//     that starts two of the cell's edges: two tangent boss circles the
//     arrangement cut at their tangency come back as one hole ring through
//     that point twice;
//   - one entity walked by two rings;
//   - RegionContact over the cell's entities, each tagged with its ring: the
//     arrangement cuts neither a tangency nor a line at another line's
//     endpoint lying on it, so a boss circle tangent inside a plate's outline
//     comes back as a hole touching the outer at a point the cell never names.
func RingContact(budget *proofbound.WorkBudget, cell *sketch.Profile) (bool, error) {
	tags := map[sketch.Entity]Origin{}
	starts := map[[2]float64]struct{}{}
	for r, ring := range append([][]sketch.BoundaryEdge{cell.Outer}, cell.Holes...) {
		for _, e := range ring {
			if err := budget.Step(); err != nil {
				return false, err
			}
			if len(e.Polyline) == 0 {
				return true, nil
			}
			if _, twice := starts[e.Polyline[0]]; twice {
				return true, nil
			}
			starts[e.Polyline[0]] = struct{}{}
			if have, ok := tags[e.Entity]; ok && have.Region != r {
				return true, nil
			}
			tags[e.Entity] = Origin{Region: r}
		}
	}
	return RegionContact(budget, tags)
}

// lineContactBand is the parameter slack of RegionContact's line reading, a
// fraction of the segment's length as curvedPairContact's is. A wider reading
// only refuses more.
const lineContactBand = 1e-9

func withinSegment(l *geom.Line, p *geom.Point) bool {
	_, t := geom.ClosestPointOnLine(l, p)
	return t >= -lineContactBand && t <= 1+lineContactBand
}
