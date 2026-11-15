package filletband

import (
	"github.com/lestrrat-3d/decad/internal/capband"
	"github.com/lestrrat-3d/decad/internal/capcontour"
	"github.com/lestrrat-3d/decad/internal/offset2d"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/survey2d"
)

// ReadLoop classifies the recorded loop for a fillet of radius r with
// conversion bound rDelta and builds its patch sequence. A nonnegative
// badCorner names a join that disagrees with Table LF, and badWhy the
// near-tangent, restatement or curved-miter rule's reason when one refused it
// ("" otherwise); the caller can attach its face role and recorded segment
// indices to that refusal.
//
// Every corner is read about the restated centres of its off-radius arcs
// (CornerClass, §5.6). Each off-radius arc must be its walk's only segment
// and capband.OffRadiusShift must bound it at the fillet's reach r + rDelta;
// the loop's capband.OffRadiusLoop terms are kept for the strip (§5.6). Every
// wall with an LF5′ end must pass capband.NearTangentWallRefusal with
// restated centres, the survival gate §5.5's corner terms assume: its true
// edge runs from its start corner to its end corner at every level. A wall
// whose other end is LF8 or LF6 refuses, since that gate does not enclose such
// an end. A wall that fails names the LF5′ corner at its start, or at its end
// when only that one is LF5′.
func ReadLoop(budget *proofbound.WorkBudget, record sectionrecord.LoopRecord,
	walks []survey2d.SideWalk, radius, radiusDelta, tol float64) (loop Loop, joins []offset2d.Join,
	pieces []Piece, badCorner int, badWhy string, err error) {
	for _, w := range walks {
		loop.Walks = append(loop.Walks, Walk{
			Circular: w.IsCircular(), Closed: w.Closed, CCW: w.Th1 > w.Th0,
			Start:  Point{U: w.StartU, V: w.StartV},
			End:    Point{U: w.EndU, V: w.EndV},
			Center: Point{U: w.CU, V: w.CV}, Radius: w.Radius,
		})
	}
	if !loop.WholeTurn() {
		joins, err = OffsetJoins(budget, walks, radius, 0, tol)
		if err != nil {
			return Loop{}, nil, nil, -1, "", err
		}
		n := len(walks)
		rho, rhoRec, gap := make([]float64, n), make([]float64, n), make([]float64, n)
		near, step := false, false
		for k := range walks {
			if err = survey2d.WallBudgetStep(budget); err != nil {
				return Loop{}, nil, nil, -1, "", err
			}
			read, err := CornerClass(record, walks, loop.Walks, joins, k, radius, radiusDelta)
			if err != nil {
				return Loop{}, nil, nil, -1, "", err
			}
			if read.Class == 0 {
				return Loop{}, nil, nil, k, read.Why, nil
			}
			loop.Corners = append(loop.Corners, read.Class)
			rho[k], rhoRec[k] = read.Rho, read.RhoRec
			near = near || read.Class == NearTangent || read.RhoRec > 0
			g, ok, err := cornerGap(record, walks, k)
			if err != nil {
				return Loop{}, nil, nil, -1, "", err
			}
			if !ok {
				return Loop{}, nil, nil, k, capband.RestateRefusal, nil
			}
			gap[k] = g
			step = step || g > 0
		}
		reach := proofbound.AbsSumUpper(radius, radiusDelta)
		restate, k, why, err := restateLoop(record, walks, loop.Corners, reach)
		if err != nil {
			return Loop{}, nil, nil, -1, "", err
		}
		if why != "" {
			return Loop{}, nil, nil, k, why, nil
		}
		loop.Restate = restate
		if near || step || restate.Arcs > 0 {
			loop.Rho, loop.RhoRec, loop.Gap = rho, rhoRec, gap
		}
		if near {
			k, why, err := nearTangentWalls(budget, record, walks, loop.Corners, rho, rhoRec, radius, radiusDelta)
			if err != nil {
				return Loop{}, nil, nil, -1, "", err
			}
			if why != "" {
				return Loop{}, nil, nil, k, why, nil
			}
		}
	}
	pieces, err = loop.Pieces()
	if err != nil {
		return Loop{}, nil, nil, -1, "", err
	}
	return loop, joins, pieces, -1, "", nil
}

// cornerGap is the radial step the records of corner k's two walks hold at
// its recorded junction, the sum of junctionGap over the segment ending the
// arriving walk and the one starting the leaving walk. ok is false where a
// number does not lift.
func cornerGap(record sectionrecord.LoopRecord, walks []survey2d.SideWalk, k int) (float64, bool, error) {
	n := len(walks)
	prevSeg, curSeg, err := cornerSegments(record, walks[(k+n-1)%n], walks[k])
	if err != nil {
		return 0, false, err
	}
	v, err := ratPoint(Point{U: walks[k].StartU, V: walks[k].StartV})
	if err != nil {
		return 0, false, err
	}
	a, okA := junctionGap(prevSeg, v)
	b, okB := junctionGap(curSeg, v)
	if !okA || !okB {
		return 0, false, nil
	}
	return proofbound.AbsSumUpper(a, b), true, nil
}

// restateLoop reads the loop's restatement terms at reach (§5.6). Every
// off-radius arc must be its walk's only recorded segment, and
// capband.OffRadiusLoop must bound every one; otherwise it names the corner
// at that walk's start with the restatement refusal.
func restateLoop(record sectionrecord.LoopRecord, walks []survey2d.SideWalk, corners []Corner, reach float64) (RestateTerms, int, string, error) {
	n := len(walks)
	restated := make([]bool, n)
	found := false
	for i, w := range walks {
		if !w.IsCircular() || len(w.Segs) == 0 {
			continue
		}
		for _, si := range w.Segs {
			seg, err := sectionrecord.NormalizeSegment(record.Segments[si])
			if err != nil {
				return RestateTerms{}, -1, "", err
			}
			if !segmentOffRadius(seg) {
				continue
			}
			if len(w.Segs) != 1 {
				return RestateTerms{}, i, capband.RestateRefusal, nil
			}
			arc, isArc := seg.(sectionrecord.ArcSeg)
			if _, ok := capband.OffRadiusShift(arc, reach); !isArc || !ok {
				return RestateTerms{}, i, capband.RestateRefusal, nil
			}
			restated[i], found = true, true
		}
	}
	if !found {
		return RestateTerms{}, -1, "", nil
	}
	terms, ok := capband.OffRadiusLoop(record, reach)
	if !ok {
		return RestateTerms{}, 0, capband.RestateRefusal, nil
	}
	out := RestateTerms{OffRadiusTerms: terms, Walks: restated}
	for k, c := range corners {
		if c == Reflex && (restated[k] || restated[(k+n-1)%n]) {
			out.ReflexBeside++
		}
	}
	return out, -1, "", nil
}

// nearTangentWalls runs capband.NearTangentWallRefusal on every wall with a
// near-tangent end, every walk moving by radius over radiusDelta: once with
// restated centres and the ρ′ reaches rho, and once with the recorded centres
// and the ρ_rec reaches rhoRec, wherever either is positive at an end
// (agent-docs/loop-fillet-design.md §5.5). It returns the near-tangent corner
// a failing wall names and the refusal's reason, or "" when every such wall
// survives; err is a budget or context error.
func nearTangentWalls(budget *proofbound.WorkBudget, record sectionrecord.LoopRecord, walks []survey2d.SideWalk,
	corners []Corner, rho, rhoRec []float64, radius, radiusDelta float64) (int, string, error) {
	n := len(walks)
	amounts := make([]float64, n)
	for i := range amounts {
		amounts[i] = radius
	}
	near := func(k int) bool { return rho[k] > 0 || rhoRec[k] > 0 }
	for i := range walks {
		if err := survey2d.WallBudgetStep(budget); err != nil {
			return -1, "", err
		}
		next := (i + 1) % n
		if !near(i) && !near(next) {
			continue
		}
		end := i
		if !near(i) {
			end = next
		}
		// The gate reads a far end as the carriers' miter or a circular
		// neighbour's radial foot, which an LF8 or LF6 end is not, so such a
		// wall is refused rather than gated on the wrong end.
		for _, k := range [2]int{i, next} {
			if corners[k] == CurvedMiter || corners[k] == Reflex {
				return end, nearTangentOtherEndWhy, nil
			}
		}
		for _, reading := range []struct {
			reach   []float64
			restate bool
		}{{rho, true}, {rhoRec, false}} {
			if reading.reach[i] == 0 && reading.reach[next] == 0 {
				continue
			}
			if why := capband.NearTangentWallRefusal(record, walks, i, amounts, radiusDelta, reading.reach[i], reading.reach[next], reading.restate); why != "" {
				return end, why, nil
			}
		}
	}
	return -1, "", nil
}

// nearTangentOtherEndWhy refuses a wall with an LF5′ end whose other end is an
// LF8 or LF6 corner, which the near-tangent survival gate does not enclose.
const nearTangentOtherEndWhy = `a wall at a near-tangent join whose other end is a curved miter or a reflex corner, which the near-tangent survival gate does not enclose`

// ContourDelta bounds the contour after sphere poles consume their inward
// arcs. The surviving carriers meet at each recorded pole centre.
func ContourDelta(budget *proofbound.WorkBudget, walks []survey2d.SideWalk,
	radius, radiusDelta, tol float64) (float64, error) {
	joins, err := OffsetJoins(budget, walks, radius, radiusDelta, tol)
	if err != nil {
		return 0, err
	}
	var kept []survey2d.SideWalk
	var keptJoins []capcontour.Join
	n := len(walks)
	for i, w := range walks {
		if SphereWalk(w, radius) {
			continue
		}
		j := joins[i]
		prev := walks[(i+n-1)%n]
		if SphereWalk(prev, radius) {
			j = offset2d.Join{M: sectionrecord.Point2{U: prev.CU, V: prev.CV}, VertU: prev.CU, VertV: prev.CV}
		}
		kept = append(kept, w)
		keptJoins = append(keptJoins, capcontour.Join{
			Arc: j.Arc, G1: j.G1, VU: j.VertU, VV: j.VertV,
			M: j.M, PA: j.PA, PB: j.PB,
		})
	}
	if len(kept) < 2 {
		return 0, offset2d.ErrDrop
	}
	return capband.ContourDisplacement(kept, keptJoins, radius, radiusDelta, tol)
}
