package filletband

import (
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/capband"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/offset2d"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/survey2d"
)

// SphereWalk requires both pinned arc endpoints to lie exactly on the
// requested sphere. A rounded hypot equal to r is insufficient.
func SphereWalk(w survey2d.SideWalk, r float64) bool {
	if !w.IsCircular() || w.Closed || w.Th1 <= w.Th0 || w.Radius != r {
		return false
	}
	cu, cv, rr := proofarith.FloatRat(w.CU), proofarith.FloatRat(w.CV), proofarith.FloatRat(r)
	if cu == nil || cv == nil || rr == nil {
		return false
	}
	r2 := new(big.Rat).Mul(rr, rr)
	for _, p := range [2]sectionrecord.Point2{{U: w.StartU, V: w.StartV}, {U: w.EndU, V: w.EndV}} {
		u, v := proofarith.FloatRat(p.U), proofarith.FloatRat(p.V)
		if u == nil || v == nil {
			return false
		}
		du, dv := new(big.Rat).Sub(u, cu), new(big.Rat).Sub(v, cv)
		squared := new(big.Rat).Add(new(big.Rat).Mul(du, du), new(big.Rat).Mul(dv, dv))
		if squared.Cmp(r2) != 0 {
			return false
		}
	}
	return true
}

// OffsetJoins reads an inward circular walk whose radius equals the fillet
// radius as a pole. Its neighbours must be straight and exactly G1; the
// ordinary offset's computed feet are replaced by the recorded centre. Every
// other G1 or connector-arc foot is held exactly where the recorded data
// denotes a float64 point (exactFeet).
func OffsetJoins(budget *proofbound.WorkBudget, walks []survey2d.SideWalk, r, rDelta, tol float64) ([]offset2d.Join, error) {
	n := len(walks)
	if n == 0 {
		return nil, fmt.Errorf(`%w: a loop fillet holds no walks`, decaderr.ErrDegenerate)
	}
	collapsed := make([]bool, n)
	for i, w := range walks {
		if err := survey2d.WallBudgetStep(budget); err != nil {
			return nil, err
		}
		if !w.IsCircular() {
			continue
		}
		if SphereWalk(w, r) {
			if rDelta != 0 || w.Closed {
				return nil, fmt.Errorf(`%w: a circular fillet walk collapses over a radius span or a whole turn`, decaderr.ErrUnsupported)
			}
			collapsed[i] = true
			continue
		}
		if w.Th1 > w.Th0 && w.Radius == r {
			return nil, fmt.Errorf(`%w: a three-edge sphere needs the arc's recorded endpoints to lie exactly at the fillet radius`, decaderr.ErrUnsupported)
		}
		if w.Th1 > w.Th0 && w.Radius > r && w.Radius-r <= tol {
			return nil, fmt.Errorf(`%w: a three-edge sphere needs the circular wall's recorded radius to equal the fillet radius; the held radii are %.17g and %.17g mm`, decaderr.ErrUnsupported, w.Radius, r)
		}
		if _, err := capband.BandRadius(w, r, tol); err != nil {
			return nil, err
		}
	}
	joins, err := offset2d.SectionJoinsBudget(budget, walks, 1, r, tol)
	if err != nil {
		return nil, err
	}
	exactFeet(walks, joins, r)
	for i, on := range collapsed {
		if !on {
			continue
		}
		prev, next := (i+n-1)%n, (i+1)%n
		if !walks[prev].IsLine() || !walks[next].IsLine() || !joins[i].G1 || !joins[next].G1 {
			return nil, fmt.Errorf(`%w: a collapsed circular fillet walk needs two straight G1 neighbours (loop-fillet SF1)`, decaderr.ErrUnsupported)
		}
		pole := sectionrecord.Point2{U: walks[i].CU, V: walks[i].CV}
		joins[i].M, joins[next].M = pole, pole
	}
	return joins, nil
}

// exactFeet holds every join's feet exactly where the recorded data denotes
// a float64 point (offset2d.ExactCornerFeet, loop-fillet §4 step 1), at the
// held radius r alone: whatever radius span a caller reads, every caller of
// OffsetJoins holds the same contour vertices and LF5 and LF6 meridian
// centres. offset2d.PartialFilletContour applies the same rule to its
// selected pairs, so a partial fillet's contour and meridians stand on the
// same feet.
func exactFeet(walks []survey2d.SideWalk, joins []offset2d.Join, r float64) {
	n := len(walks)
	for i := range joins {
		joins[i] = offset2d.ExactCornerFeet(walks[(i+n-1)%n], walks[i], joins[i], r)
	}
}

// OffsetLoop records the cap contour. A collapsed circular walk leaves one
// vertex at its recorded centre and contributes no segment to the loop.
func OffsetLoop(budget *proofbound.WorkBudget, walks []survey2d.SideWalk, joins []offset2d.Join, r, tol float64) ([]sectionrecord.CurveSegment, error) {
	var segs []sectionrecord.CurveSegment
	n := len(walks)
	for i, w := range walks {
		if err := survey2d.WallBudgetStep(budget); err != nil {
			return nil, err
		}
		start, end := joins[i].M, joins[(i+1)%n].M
		if joins[i].Arc {
			start = joins[i].PB
		}
		if joins[(i+1)%n].Arc {
			end = joins[(i+1)%n].PA
		}
		if SphereWalk(w, r) {
			continue
		}
		if offset2d.WalkConsumed(w, start, end, tol) {
			return nil, offset2d.ErrDrop
		}
		seg, err := offset2d.WalkSegment(w, 1, r, start, end, tol)
		if err != nil {
			return nil, err
		}
		segs = append(segs, seg)
		j := joins[(i+1)%n]
		if j.Arc {
			segs = append(segs, offset2d.ArcSegment(sectionrecord.Point2{U: j.VertU, V: j.VertV}, j.PA, j.PB, false))
		}
	}
	return segs, nil
}

// CornerClass is Table LF's class of corner k for a fillet of radius r with
// conversion bound rDelta, beside the corner's near-tangent reach ρ (zero for
// every class but LF5′). It returns zero for SF1, with the reason
// capband.NearTangentCorner gives when the offset reads the join as G1 but
// that class refuses it, and "" otherwise.
//
// A join the offset reads as G1 is LF5 when capband.JoinIsG1 proves it exactly
// tangent, and LF5′ when capband.NearTangentCorner admits it with both walks
// moving by r over its conversion span: a line and an arc, or two arcs, at one
// recorded point whose exact tangents run the same way and turn by
// sin²θ <= 2⁻⁶⁴, each moved circle clear of the corner
// (agent-docs/loop-fillet-design.md §5.5). Both tests are exact rational
// arithmetic over the record's floats; neither admits on a residual.
func CornerClass(loop sectionrecord.LoopRecord, walks []survey2d.SideWalk, fw []Walk, joins []offset2d.Join, k int,
	r, rDelta float64) (Corner, float64, string, error) {
	n := len(walks)
	prev, cur := walks[(k+n-1)%n], walks[k]
	turn, err := Turn(fw[(k+n-1)%n], fw[k])
	if err != nil {
		return 0, 0, "", err
	}
	j := joins[k]
	switch {
	case turn < 0 && j.Arc:
		return Reflex, 0, "", nil
	case turn > 0 && prev.IsLine() && cur.IsLine() && !j.Arc && !j.G1:
		return Miter, 0, "", nil
	case turn > 0 && (prev.IsCircular() || cur.IsCircular()) && !j.Arc && !j.G1:
		return CurvedMiter, 0, "", nil
	case j.G1 && !j.Arc:
		prevSeg, curSeg, err := cornerSegments(loop, prev, cur)
		if err != nil {
			return 0, 0, "", err
		}
		if turn == 0 && capband.JoinIsG1(prevSeg, curSeg) {
			return Tangent, 0, "", nil
		}
		charge, why, hint := capband.NearTangentCorner(prevSeg, curSeg, r, r, rDelta)
		if why != "" {
			return 0, 0, why + hint, nil
		}
		return NearTangent, charge.Rho, "", nil
	}
	return 0, 0, "", nil
}

// cornerSegments is the normalized recorded segment that ends walk prev and
// the one that starts walk cur.
func cornerSegments(loop sectionrecord.LoopRecord, prev, cur survey2d.SideWalk) (sectionrecord.CurveSegment, sectionrecord.CurveSegment, error) {
	prevSeg, err := sectionrecord.NormalizeSegment(loop.Segments[prev.Segs[len(prev.Segs)-1]])
	if err != nil {
		return nil, nil, err
	}
	curSeg, err := sectionrecord.NormalizeSegment(loop.Segments[cur.Segs[0]])
	if err != nil {
		return nil, nil, err
	}
	return prevSeg, curSeg, nil
}
