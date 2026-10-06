package clearance

import (
	"math"

	"github.com/lestrrat-3d/decad/internal/survey2d"

	"github.com/lestrrat-3d/decad/internal/proofbound"

	"github.com/lestrrat-3d/r3"
)

// GapContrib is one contribution to a pair's distance interval: a proven
// [lo, hi], with hi = +Inf for a lower-bound-only (straddle-admitted or
// unresolvable) contribution.
type GapContrib struct {
	Lo, Hi float64
	Exact  bool
}

// SpineCrit is one spine-pair critical: a proven value interval and the
// spine feet it is attained at.
type SpineCrit struct {
	Lo, Hi float64
	Exact  bool
	Fa, Fb r3.Vec // feet on f's and g's spines
}

// SpineOffset is a face's spine offset radius.
func SpineOffset(f *CFace) float64 { return f.Radius }

// SpineOf reports the spine kind: 0 point, 1 line, 2 circle.
func SpineOf(f *CFace) int {
	switch f.Kind {
	case CkSphere:
		return 0
	case CkCylinder:
		return 1
	default:
		return 2
	}
}

func ExactCrit(a, b r3.Vec) SpineCrit {
	d := a.Sub(b).Len()
	return SpineCrit{Lo: d, Hi: d, Exact: true, Fa: a, Fb: b}
}

// LinePoint is the foot of p on the line (a, unit d).
func LinePoint(a, d, p r3.Vec) r3.Vec {
	return a.Add(d.Scale(p.Sub(a).Dot(d)))
}

// RingAngles proposes representative azimuths of the ring family: a
// deterministic uniform set plus each face's own window endpoints and mid
// mapped into the ring frame — the places a partial trim's admission can begin,
// end or sit comfortably inside.
func RingAngles(f, g *CFace, u, v r3.Vec) []float64 {
	const uniform = 16
	out := make([]float64, 0, uniform+6)
	for i := range uniform {
		out = append(out, 2*math.Pi*float64(i)/uniform)
	}
	for _, fc := range []*CFace{f, g} {
		if fc.Sweep.Full {
			continue
		}
		for _, th := range []float64{fc.Sweep.Lo, (fc.Sweep.Lo + fc.Sweep.Hi) / 2, fc.Sweep.Hi} {
			dir := fc.RefU.Scale(math.Cos(th)).Add(fc.RefV.Scale(math.Sin(th)))
			out = append(out, math.Atan2(dir.Dot(v), dir.Dot(u)))
		}
	}
	return out
}

// PlanesIntersect returns a point on the two planes' intersection line.
func PlanesIntersect(f, g *CFace) (r3.Vec, bool) {
	// Solve within the pencil: p = f.o + s·d with d in f's plane,
	// perpendicular to the intersection direction.
	dir := f.N.Cross(g.N)
	inPlane, ok := f.N.Cross(dir).Normalize()
	if !ok {
		return r3.Vec{}, false
	}
	den := inPlane.Dot(g.N)
	if math.Abs(den) < 1e-12 {
		return r3.Vec{}, false
	}
	s := g.O.Sub(f.O).Dot(g.N) / den
	return f.O.Add(inPlane.Scale(s)), true
}

// IntervalsMeet classifies two interval sets on a shared line: +1 provably
// overlapping, −1 provably apart, 0 ambiguous.
func IntervalsMeet(a, b []ClrIv, tol float64) int {
	if len(a) == 0 || len(b) == 0 {
		return -1
	}
	out := -1
	for _, ia := range a {
		for _, ib := range b {
			lo := math.Max(ia.Lo, ib.Lo)
			hi := math.Min(ia.Hi, ib.Hi)
			if hi-lo > tol {
				return 1
			}
			if hi-lo > -tol {
				out = 0
			}
		}
	}
	return out
}

func CoplanarBoundaryClearanceBudget(budget *proofbound.WorkBudget, a, b Region2) (float64, error) {
	clearing := math.Inf(1)
	for _, ea := range a.Elems {
		for _, eb := range b.Elems {
			if err := budget.Step(); err != nil {
				return 0, err
			}
			if d := ElemElemDistLB(ea, eb); d < clearing {
				clearing = d
			}
		}
	}
	return clearing, nil
}

// regionSampleOutside reports whether a sample of src is cleanly outside dst
// — with boundaries provably apart, one clean sample settles containment.
func RegionSampleOutsideBudget(budget *proofbound.WorkBudget, src, dst Region2, tol float64) (bool, error) {
	samples, err := RegionSamplesBudget(budget, src)
	if err != nil {
		return false, err
	}
	for _, s := range samples {
		if err := budget.Step(); err != nil {
			return false, err
		}
		class, err := RegionClassifyBudget(budget, dst, s[0], s[1], tol)
		if err != nil {
			return false, err
		}
		switch class {
		case -1:
			return true, nil
		case 1:
			return false, nil
		}
	}
	return false, nil
}

// TransformElem maps a boundary element from one plane face's 2D frame into
// another parallel face's frame (projection along the shared normal — a
// planar rigid motion, possibly reflected).
func TransformElem(e survey2d.SurveyElem, from, to *CFace) survey2d.SurveyElem {
	mapPt := func(x, y float64) (float64, float64) {
		w := from.O.Add(from.U.Scale(x)).Add(from.V.Scale(y))
		return to.PlaneCoords(w)
	}
	if e.Kind == survey2d.SurveyLine {
		ax, ay := mapPt(e.Ax, e.Ay)
		bx, by := mapPt(e.Bx, e.By)
		out, _ := survey2d.LineElem(ax, ay, bx, by)
		return out
	}
	cx, cy := mapPt(e.Qx, e.Qy)
	delta := math.Atan2(from.U.Dot(to.V), from.U.Dot(to.U))
	det := from.U.Cross(from.V).Dot(to.U.Cross(to.V))
	var th0, th1 float64
	if det >= 0 {
		th0, th1 = e.Th0+delta, e.Th1+delta
	} else {
		th0, th1 = delta-e.Th1, delta-e.Th0
	}
	out, _ := survey2d.ArcElem(cx, cy, e.Rr, math.Min(th0, th1), math.Max(th0, th1), e.Closed)
	// A rigid motion moves no radius, so the mapped element keeps the source
	// element's radius under the source's own bound on it.
	out.RrBound = e.RrBound
	return out
}

// ElemElemDistLB is a lower bound on the distance between two 2D boundary
// elements (arcs bounded through their full circles — an underestimate, the
// sound direction for exclusion proofs).
func ElemElemDistLB(a, b survey2d.SurveyElem) float64 {
	if a.Kind == survey2d.SurveyLine && b.Kind == survey2d.SurveyLine {
		return SegSegDist(a.Ax, a.Ay, a.Bx, a.By, b.Ax, b.Ay, b.Bx, b.By)
	}
	if a.Kind == survey2d.SurveyLine {
		return SegElemDistLB(b, a.Ax, a.Ay, a.Bx, a.By)
	}
	if b.Kind == survey2d.SurveyLine {
		return SegElemDistLB(a, b.Ax, b.Ay, b.Bx, b.By)
	}
	d := math.Hypot(b.Qx-a.Qx, b.Qy-a.Qy)
	if d >= a.Rr+b.Rr {
		return d - a.Rr - b.Rr
	}
	if d <= math.Abs(a.Rr-b.Rr) {
		return math.Abs(a.Rr-b.Rr) - d
	}
	return 0
}

// CircleRegionHits classifies a full 2D circle against a region: +1 provably
// meeting it, −1 provably clear of it, 0 ambiguous.
func CircleRegionHits(r Region2, cx, cy, rad float64) int {
	tol := r.Tol()
	for i := range 8 {
		th := float64(i) * math.Pi / 4
		if r.Classify(cx+rad*math.Cos(th), cy+rad*math.Sin(th), tol) == 1 {
			return 1
		}
	}
	clearing := math.Inf(1)
	probe := survey2d.SurveyElem{Kind: survey2d.SurveyArc, Qx: cx, Qy: cy, Rr: rad, Th0: 0, Th1: 2 * math.Pi, Closed: true}
	for _, e := range r.Elems {
		if d := ElemElemDistLB(e, probe); d < clearing {
			clearing = d
		}
	}
	if clearing > tol && r.Classify(cx+rad, cy, tol) == -1 {
		return -1
	}
	return 0
}

// TrimmedCircleCrossingRelation classifies a transversal carrier-crossing
// circle through both face trims. A full-circle miss on the plane proves
// exclusion. Overlap requires one deterministic point cleanly admitted by the
// plane region and the revolved face's own sweep/meridian/axial trims; missing
// such a point is undecided, never a false admission from the full carrier.
func TrimmedCircleCrossingRelation(f, g *CFace, center r3.Vec, rad, tol float64) int {
	cx, cy := f.PlaneCoords(center)
	if CircleRegionHits(f.Region, cx, cy, rad) == -1 {
		return -1
	}
	const samples = 64
	for i := range samples {
		th := 2 * math.Pi * float64(i) / samples
		q := center.Add(f.U.Scale(rad * math.Cos(th))).Add(f.V.Scale(rad * math.Sin(th)))
		if AdmitState(f.AdmitPoint(q, tol), g.AdmitPoint(q, tol)) == 1 {
			return 1
		}
	}
	return 0
}
