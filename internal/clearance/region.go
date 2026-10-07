package clearance

import (
	"math"

	"github.com/lestrrat-3d/decad/internal/survey2d"

	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// Region2 is a planar trim region: line/arc boundary loops in the face's own
// 2D frame, queried by exact crossing parity with a deterministic retry
// ladder — the same discipline as the wall survey's containment test.
type Region2 struct {
	Elems []survey2d.SurveyElem
	Scale float64
}

func NewRegion2(elems []survey2d.SurveyElem) Region2 {
	scale := 1.0
	grow := func(vs ...float64) {
		for _, v := range vs {
			if a := math.Abs(v); a > scale {
				scale = a
			}
		}
	}
	for _, e := range elems {
		if e.Kind == survey2d.SurveyLine {
			grow(e.Ax, e.Ay, e.Bx, e.By)
			continue
		}
		grow(e.Qx-e.Rr, e.Qx+e.Rr, e.Qy-e.Rr, e.Qy+e.Rr)
	}
	return Region2{Elems: elems, Scale: scale}
}

func (r Region2) Tol() float64 { return 1e-9 * r.Scale }

// contains is exact parity membership; the second result is false when every
// ladder direction stays ambiguous.
func (r Region2) Contains(px, py float64) (bool, bool) {
	for i := range 16 {
		th := 0.5 + float64(i)*2.399963229728653
		dx, dy := math.Cos(th), math.Sin(th)
		crossings, ok := 0, true
		for _, e := range r.Elems {
			n, good := survey2d.RayCrossings(e, px, py, dx, dy, r.Tol())
			if !good {
				ok = false
				break
			}
			crossings += n
		}
		if ok {
			return crossings%2 == 1, true
		}
	}
	return false, false
}

// boundaryDist is the distance from p to the region's boundary.
func (r Region2) BoundaryDist(px, py float64) float64 {
	best := math.Inf(1)
	for _, e := range r.Elems {
		d, _, _ := e.Nearest(px, py, survey2d.SurvTiny*r.Scale)
		if d < best {
			best = d
		}
	}
	return best
}

// classify reports +1 strictly inside with more than margin of boundary
// clearance, −1 strictly outside with more than margin, 0 ambiguous or on
// the boundary.
func (r Region2) Classify(px, py, margin float64) int {
	if r.BoundaryDist(px, py) <= margin {
		return 0
	}
	in, ok := r.Contains(px, py)
	if !ok {
		return 0
	}
	if in {
		return 1
	}
	return -1
}

// samples returns boundary sample points (element endpoints and midpoints) —
// points that lie ON the closed region by construction.
func (r Region2) Samples() [][2]float64 {
	var out [][2]float64
	for _, e := range r.Elems {
		if e.Kind == survey2d.SurveyLine {
			out = append(out, [2]float64{e.Ax, e.Ay}, [2]float64{(e.Ax + e.Bx) / 2, (e.Ay + e.By) / 2})
			continue
		}
		lo, hi := e.ArcRange()
		for _, th := range []float64{lo, (lo + hi) / 2} {
			out = append(out, [2]float64{e.Qx + e.Rr*math.Cos(th), e.Qy + e.Rr*math.Sin(th)})
		}
	}
	return out
}

// interiorPoint returns a point proven strictly inside the region, probing
// element-midpoint inward offsets at a few depths.
func (r Region2) InteriorPoint() ([2]float64, bool) {
	probe := func(x, y float64) bool { return r.Classify(x, y, r.Tol()) == 1 }
	for _, e := range r.Elems {
		var px, py, nx, ny float64
		if e.Kind == survey2d.SurveyLine {
			px, py = (e.Ax+e.Bx)/2, (e.Ay+e.By)/2
			nx, ny = e.Nx, e.Ny
		} else {
			lo, hi := e.ArcRange()
			th := (lo + hi) / 2
			px, py = e.Qx+e.Rr*math.Cos(th), e.Qy+e.Rr*math.Sin(th)
			s := e.MatSign()
			nx, ny = -s*math.Cos(th), -s*math.Sin(th)
		}
		for _, f := range []float64{0.25, 0.03, 1e-4} {
			step := f * r.Scale
			if e.Kind == survey2d.SurveyArc && step > e.Rr/2 {
				step = e.Rr / 2
			}
			x, y := px+nx*step, py+ny*step
			if probe(x, y) {
				return [2]float64{x, y}, true
			}
		}
	}
	return [2]float64{}, false
}

// NewRegion2Budget is NewRegion2 with cancellation charged to the caller's
// shared coplanar-scan budget.
func NewRegion2Budget(budget *proofbound.WorkBudget, elems []survey2d.SurveyElem) (Region2, error) {
	scale := 1.0
	grow := func(vs ...float64) {
		for _, v := range vs {
			if a := math.Abs(v); a > scale {
				scale = a
			}
		}
	}
	for _, e := range elems {
		if err := budget.Step(); err != nil {
			return Region2{}, err
		}
		if e.Kind == survey2d.SurveyLine {
			grow(e.Ax, e.Ay, e.Bx, e.By)
			continue
		}
		grow(e.Qx-e.Rr, e.Qx+e.Rr, e.Qy-e.Rr, e.Qy+e.Rr)
	}
	return Region2{Elems: elems, Scale: scale}, nil
}

func RegionContainsBudget(budget *proofbound.WorkBudget, r Region2, px, py float64) (bool, bool, error) {
	for i := range 16 {
		if err := budget.Step(); err != nil {
			return false, false, err
		}
		th := 0.5 + float64(i)*2.399963229728653
		dx, dy := math.Cos(th), math.Sin(th)
		crossings, ok := 0, true
		for _, e := range r.Elems {
			if err := budget.Step(); err != nil {
				return false, false, err
			}
			n, good := survey2d.RayCrossings(e, px, py, dx, dy, r.Tol())
			if !good {
				ok = false
				break
			}
			crossings += n
		}
		if ok {
			return crossings%2 == 1, true, nil
		}
	}
	return false, false, nil
}

func RegionBoundaryDistBudget(budget *proofbound.WorkBudget, r Region2, px, py float64) (float64, error) {
	best := math.Inf(1)
	for _, e := range r.Elems {
		if err := budget.Step(); err != nil {
			return 0, err
		}
		d, _, _ := e.Nearest(px, py, survey2d.SurvTiny*r.Scale)
		if d < best {
			best = d
		}
	}
	return best, nil
}

func RegionClassifyBudget(budget *proofbound.WorkBudget, r Region2, px, py, margin float64) (int, error) {
	distance, err := RegionBoundaryDistBudget(budget, r, px, py)
	if err != nil {
		return 0, err
	}
	if distance <= margin {
		return 0, nil
	}
	in, ok, err := RegionContainsBudget(budget, r, px, py)
	if err != nil {
		return 0, err
	}
	if !ok {
		return 0, nil
	}
	if in {
		return 1, nil
	}
	return -1, nil
}

func RegionSamplesBudget(budget *proofbound.WorkBudget, r Region2) ([][2]float64, error) {
	var out [][2]float64
	for _, e := range r.Elems {
		if err := budget.Step(); err != nil {
			return nil, err
		}
		if e.Kind == survey2d.SurveyLine {
			out = append(out, [2]float64{e.Ax, e.Ay}, [2]float64{(e.Ax + e.Bx) / 2, (e.Ay + e.By) / 2})
			continue
		}
		lo, hi := e.ArcRange()
		for _, th := range []float64{lo, (lo + hi) / 2} {
			out = append(out, [2]float64{e.Qx + e.Rr*math.Cos(th), e.Qy + e.Rr*math.Sin(th)})
		}
	}
	return out, nil
}

func RegionInteriorPointBudget(budget *proofbound.WorkBudget, r Region2) ([2]float64, bool, error) {
	for _, e := range r.Elems {
		if err := budget.Step(); err != nil {
			return [2]float64{}, false, err
		}
		var px, py, nx, ny float64
		if e.Kind == survey2d.SurveyLine {
			px, py = (e.Ax+e.Bx)/2, (e.Ay+e.By)/2
			nx, ny = e.Nx, e.Ny
		} else {
			lo, hi := e.ArcRange()
			th := (lo + hi) / 2
			px, py = e.Qx+e.Rr*math.Cos(th), e.Qy+e.Rr*math.Sin(th)
			s := e.MatSign()
			nx, ny = -s*math.Cos(th), -s*math.Sin(th)
		}
		for _, f := range []float64{0.25, 0.03, 1e-4} {
			if err := budget.Step(); err != nil {
				return [2]float64{}, false, err
			}
			step := f * r.Scale
			if e.Kind == survey2d.SurveyArc && step > e.Rr/2 {
				step = e.Rr / 2
			}
			x, y := px+nx*step, py+ny*step
			class, err := RegionClassifyBudget(budget, r, x, y, r.Tol())
			if err != nil {
				return [2]float64{}, false, err
			}
			if class == 1 {
				return [2]float64{x, y}, true, nil
			}
		}
	}
	return [2]float64{}, false, nil
}
