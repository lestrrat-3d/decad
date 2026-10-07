package clearance

import (
	"math"

	"github.com/lestrrat-3d/decad/internal/survey2d"

	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// ClrAngTol is the dimensionless angular tolerance for parallelism and
// window-membership decisions, matching the evaluator's own 1e-9-relative
// classification style (revolve.go's snapTol).
const ClrAngTol = 1e-9

// AngWindow is a closed angular window [lo, hi] (hi − lo < 2π) or the full
// circle.
type AngWindow struct {
	Lo, Hi float64
	Full   bool
}

// NewAngWindow builds the window. A window is FULL only when it really closes
// on itself: `full` is an ADMISSION — it says every azimuth lies in the trim —
// and a tolerance that rounds a window up to full admits carrier points inside
// the sliver the window is missing. The closed carriers say so themselves (a
// Circle3 edge, a closed sphere/torus meridian, a full revolve) and pass
// `full` directly; nothing else earns it on a near miss.
func NewAngWindow(a, b float64) AngWindow {
	lo, hi := math.Min(a, b), math.Max(a, b)
	if hi-lo >= 2*math.Pi {
		return AngWindow{Full: true}
	}
	return AngWindow{Lo: lo, Hi: hi}
}

// classify reports +1 when th lies in the window with more than margin to
// spare, −1 when it is out by more than margin, 0 in between.
func (w AngWindow) Classify(th, margin float64) int {
	if w.Full {
		return 1
	}
	off := survey2d.Mod2pi(th - w.Lo)
	ext := w.Hi - w.Lo
	if off <= ext {
		if off >= margin && ext-off >= margin {
			return 1
		}
		return 0
	}
	if off-ext > margin && 2*math.Pi-off > margin {
		return -1
	}
	return 0
}

// LinWindow is a closed interval on a linear coordinate.
type LinWindow struct{ Lo, Hi float64 }

func NewLinWindow(a, b float64) LinWindow {
	return LinWindow{Lo: math.Min(a, b), Hi: math.Max(a, b)}
}

func (w LinWindow) Classify(z, margin float64) int {
	if z >= w.Lo+margin && z <= w.Hi-margin {
		return 1
	}
	if z < w.Lo-margin || z > w.Hi+margin {
		return -1
	}
	return 0
}

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

// ElemLineHits collects the crossing parameters of the (infinite) 2D line
// p + t·d with one boundary element; ok is false on an ambiguous geometry
// (near-parallel overlap, grazing an endpoint).
func ElemLineHits(e survey2d.SurveyElem, px, py, dx, dy, tol float64) ([]float64, bool) {
	if e.Kind == survey2d.SurveyLine {
		ex, ey := e.Bx-e.Ax, e.By-e.Ay
		seg := math.Hypot(ex, ey)
		det := dx*(-ey) + ex*dy
		if math.Abs(det) <= 1e-12*math.Max(1, seg) {
			if math.Abs((e.Ax-px)*dy-(e.Ay-py)*dx) <= tol {
				return nil, false
			}
			return nil, true
		}
		rx, ry := e.Ax-px, e.Ay-py
		t := (rx*(-ey) + ex*ry) / det
		u := (dx*ry - dy*rx) / det
		uTol := tol / seg
		if u < -uTol || u > 1+uTol {
			return nil, true
		}
		if u <= uTol || u >= 1-uTol {
			return nil, false
		}
		return []float64{t}, true
	}
	fx, fy := px-e.Qx, py-e.Qy
	b := fx*dx + fy*dy
	cc := fx*fx + fy*fy - e.Rr*e.Rr
	disc := b*b - cc
	if disc <= 0 {
		if disc > -tol*e.Rr {
			return nil, false
		}
		return nil, true
	}
	s := math.Sqrt(disc)
	if s <= tol {
		return nil, false
	}
	var out []float64
	for _, t := range []float64{-b - s, -b + s} {
		x, y := px+t*dx, py+t*dy
		th := math.Atan2(y-e.Qy, x-e.Qx)
		if e.Closed {
			out = append(out, t)
			continue
		}
		lo, hi := e.ArcRange()
		off := survey2d.Mod2pi(th - lo)
		ext := hi - lo
		angTol := tol / e.Rr
		if off <= angTol || math.Abs(off-ext) <= angTol || math.Abs(off-2*math.Pi) <= angTol {
			return nil, false
		}
		if off < ext {
			out = append(out, t)
		}
	}
	return out, true
}

// ClrIv is a float interval.
type ClrIv struct{ Lo, Hi float64 }

// lineIntervals returns the parameter intervals along the unit-direction 2D
// line p + t·d that lie inside the region; ok is false when the crossing
// structure is ambiguous.
func (r Region2) LineIntervals(px, py, dx, dy float64) ([]ClrIv, bool) {
	var ts []float64
	for _, e := range r.Elems {
		hits, ok := ElemLineHits(e, px, py, dx, dy, r.Tol())
		if !ok {
			return nil, false
		}
		ts = append(ts, hits...)
	}
	if len(ts) == 0 {
		return nil, true
	}
	// Sort the crossings; between consecutive crossings membership is
	// constant, probed at the midpoint.
	for i := 1; i < len(ts); i++ {
		for j := i; j > 0 && ts[j] < ts[j-1]; j-- {
			ts[j], ts[j-1] = ts[j-1], ts[j]
		}
	}
	var out []ClrIv
	for i := 0; i+1 < len(ts); i++ {
		mid := (ts[i] + ts[i+1]) / 2
		in, ok := r.Contains(px+mid*dx, py+mid*dy)
		if !ok {
			return nil, false
		}
		if in {
			out = append(out, ClrIv{Lo: ts[i], Hi: ts[i+1]})
		}
	}
	return out, true
}

// ElemLineSuperset returns the proper crossing parameters of the unit 2D
// line p + t·d with one element, plus conservative spans covering every
// ambiguous contact (a parallel overlap, an endpoint graze, a tangency) —
// the ingredients of a SUPERSET of the region's intersection with the line,
// which may only ever exclude, never bless.
func ElemLineSuperset(e survey2d.SurveyElem, px, py, dx, dy, tol float64) ([]float64, []ClrIv) {
	if e.Kind == survey2d.SurveyLine {
		ex, ey := e.Bx-e.Ax, e.By-e.Ay
		seg := math.Hypot(ex, ey)
		det := dx*(-ey) + ex*dy
		if math.Abs(det) <= 1e-12*math.Max(1, seg) {
			if math.Abs((e.Ax-px)*dy-(e.Ay-py)*dx) <= tol {
				ta := (e.Ax-px)*dx + (e.Ay-py)*dy
				tb := (e.Bx-px)*dx + (e.By-py)*dy
				return nil, []ClrIv{{Lo: math.Min(ta, tb) - tol, Hi: math.Max(ta, tb) + tol}}
			}
			return nil, nil
		}
		rx, ry := e.Ax-px, e.Ay-py
		t := (rx*(-ey) + ex*ry) / det
		u := (dx*ry - dy*rx) / det
		uTol := tol / seg
		if u < -uTol || u > 1+uTol {
			return nil, nil
		}
		if u <= uTol || u >= 1-uTol {
			return nil, []ClrIv{{Lo: t - tol, Hi: t + tol}}
		}
		return []float64{t}, nil
	}
	fx, fy := px-e.Qx, py-e.Qy
	b := fx*dx + fy*dy
	cc := fx*fx + fy*fy - e.Rr*e.Rr
	disc := b*b - cc
	if disc <= 0 {
		if disc > -tol*e.Rr {
			w := math.Sqrt(math.Abs(disc)) + tol
			return nil, []ClrIv{{Lo: -b - w, Hi: -b + w}}
		}
		return nil, nil
	}
	s := math.Sqrt(disc)
	if s <= tol {
		return nil, []ClrIv{{Lo: -b - s - tol, Hi: -b + s + tol}}
	}
	var hits []float64
	var spans []ClrIv
	for _, t := range []float64{-b - s, -b + s} {
		x, y := px+t*dx, py+t*dy
		th := math.Atan2(y-e.Qy, x-e.Qx)
		if e.Closed {
			hits = append(hits, t)
			continue
		}
		lo, hi := e.ArcRange()
		off := survey2d.Mod2pi(th - lo)
		ext := hi - lo
		angTol := tol / e.Rr
		if off <= angTol || math.Abs(off-ext) <= angTol || math.Abs(off-2*math.Pi) <= angTol {
			spans = append(spans, ClrIv{Lo: t - tol, Hi: t + tol})
			continue
		}
		if off < ext {
			hits = append(hits, t)
		}
	}
	return hits, spans
}

// lineIntervalsSuperset returns a conservative superset of the region's
// intersection with the unit 2D line p + t·d: the parity-inside intervals
// between proper crossings plus every ambiguous contact's span. Sound for
// exclusion only — an empty result proves the line misses the closed region.
func (r Region2) LineIntervalsSuperset(px, py, dx, dy float64) []ClrIv {
	var ts []float64
	var spans []ClrIv
	for _, e := range r.Elems {
		hits, sp := ElemLineSuperset(e, px, py, dx, dy, r.Tol())
		ts = append(ts, hits...)
		spans = append(spans, sp...)
	}
	for i := 1; i < len(ts); i++ {
		for j := i; j > 0 && ts[j] < ts[j-1]; j-- {
			ts[j], ts[j-1] = ts[j-1], ts[j]
		}
	}
	for i := 0; i+1 < len(ts); i++ {
		mid := (ts[i] + ts[i+1]) / 2
		in, ok := r.Contains(px+mid*dx, py+mid*dy)
		if !ok || in {
			// Undecidable membership is conservatively included.
			spans = append(spans, ClrIv{Lo: ts[i], Hi: ts[i+1]})
		}
	}
	return spans
}

// segmentHits classifies a 2D segment against the region: +1 provably
// intersecting, −1 provably disjoint, 0 ambiguous. The +1 witness point is
// returned for admitted candidates.
func (r Region2) SegmentHits(ax, ay, bx, by float64) (int, [2]float64) {
	l := math.Hypot(bx-ax, by-ay)
	if l <= r.Tol() {
		c := r.Classify(ax, ay, r.Tol())
		return c, [2]float64{ax, ay}
	}
	dx, dy := (bx-ax)/l, (by-ay)/l
	ivs, ok := r.LineIntervals(ax, ay, dx, dy)
	if ok {
		for _, iv := range ivs {
			lo := math.Max(iv.Lo, 0)
			hi := math.Min(iv.Hi, l)
			if hi-lo > 2*r.Tol() {
				m := (lo + hi) / 2
				return 1, [2]float64{ax + m*dx, ay + m*dy}
			}
		}
	}
	// Endpoint membership decides containment with no crossing.
	for _, p := range [][2]float64{{ax, ay}, {bx, by}, {(ax + bx) / 2, (ay + by) / 2}} {
		if r.Classify(p[0], p[1], r.Tol()) == 1 {
			return 1, p
		}
	}
	// Provably disjoint: the segment clears every boundary element and one
	// endpoint is cleanly outside — a segment entering the region would have
	// to cross the boundary.
	clearing := math.Inf(1)
	for _, e := range r.Elems {
		d := SegElemDistLB(e, ax, ay, bx, by)
		if d < clearing {
			clearing = d
		}
	}
	if clearing > r.Tol() && r.Classify(ax, ay, r.Tol()) == -1 {
		return -1, [2]float64{}
	}
	return 0, [2]float64{}
}

// SegElemDistLB is a lower bound on the distance between a 2D segment and a
// boundary element (the arc bound goes through its full circle — an
// underestimate, which is the sound direction for exclusion proofs).
func SegElemDistLB(e survey2d.SurveyElem, ax, ay, bx, by float64) float64 {
	if e.Kind == survey2d.SurveyLine {
		return SegSegDist(ax, ay, bx, by, e.Ax, e.Ay, e.Bx, e.By)
	}
	lo, hi := SegPointDistRange(ax, ay, bx, by, e.Qx, e.Qy)
	if hi < e.Rr {
		return e.Rr - hi
	}
	if lo > e.Rr {
		return lo - e.Rr
	}
	return 0
}

// SegPointDistRange is the exact [min, max] of the distance from a point to
// a segment's points.
func SegPointDistRange(ax, ay, bx, by, px, py float64) (float64, float64) {
	ex, ey := bx-ax, by-ay
	l2 := ex*ex + ey*ey
	da := math.Hypot(px-ax, py-ay)
	db := math.Hypot(px-bx, py-by)
	mx := math.Max(da, db)
	if l2 == 0 {
		return da, mx
	}
	u := ((px-ax)*ex + (py-ay)*ey) / l2
	if u <= 0 || u >= 1 {
		return math.Min(da, db), mx
	}
	fx, fy := ax+u*ex-px, ay+u*ey-py
	return math.Hypot(fx, fy), mx
}

// SegSegDist is the exact distance between two 2D segments.
func SegSegDist(ax, ay, bx, by, cx, cy, dx, dy float64) float64 {
	if SegsIntersect(ax, ay, bx, by, cx, cy, dx, dy) {
		return 0
	}
	d1, _ := SegPointDistRange(ax, ay, bx, by, cx, cy)
	d2, _ := SegPointDistRange(ax, ay, bx, by, dx, dy)
	d3, _ := SegPointDistRange(cx, cy, dx, dy, ax, ay)
	d4, _ := SegPointDistRange(cx, cy, dx, dy, bx, by)
	return math.Min(math.Min(d1, d2), math.Min(d3, d4))
}

func SegsIntersect(ax, ay, bx, by, cx, cy, dx, dy float64) bool {
	o := func(px, py, qx, qy, rx, ry float64) float64 {
		return (qx-px)*(ry-py) - (qy-py)*(rx-px)
	}
	o1 := o(ax, ay, bx, by, cx, cy)
	o2 := o(ax, ay, bx, by, dx, dy)
	o3 := o(cx, cy, dx, dy, ax, ay)
	o4 := o(cx, cy, dx, dy, bx, by)
	return o1*o2 < 0 && o3*o4 < 0
}
