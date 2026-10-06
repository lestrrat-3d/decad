package clearance

import (
	"context"
	"math"

	"github.com/lestrrat-3d/decad/internal/freeform"

	"github.com/lestrrat-3d/decad/internal/survey2d"

	"github.com/lestrrat-3d/decad/internal/proofbound"

	"github.com/lestrrat-3d/r3"
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

// CKind discriminates the kernel's carrier variants.
type CKind int

const (
	CkPlane CKind = iota
	CkCylinder
	CkCone
	CkSphere
	CkTorus
)

// CFace is one trimmed face in world coordinates: the carrier plus its
// closed-form trim, a containing box, and on-face witness points.
type CFace struct {
	Kind CKind

	// plane: o + x·u + y·v, outward normal n; trim = region.
	O, U, V, N r3.Vec
	Region     Region2

	// axis-symmetric carriers: anchor + axis (unit) + azimuth frame.
	Anchor     r3.Vec
	Axis       r3.Vec
	RefU, RefV r3.Vec
	Radius     float64 // cylinder/sphere radius, torus minor
	Major      float64 // torus major
	Half       float64 // cone half-angle
	Sweep      AngWindow
	ZWin       LinWindow // cylinder: axial range; cone: axial range from apex
	Merid      AngWindow // sphere/torus meridian window
	Spindle    bool      // torus minor >= major: off the polynomial path (§4)

	Box [2]r3.Vec
	Wit []r3.Vec
}

// AdmitState folds two per-side admission classifications: any −1 rejects,
// else any 0 downgrades to a straddle.
func AdmitState(cs ...int) int {
	out := 1
	for _, c := range cs {
		if c == -1 {
			return -1
		}
		if c == 0 {
			out = 0
		}
	}
	return out
}

// planeCoords maps a world point into the plane face's 2D frame.
func (f *CFace) PlaneCoords(p r3.Vec) (float64, float64) {
	rel := p.Sub(f.O)
	return rel.Dot(f.U), rel.Dot(f.V)
}

// planeOffset is the face plane's offset along its unit normal.
func (f *CFace) PlaneOffset() float64 { return f.N.Dot(f.O) }

// axisCoords maps a world point into (z along axis from anchor, ρ, azimuth).
func (f *CFace) AxisCoords(p r3.Vec) (float64, float64, float64) {
	rel := p.Sub(f.Anchor)
	z := rel.Dot(f.Axis)
	rad := rel.Sub(f.Axis.Scale(z))
	rho := rad.Len()
	return z, rho, math.Atan2(rad.Dot(f.RefV), rad.Dot(f.RefU))
}

// admitPoint classifies a world point claimed to lie on the face's carrier
// against the trim: +1 admitted with margin, −1 rejected with margin, 0
// ambiguous. margin is a length.
func (f *CFace) AdmitPoint(p r3.Vec, margin float64) int {
	switch f.Kind {
	case CkPlane:
		x, y := f.PlaneCoords(p)
		return f.Region.Classify(x, y, margin)
	case CkCylinder:
		z, _, phi := f.AxisCoords(p)
		angMargin := margin / math.Max(f.Radius, 1e-30)
		return AdmitState(f.ZWin.Classify(z, margin), f.Sweep.Classify(phi, angMargin))
	case CkCone:
		z, rho, phi := f.AxisCoords(p)
		angMargin := margin / math.Max(rho, 1e-30)
		return AdmitState(f.ZWin.Classify(z, margin), f.Sweep.Classify(phi, angMargin))
	case CkSphere:
		z, rho, phi := f.AxisCoords(p)
		th := math.Atan2(rho, z)
		angMargin := margin / math.Max(f.Radius, 1e-30)
		c := f.Merid.Classify(th, angMargin)
		if rho <= margin {
			// A pole: every azimuth matches; only the meridian decides.
			return c
		}
		return AdmitState(c, f.Sweep.Classify(phi, margin/math.Max(rho, 1e-30)))
	default: // CkTorus
		z, rho, phi := f.AxisCoords(p)
		mu := math.Atan2(rho-f.Major, z)
		angMargin := margin / math.Max(f.Radius, 1e-30)
		c := f.Merid.Classify(mu, angMargin)
		if rho <= margin {
			return c
		}
		return AdmitState(c, f.Sweep.Classify(phi, margin/math.Max(rho, 1e-30)))
	}
}

// BoxOf grows a box over points.
func BoxOf(pts ...r3.Vec) [2]r3.Vec {
	lo := r3.NewVec(math.Inf(1), math.Inf(1), math.Inf(1))
	hi := r3.NewVec(math.Inf(-1), math.Inf(-1), math.Inf(-1))
	for _, p := range pts {
		lo = r3.NewVec(math.Min(lo.X, p.X), math.Min(lo.Y, p.Y), math.Min(lo.Z, p.Z))
		hi = r3.NewVec(math.Max(hi.X, p.X), math.Max(hi.Y, p.Y), math.Max(hi.Z, p.Z))
	}
	return [2]r3.Vec{lo, hi}
}

func BoxUnion(a, b [2]r3.Vec) [2]r3.Vec {
	return [2]r3.Vec{
		r3.NewVec(math.Min(a[0].X, b[0].X), math.Min(a[0].Y, b[0].Y), math.Min(a[0].Z, b[0].Z)),
		r3.NewVec(math.Max(a[1].X, b[1].X), math.Max(a[1].Y, b[1].Y), math.Max(a[1].Z, b[1].Z)),
	}
}

// CircleBox is the exact box of a full 3D circle.
func CircleBox(c, axis r3.Vec, r float64) [2]r3.Vec {
	ext := r3.NewVec(
		r*math.Sqrt(math.Max(0, 1-axis.X*axis.X)),
		r*math.Sqrt(math.Max(0, 1-axis.Y*axis.Y)),
		r*math.Sqrt(math.Max(0, 1-axis.Z*axis.Z)),
	)
	return [2]r3.Vec{c.Sub(ext), c.Add(ext)}
}

// ClrBoxDist is the distance between two boxes (zero when they meet).
func ClrBoxDist(a, b [2]r3.Vec) float64 {
	gap := func(alo, ahi, blo, bhi float64) float64 {
		if ahi < blo {
			return blo - ahi
		}
		if bhi < alo {
			return alo - bhi
		}
		return 0
	}
	dx := gap(a[0].X, a[1].X, b[0].X, b[1].X)
	dy := gap(a[0].Y, a[1].Y, b[0].Y, b[1].Y)
	dz := gap(a[0].Z, a[1].Z, b[0].Z, b[1].Z)
	return math.Sqrt(dx*dx + dy*dy + dz*dz)
}

// FaceFootExtent bounds |p − f.o| over every point p a planar face's own trim
// can admit. f.box contains f.region by construction (every arm that builds a
// CkPlane face's box — CapBox, BoxOf's own callers — grows it to cover the
// region), so the farthest of the box's eight corners from the foot o is a
// sound, if not tight, upper bound on the face's own extent.
func FaceFootExtent(f *CFace) float64 {
	best := 0.0
	for _, x := range []float64{f.Box[0].X, f.Box[1].X} {
		for _, y := range []float64{f.Box[0].Y, f.Box[1].Y} {
			for _, z := range []float64{f.Box[0].Z, f.Box[1].Z} {
				if d := r3.NewVec(x, y, z).Sub(f.O).Len(); d > best {
					best = d
				}
			}
		}
	}
	return best
}

// PlaneTiltAllow bounds the displacement one rounded carrier PLANE commits
// across its own extent from its foot f.o. addPrismFaces and addRevolveFaces
// build a plane carrier's u, v and outward normal n through the payload's own
// `dir`-style construction in raw float64 (this file's own package doc
// comment; docs/clearance-design.md §2), so n is not exactly perpendicular to
// the plane it is meant to describe. For a point p genuinely on the TRUE
// plane through o with the TRUE normal n_true, the rounded plane's own
// equation reads n·(p−o) = (n−n_true)·(p−o) + n_true·(p−o) = (n−n_true)·(p−o)
// — the second term vanishes by definition of the true plane — which
// Cauchy–Schwarz bounds by |n−n_true|·|p−o|. proofbound.DirRoundAllow bounds the first
// factor for the frame/placement composition every such normal is built
// through (a unit-scale input, since every direction this file feeds it is a
// unit or axis vector); the ×4 safety factor is prismPointBound's own rule
// for a value that passes through more than one held linear map — here the
// frame lift, a cross product and a normalize — before publication, so it is
// sound rather than tight. FaceFootExtent bounds the second factor.
func PlaneTiltAllow(f *CFace, frame r3.Frame, xform r3.Transform) float64 {
	if f.Kind != CkPlane {
		return 0
	}
	normalErr := proofbound.ProductUpper(4, proofbound.DirRoundAllow(frame, xform, 1))
	return proofbound.ProductUpper(normalErr, FaceFootExtent(f))
}

// BodyFaceTiltDelta is the worst PlaneTiltAllow over every planar face a
// body's carrier model built — the one per-body term addPrismFaces and
// addRevolveFaces fold into bodyGeom.delta beside the point and axial/angular
// terms (bodyGeom's own doc comment).
func BodyFaceTiltDelta(faces []*CFace, frame r3.Frame, xform r3.Transform) float64 {
	best := 0.0
	for _, f := range faces {
		if t := PlaneTiltAllow(f, frame, xform); t > best {
			best = t
		}
	}
	return best
}

// CEdge is one boundary edge: a segment or a circular arc/circle in world
// coordinates.
type CEdge struct {
	Line   bool
	A, B   r3.Vec // segment endpoints
	Center r3.Vec
	Axis   r3.Vec
	RefU   r3.Vec
	RefV   r3.Vec
	Radius float64
	Ang    AngWindow
	Box    [2]r3.Vec
}

func (e *CEdge) At(th float64) r3.Vec {
	s, c := math.Sincos(th)
	return e.Center.Add(e.RefU.Scale(e.Radius * c)).Add(e.RefV.Scale(e.Radius * s))
}

// PerpTo returns a deterministic unit vector perpendicular to a unit vector.
func PerpTo(a r3.Vec) r3.Vec {
	seed := r3.NewVec(1, 0, 0)
	if math.Abs(a.X) > 0.9 {
		seed = r3.NewVec(0, 1, 0)
	}
	p, _ := a.Cross(seed).Normalize()
	return p
}

// CapBox is the world box of a planar face's region boundary (arcs taken as
// their full circles — conservative, and a box only needs to contain).
func CapBox(f *CFace) [2]r3.Vec {
	at := func(x, y float64) r3.Vec { return f.O.Add(f.U.Scale(x)).Add(f.V.Scale(y)) }
	box := [2]r3.Vec{
		r3.NewVec(math.Inf(1), math.Inf(1), math.Inf(1)),
		r3.NewVec(math.Inf(-1), math.Inf(-1), math.Inf(-1)),
	}
	for _, e := range f.Region.Elems {
		if e.Kind == survey2d.SurveyLine {
			box = BoxUnion(box, BoxOf(at(e.Ax, e.Ay), at(e.Bx, e.By)))
			continue
		}
		axis := f.U.Cross(f.V)
		box = BoxUnion(box, CircleBox(at(e.Qx, e.Qy), axis, e.Rr))
	}
	return box
}

// CapWitnesses returns on-face points: boundary samples plus a verified
// interior point when one is found.
func CapWitnesses(f *CFace) []r3.Vec {
	var out []r3.Vec
	for _, s := range f.Region.Samples() {
		out = append(out, f.O.Add(f.U.Scale(s[0])).Add(f.V.Scale(s[1])))
	}
	if p, ok := f.Region.InteriorPoint(); ok {
		out = append(out, f.O.Add(f.U.Scale(p[0])).Add(f.V.Scale(p[1])))
	}
	return out
}

// AnnularRegion builds the 2D region of a revolve wallPlane face in its
// (e0, e1) frame: an annular sector, a disk sector, an annulus or a disk.
func AnnularRegion(rlo, rhi, phi0, phi1 float64, full bool) Region2 {
	var elems []survey2d.SurveyElem
	if full {
		if e, ok := survey2d.ArcElem(0, 0, rhi, 0, 2*math.Pi, true); ok {
			elems = append(elems, e)
		}
		if rlo > 0 {
			if e, ok := survey2d.ArcElem(0, 0, rlo, 0, 2*math.Pi, true); ok {
				elems = append(elems, e)
			}
		}
		return NewRegion2(elems)
	}
	if e, ok := survey2d.ArcElem(0, 0, rhi, phi0, phi1, false); ok {
		elems = append(elems, e)
	}
	if rlo > 0 {
		if e, ok := survey2d.ArcElem(0, 0, rlo, phi0, phi1, false); ok {
			elems = append(elems, e)
		}
	}
	s0, c0 := math.Sincos(phi0)
	s1, c1 := math.Sincos(phi1)
	if e, ok := survey2d.LineElem(rlo*c0, rlo*s0, rhi*c0, rhi*s0); ok {
		elems = append(elems, e)
	}
	if e, ok := survey2d.LineElem(rlo*c1, rlo*s1, rhi*c1, rhi*s1); ok {
		elems = append(elems, e)
	}
	return NewRegion2(elems)
}

// ClrLadder is the deterministic cast-direction ladder of §2: fixed,
// never random, so a replay resolves identically.
func ClrLadder() []r3.Vec {
	out := make([]r3.Vec, 16)
	for i := range out {
		th := 0.5 + float64(i)*2.399963229728653
		z := 1 - 2*(float64(i)+0.5)/16
		r := math.Sqrt(math.Max(0, 1-z*z))
		out[i] = r3.NewVec(r*math.Cos(th), r*math.Sin(th), z)
	}
	return out
}

// survey2d.RayCrossings counts certified transversal crossings of the ray p + t·dir
// (t > tol) with the trimmed face; good is false on any ambiguity.
func (f *CFace) RayCrossings(ctx context.Context, p, dir r3.Vec, tol float64) (int, bool, error) {
	if err := ctx.Err(); err != nil {
		return 0, false, err
	}
	switch f.Kind {
	case CkPlane:
		nd := f.N.Dot(dir)
		if nd == 0 {
			// EXACTLY parallel: the ray never meets the carrier when the
			// start is cleanly off the plane; on the plane it is a graze —
			// ambiguous, retry the ladder.
			if math.Abs(f.N.Dot(p)-f.PlaneOffset()) > tol {
				return 0, true, nil
			}
			return 0, false, nil
		}
		if math.Abs(nd) < 1e-7 {
			// Near-parallel: the crossing exists but sits far away and
			// poorly conditioned — the count is not certified; retry the
			// ladder rather than claim zero.
			return 0, false, nil
		}
		t := (f.PlaneOffset() - f.N.Dot(p)) / nd
		if math.Abs(t) <= tol {
			// The carrier passes through the ray start (a witness on a
			// shared construction plane): a crossing only if the start sits
			// on the trimmed face — cleanly off it, no crossing.
			if f.AdmitPoint(p.Add(dir.Scale(t)), tol) == -1 {
				return 0, true, nil
			}
			return 0, false, nil
		}
		if t < 0 {
			return 0, true, nil
		}
		switch f.AdmitPoint(p.Add(dir.Scale(t)), tol) {
		case 1:
			return 1, true, nil
		case -1:
			return 0, true, nil
		default:
			return 0, false, nil
		}
	case CkCylinder:
		rel := p.Sub(f.Anchor)
		relP := rel.Sub(f.Axis.Scale(rel.Dot(f.Axis)))
		dirP := dir.Sub(f.Axis.Scale(dir.Dot(f.Axis)))
		a := dirP.Dot(dirP)
		b := relP.Dot(dirP)
		c := relP.Dot(relP) - f.Radius*f.Radius
		n, ok := f.QuadraticCrossings(p, dir, a, b, c, tol)
		return n, ok, nil
	case CkSphere:
		rel := p.Sub(f.Anchor)
		n, ok := f.QuadraticCrossings(p, dir, 1, rel.Dot(dir), rel.Dot(rel)-f.Radius*f.Radius, tol)
		return n, ok, nil
	case CkCone:
		rel := p.Sub(f.Anchor)
		k := math.Tan(f.Half)
		az := rel.Dot(f.Axis)
		dz := dir.Dot(f.Axis)
		relP := rel.Sub(f.Axis.Scale(az))
		dirP := dir.Sub(f.Axis.Scale(dz))
		a := dirP.Dot(dirP) - k*k*dz*dz
		b := relP.Dot(dirP) - k*k*az*dz
		c := relP.Dot(relP) - k*k*az*az
		n, ok := f.QuadraticCrossings(p, dir, a, b, c, tol)
		if !ok {
			return 0, false, nil
		}
		return n, true, nil
	default: // CkTorus
		return f.TorusCrossings(ctx, p, dir, tol)
	}
}

// quadraticCrossings counts admitted roots of a·t² + 2b·t + c = 0 along the
// ray; the cone caller relies on admitPoint's z-window (from the apex,
// positive) to reject the wrong nappe.
func (f *CFace) QuadraticCrossings(p, dir r3.Vec, a, b, c, tol float64) (int, bool) {
	if math.Abs(a) < 1e-14 {
		return 0, false
	}
	disc := b*b - a*c
	scale := math.Abs(a)*f.Radius*f.Radius + math.Abs(c) + 1
	if f.Kind == CkCone {
		scale = math.Abs(c) + math.Abs(b) + 1
	}
	if disc <= 0 {
		if disc > -1e-9*scale {
			return 0, false
		}
		return 0, true
	}
	s := math.Sqrt(disc)
	if s <= 1e-9*math.Sqrt(scale) {
		return 0, false
	}
	n := 0
	for _, t := range []float64{(-b - s) / a, (-b + s) / a} {
		q := p.Add(dir.Scale(t))
		if f.Kind == CkCone && q.Sub(f.Anchor).Dot(f.Axis) < 0 {
			continue
		}
		if math.Abs(t) <= tol {
			// The carrier passes through the ray start: cleanly off the
			// trimmed face means no crossing; anything else is ambiguous.
			if f.AdmitPoint(q, tol) == -1 {
				continue
			}
			return 0, false
		}
		if t < 0 {
			continue
		}
		switch f.AdmitPoint(q, tol) {
		case 1:
			n++
		case -1:
		default:
			return 0, false
		}
	}
	return n, true
}

// torusCrossings counts admitted ray roots of the torus quartic with Sturm
// certification (§2): a root count is a proof only when the isolation
// certifies it — a non-square-free quartic (a tangency) is ambiguous and the
// ladder retries.
func (f *CFace) TorusCrossings(ctx context.Context, p, dir r3.Vec, tol float64) (int, bool, error) {
	if err := ctx.Err(); err != nil {
		return 0, false, err
	}
	rel := p.Sub(f.Anchor)
	// |x−C|² = q2 t² + q1 t + q0; axial² = (a0 + a1 t)².
	q2 := dir.Dot(dir)
	q1 := 2 * rel.Dot(dir)
	q0 := rel.Dot(rel)
	a0 := rel.Dot(f.Axis)
	a1 := dir.Dot(f.Axis)
	k := f.Major*f.Major + q0 - f.Radius*f.Radius
	// f(t) = (|x−C|² + R² − r²)² − 4R²(|x−C|² − axial²)
	quad, ok := freeform.RatPolyOf(k, q1, q2)
	if !ok {
		return 0, false, nil
	}
	sq := freeform.RpMul(quad, quad)
	perpBase, ok := freeform.RatPolyOf(q0, q1, q2)
	if !ok {
		return 0, false, nil
	}
	axial, ok := freeform.RatPolyOf(a0, a1)
	if !ok {
		return 0, false, nil
	}
	perp := freeform.RpSub(perpBase, freeform.RpMul(axial, axial))
	four, ok := proofbound.RatOf(4 * f.Major * f.Major)
	if !ok {
		return 0, false, nil
	}
	poly := freeform.RpTrim(freeform.RpSub(sq, freeform.RpScale(perp, four)))
	if freeform.RpDeg(poly) < 1 {
		return 0, false, nil
	}
	sf := freeform.RpSquareFree(poly)
	if freeform.RpDeg(sf) != freeform.RpDeg(poly) {
		// A repeated root is a tangency somewhere on the line: ambiguous.
		return 0, false, nil
	}
	chain, err := freeform.SturmChainIntContext(ctx, sf)
	if err != nil {
		return 0, false, err
	}
	n := 0
	ivs, err := freeform.RpIsolateRootsContext(ctx, sf, chain)
	if err != nil {
		return 0, false, err
	}
	for _, iv := range ivs {
		iv, err = freeform.RpRefineRootContext(ctx, chain, iv, func(lo, hi float64) bool { return hi-lo <= 1e-11*math.Max(1, math.Abs(lo)) })
		if err != nil {
			return 0, false, err
		}
		tLo, _ := iv.Lo.Float64()
		tHi, _ := iv.Hi.Float64()
		if tHi <= -tol {
			continue // behind the start
		}
		if tLo <= tol {
			// At or straddling the start: a crossing there can only be the
			// carrier passing through the witness — cleanly off the trimmed
			// face means no crossing, anything else is ambiguous.
			q0 := p.Add(dir.Scale((tLo + tHi) / 2))
			if tHi-tLo <= tol && f.AdmitPoint(q0, tol+(tHi-tLo)) == -1 {
				continue
			}
			return 0, false, nil
		}
		q := p.Add(dir.Scale((tLo + tHi) / 2))
		switch f.AdmitPoint(q, tol+(tHi-tLo)) {
		case 1:
			n++
		case -1:
		default:
			return 0, false, nil
		}
	}
	return n, true, nil
}
