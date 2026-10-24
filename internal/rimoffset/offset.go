// Package rimoffset offsets a smooth closed section loop of fit splines and lines, the foot curve of a
// free-form cap-loop chamfer (agent-docs/modify-reach-design.md §8.5).
package rimoffset

import (
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/sketch/geom"
)

// MaxJointAngle is the largest turn, in radians, at a joint between two consecutive segments that
// the offset still reads as one smooth curve. A larger turn is a corner, and a corner needs a miter
// the exact offset construction owns (modify-reach §8.3).
const MaxJointAngle = 10 * math.Pi / 180

// sampleSteps are the distances, in spans between the recorded fit points, between the samples the offset spline
// passes through, widest first. A build is charged for every point of a spline, so OffsetLoop takes the first
// step that meets MaxDeviationRatio.
var sampleSteps = []float64{3, 2, 1, 0.5}

// MaxDeviationRatio is the tolerance OffsetLoop is asked for, as a fraction of the chamfer setback: how far the
// refit offset curve may sit from the exact offset of the recorded spline at a point between two samples. A larger
// gap is refused: the check only falsifies, a small gap proves nothing (CLAUDE.md, hard rules).
const MaxDeviationRatio = 2e-3

// minRadiusFraction is the smallest part of a radius of curvature an offset may leave. A recorded
// curve whose radius of curvature is within this fraction of the offset distance would fold the
// offset onto itself.
const minRadiusFraction = 0.05

// Result is an offset loop and the largest gap measured between it and the exact offset.
type Result struct {
	// Segments are the offset loop's segments, in the order and sense of the recorded loop.
	Segments []sectionrecord.CurveSegment
	// Step is the spacing, in spans between the recorded fit points, of the samples the offset splines pass through.
	Step float64
	// Deviation is the largest distance, in millimeters, measured from the exact offset of the
	// recorded curve to the offset spline, over points between the samples the spline passes through.
	Deviation float64
}

type vec struct{ x, y float64 }

func (a vec) add(b vec) vec       { return vec{a.x + b.x, a.y + b.y} }
func (a vec) sub(b vec) vec       { return vec{a.x - b.x, a.y - b.y} }
func (a vec) scale(s float64) vec { return vec{a.x * s, a.y * s} }
func (a vec) len() float64        { return math.Hypot(a.x, a.y) }
func (a vec) unit() vec           { return a.scale(1 / a.len()) }
func (a vec) left() vec           { return vec{-a.y, a.x} }
func (a vec) dot(b vec) float64   { return a.x*b.x + a.y*b.y }

// piece is one recorded segment read in the walk order of the loop.
type piece struct {
	fit  []vec // the fit points of a spline; the two endpoints of a line
	line bool
}

func (p piece) first() vec { return p.fit[0] }
func (p piece) last() vec  { return p.fit[len(p.fit)-1] }

// fitCoords converts points to the coordinate pairs geom evaluates.
func fitCoords(points []vec) [][2]float64 {
	out := make([][2]float64, len(points))
	for i, p := range points {
		out[i] = [2]float64{p.x, p.y}
	}
	return out
}

// at returns the point of the piece at the normalized parameter t and the unit walk tangent there.
func (p piece) at(t float64) (vec, vec, error) {
	if p.line {
		return p.fit[0].add(p.fit[1].sub(p.fit[0]).scale(t)), p.fit[1].sub(p.fit[0]).unit(), nil
	}
	fit := fitCoords(p.fit)
	x, y, err := geom.EvalFitSpline(fit, t)
	if err != nil {
		return vec{}, vec{}, err
	}
	dx, dy, err := geom.EvalFitSplineDeriv(fit, t)
	if err != nil {
		return vec{}, vec{}, err
	}
	d := vec{dx, dy}
	if d.len() == 0 {
		return vec{}, vec{}, fmt.Errorf(`%w: a recorded fit spline has no tangent`, decaderr.ErrDegenerate)
	}
	return vec{x, y}, d.unit(), nil
}

// curvature returns the signed curvature of the piece at t, positive when the walk turns left.
func (p piece) curvature(t float64) (float64, error) {
	if p.line {
		return 0, nil
	}
	const h = 1e-4
	lo, hi := math.Max(t-h, 0), math.Min(t+h, 1)
	a, ta, err := p.at(lo)
	if err != nil {
		return 0, err
	}
	b, tb, err := p.at(hi)
	if err != nil {
		return 0, err
	}
	turn := math.Atan2(ta.x*tb.y-ta.y*tb.x, ta.dot(tb))
	run := b.sub(a).len()
	if run == 0 {
		return 0, fmt.Errorf(`%w: a recorded fit spline has a zero-length span`, decaderr.ErrDegenerate)
	}
	return turn / run, nil
}

// pieces reads loop as walk-ordered pieces; a segment kind other than a whole fit spline or a whole line
// is unsupported.
func pieces(loop sectionrecord.LoopRecord) ([]piece, error) {
	out := make([]piece, 0, len(loop.Segments))
	for _, segment := range loop.Segments {
		switch s := segment.(type) {
		case sectionrecord.FitSplineSeg:
			forward, ok := wholeRange(s.TStart, s.TEnd)
			if !ok {
				return nil, fmt.Errorf(`%w: the chamfered rim holds a trimmed fit spline`, decaderr.ErrUnsupported)
			}
			fit := make([]vec, len(s.Fit))
			for i, q := range s.Fit {
				fit[i] = vec{q.U, q.V}
			}
			if !forward {
				reverse(fit)
			}
			out = append(out, piece{fit: fit})
		case sectionrecord.LineSeg:
			forward, ok := wholeRange(s.TStart, s.TEnd)
			if !ok {
				return nil, fmt.Errorf(`%w: the chamfered rim holds a trimmed line`, decaderr.ErrUnsupported)
			}
			fit := []vec{{s.Start.U, s.Start.V}, {s.End.U, s.End.V}}
			if !forward {
				reverse(fit)
			}
			out = append(out, piece{fit: fit, line: true})
		default:
			return nil, fmt.Errorf(`%w: the chamfered rim holds a %T, and only fit splines and lines offset`,
				decaderr.ErrUnsupported, segment)
		}
	}
	if len(out) < 2 {
		return nil, fmt.Errorf(`%w: the chamfered rim has fewer than two segments`, decaderr.ErrUnsupported)
	}
	return out, nil
}

// wholeRange reports whether [ts, te] is the whole curve, and whether it is walked forward.
func wholeRange(ts, te float64) (forward, whole bool) {
	switch {
	case ts == 0 && te == 1:
		return true, true
	case ts == 1 && te == 0:
		return false, true
	}
	return false, false
}

func reverse(v []vec) {
	for i, j := 0, len(v)-1; i < j; i, j = i+1, j-1 {
		v[i], v[j] = v[j], v[i]
	}
}

// OffsetLoop moves every point of loop, a counter-clockwise outer loop or a clockwise hole, by dist
// to the left of its walk, which is toward the material; a negative dist moves it away. Lines move
// exactly. A fit spline is sampled every few spans, each sample moves along its normal,
// and the new fit spline interpolates the moved samples. The joints between segments move along the
// bisector of their two normals, so the offset loop closes. A joint that turns by more than
// MaxJointAngle, an offset that would fold the curve, or a gap beyond tol is refused.
func OffsetLoop(loop sectionrecord.LoopRecord, dist, tol float64) (Result, error) {
	if dist == 0 || math.IsNaN(dist) || math.IsInf(dist, 0) {
		return Result{}, fmt.Errorf(`%w: the rim offset %v is not a finite non-zero length`, decaderr.ErrDegenerate, dist)
	}
	ps, err := pieces(loop)
	if err != nil {
		return Result{}, err
	}
	joints, err := jointPoints(ps, dist)
	if err != nil {
		return Result{}, err
	}
	var result Result
	for _, step := range sampleSteps {
		result, err = offsetPieces(ps, joints, dist, step)
		if err != nil {
			return Result{}, err
		}
		result.Step = step
		if result.Deviation <= tol {
			return result, nil
		}
	}
	return Result{}, fmt.Errorf(`%w: the offset spline departs %.3g mm from the exact offset, past the %.3g mm tolerance`,
		decaderr.ErrUnsupported, result.Deviation, tol)
}

// offsetPieces offsets every piece, sampling a fit spline every step spans.
func offsetPieces(ps []piece, joints []vec, dist float64, step float64) (Result, error) {
	var result Result
	for i, p := range ps {
		start, end := joints[i], joints[(i+1)%len(ps)]
		if p.line {
			result.Segments = append(result.Segments, sectionrecord.LineSeg{
				Start: point2(start), End: point2(end), TStart: 0, TEnd: 1,
			})
			continue
		}
		fit, deviation, err := offsetSpline(p, start, end, dist, step)
		if err != nil {
			return Result{}, err
		}
		result.Deviation = math.Max(result.Deviation, deviation)
		out := make([]sectionrecord.Point2, len(fit))
		for k, q := range fit {
			out[k] = point2(q)
		}
		result.Segments = append(result.Segments, sectionrecord.FitSplineSeg{Fit: out, TStart: 0, TEnd: 1})
	}
	return result, nil
}

func point2(v vec) sectionrecord.Point2 { return sectionrecord.Point2{U: v.x, V: v.y} }

// jointPoints returns the offset of the start of every piece. Piece i ends where piece i+1 starts, so
// the offset of that point is shared; both pieces' normals at it are averaged.
func jointPoints(ps []piece, dist float64) ([]vec, error) {
	joints := make([]vec, len(ps))
	for i := range ps {
		prev := ps[(i+len(ps)-1)%len(ps)]
		_, tPrev, err := prev.at(1)
		if err != nil {
			return nil, err
		}
		_, tNext, err := ps[i].at(0)
		if err != nil {
			return nil, err
		}
		if gap := prev.last().sub(ps[i].first()).len(); gap > 1e-6*(1+prev.last().len()) {
			return nil, fmt.Errorf(`%w: the chamfered rim is not a closed loop`, decaderr.ErrDegenerate)
		}
		if turn := math.Abs(math.Atan2(tPrev.x*tNext.y-tPrev.y*tNext.x, tPrev.dot(tNext))); turn > MaxJointAngle {
			return nil, fmt.Errorf(`%w: the chamfered rim has a corner of %.1f°, and only a smooth rim offsets`,
				decaderr.ErrUnsupported, turn*180/math.Pi)
		}
		n := tPrev.left().add(tNext.left())
		if n.len() == 0 {
			return nil, fmt.Errorf(`%w: the chamfered rim doubles back on itself`, decaderr.ErrUnsupported)
		}
		joints[i] = ps[i].first().add(n.unit().scale(dist))
	}
	return joints, nil
}

// offsetSpline returns the fit points of the offset of spline piece p, which runs from start to end,
// and the largest measured distance between those points' spline and the exact offset.
func offsetSpline(p piece, start, end vec, dist float64, step float64) ([]vec, float64, error) {
	params := stationParams(p.fit, step)
	fit := make([]vec, len(params))
	for i, t := range params {
		switch i {
		case 0:
			fit[i] = start
			continue
		case len(params) - 1:
			fit[i] = end
			continue
		}
		q, err := exactOffset(p, t, dist)
		if err != nil {
			return nil, 0, err
		}
		fit[i] = q
	}
	deviation, err := measureDeviation(p, fit, params, dist)
	return fit, deviation, err
}

// exactOffset returns the point at t moved by dist to the left of the walk, refusing a point where
// the offset would fold the curve.
func exactOffset(p piece, t, dist float64) (vec, error) {
	q, tangent, err := p.at(t)
	if err != nil {
		return vec{}, err
	}
	k, err := p.curvature(t)
	if err != nil {
		return vec{}, err
	}
	if 1-dist*k < minRadiusFraction {
		return vec{}, fmt.Errorf(`%w: a %.3g mm rim offset reaches the radius of curvature %.3g mm of the recorded rim`,
			decaderr.ErrDegenerate, math.Abs(dist), 1/math.Abs(k))
	}
	return q.add(tangent.left().scale(dist)), nil
}

// stationPhase is where in each step the sample lies, as a fraction of the step. A sample off the recorded fit
// points keeps the offset spline's joints and chord stations off the recorded spline's own, so the cutter's
// rulings do not cross the receiver's mesh edges exactly, where a Boolean's result vertex lands within rounding
// of an edge it must not fold through.
const stationPhase = 0.3819660112501051

// stationParams returns the normalized chord-length parameters of the samples: both ends, and the points
// at stationPhase of every step of length step along the span-index axis, where recorded fit point i sits at
// index i. A step of 1 is one sample per span between recorded fit points, 2 is one per two spans.
func stationParams(fit []vec, step float64) []float64 {
	cum := make([]float64, len(fit))
	for i := 1; i < len(fit); i++ {
		cum[i] = cum[i-1] + fit[i].sub(fit[i-1]).len()
	}
	total := cum[len(cum)-1]
	spans := float64(len(fit) - 1)
	out := []float64{0}
	for k := 0; ; k++ {
		u := (float64(k) + stationPhase) * step
		if spans-u < 0.2*step {
			break
		}
		i := int(u)
		out = append(out, (cum[i]+(cum[i+1]-cum[i])*(u-float64(i)))/total)
	}
	return append(out, 1)
}

// measureDeviation returns the largest distance from the exact offset at the middle of each step
// between two samples to the polyline of the spline through fit.
func measureDeviation(p piece, fit []vec, params []float64, dist float64) (float64, error) {
	const perStep = 32
	poly, err := geom.SampleFitSpline(fitCoords(fit), perStep*(len(fit)-1))
	if err != nil {
		return 0, err
	}
	worst := 0.0
	for i := 0; i+1 < len(params); i++ {
		mid, err := exactOffset(p, (params[i]+params[i+1])/2, dist)
		if err != nil {
			return 0, err
		}
		worst = math.Max(worst, distanceToPolyline(mid, poly))
	}
	return worst, nil
}

// distanceToPolyline returns the distance from q to the polyline.
func distanceToPolyline(q vec, poly [][2]float64) float64 {
	best := math.Inf(1)
	for i := 0; i+1 < len(poly); i++ {
		a, b := vec{poly[i][0], poly[i][1]}, vec{poly[i+1][0], poly[i+1][1]}
		ab := b.sub(a)
		s := 0.0
		if l2 := ab.dot(ab); l2 > 0 {
			s = math.Min(math.Max(q.sub(a).dot(ab)/l2, 0), 1)
		}
		best = math.Min(best, q.sub(a.add(ab.scale(s))).len())
	}
	return best
}
