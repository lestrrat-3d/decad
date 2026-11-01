// Package pathsample places points at equal arc length along a path of
// straight and circular segments (agent-docs/path-pattern-design.md). It works
// on plain millimetre vectors and touches no body or document.
package pathsample

import (
	"errors"
	"math"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// ErrDegenerate reports a segment that has no length or no circle.
var ErrDegenerate = errors.New("pathsample: degenerate segment")

// Segment is one piece of a path. A segment with Arc false is the straight
// line from the previous end to End. An arc uses either Through or the
// explicit Center, Axis and held Sweep angle when Around is true.
type Segment struct {
	Arc, Around                bool
	Through, Center, Axis, End r3.Vec
	Sweep                      float64
}

// piece is a segment resolved to what a point at a length needs.
type piece struct {
	start  r3.Vec
	length float64
	at     func(s float64) r3.Vec
	// dir is the unit direction of travel at the length s.
	dir func(s float64) r3.Vec
}

// Station is a point on the path and the unit direction of travel there.
type Station struct {
	Point, Tangent r3.Vec
}

// Sample returns count points, the first at start and the last at the path's
// end, spaced at equal arc length along the path. count is at least 2.
func Sample(start r3.Vec, segments []Segment, count int) ([]r3.Vec, error) {
	stations, err := SampleStations(start, segments, count)
	if err != nil {
		return nil, err
	}
	points := make([]r3.Vec, len(stations))
	for i, st := range stations {
		points[i] = st.Point
	}
	return points, nil
}

// SampleStations is Sample with the direction of travel at each point. A point
// where two segments meet takes the direction of the segment that holds it, and
// the last point takes the direction at the path's end.
func SampleStations(start r3.Vec, segments []Segment, count int) ([]Station, error) {
	pieces, total, end, err := resolveAll(start, segments)
	if err != nil {
		return nil, err
	}
	lengths := make([]float64, count)
	for i := range lengths {
		lengths[i] = total * float64(i) / float64(count-1)
	}
	stations := stationsAt(pieces, lengths)
	stations[0].Point = start
	stations[count-1].Point = end
	return stations, nil
}

// spacingSlack is the relative allowance that lets a spacing which divides the
// path's length, up to rounding, place a station at the path's end.
const spacingSlack = 1e-12

// SampleSpaced returns the points at 0, spacing, 2·spacing and so on along the
// path, as many as lie within its length, with the direction of travel at each.
// The first is at start. spacing is positive and finite; a spacing longer than
// the path leaves one point, which the caller refuses.
func SampleSpaced(start r3.Vec, segments []Segment, spacing float64) ([]Station, error) {
	if !(spacing > 0) || math.IsInf(spacing, 0) {
		return nil, ErrDegenerate
	}
	pieces, total, end, err := resolveAll(start, segments)
	if err != nil {
		return nil, err
	}
	steps := math.Floor(total/spacing*(1+spacingSlack)) + 1
	if steps > maxStations {
		return nil, ErrTooMany
	}
	lengths := make([]float64, int(steps))
	for i := range lengths {
		lengths[i] = min(float64(i)*spacing, total)
	}
	stations := stationsAt(pieces, lengths)
	stations[0].Point = start
	if last := len(stations) - 1; lengths[last] == total {
		stations[last].Point = end
	}
	return stations, nil
}

// maxStations bounds the points a spacing may produce.
const maxStations = 100000

// ErrTooMany reports a spacing that would place more than 100000 points.
var ErrTooMany = errors.New("pathsample: too many points")

// resolveAll resolves every segment and returns the pieces, the path's total
// length and its end point.
func resolveAll(start r3.Vec, segments []Segment) ([]piece, float64, r3.Vec, error) {
	pieces := make([]piece, 0, len(segments))
	total := 0.0
	prev := start
	for _, seg := range segments {
		p, err := resolve(prev, seg)
		if err != nil {
			return nil, 0, r3.Vec{}, err
		}
		pieces = append(pieces, p)
		total += p.length
		if math.IsInf(total, 0) || math.IsNaN(total) {
			return nil, 0, r3.Vec{}, ErrDegenerate
		}
		prev = seg.End
	}
	return pieces, total, prev, nil
}

// stationsAt returns the station at each of the ascending lengths.
func stationsAt(pieces []piece, lengths []float64) []Station {
	stations := make([]Station, len(lengths))
	index, offset := 0, 0.0
	for i, want := range lengths {
		for index < len(pieces)-1 && want > offset+pieces[index].length {
			offset += pieces[index].length
			index++
		}
		p := pieces[index]
		along := min(max(want-offset, 0), p.length)
		stations[i] = Station{Point: p.at(along), Tangent: p.dir(along)}
	}
	return stations
}

func resolve(start r3.Vec, seg Segment) (piece, error) {
	if !seg.Arc {
		span := seg.End.Sub(start)
		length := span.Len()
		if length == 0 {
			return piece{}, ErrDegenerate
		}
		return piece{start: start, length: length, at: func(s float64) r3.Vec {
			return start.Add(span.Scale(s / length))
		}, dir: func(float64) r3.Vec { return span.Scale(1 / length) }}, nil
	}
	if seg.Around {
		return resolveAround(start, seg.Center, seg.Axis, seg.Sweep)
	}
	return resolveArc(start, seg.Through, seg.End)
}

// resolveAround samples the explicit carrier without constructing a through point.
func resolveAround(start, center, direction r3.Vec, sweep float64) (piece, error) {
	radiusVector := start.Sub(center)
	radius := radiusVector.Len()
	axis, ok := direction.Normalize()
	if !ok || radius <= 0 || math.IsInf(radius, 0) || math.IsNaN(radius) {
		return piece{}, ErrDegenerate
	}
	e1 := radiusVector.Scale(1 / radius)
	e2 := axis.Cross(e1)
	if !proofbound.FiniteVec(e1) || !proofbound.FiniteVec(e2) {
		return piece{}, ErrDegenerate
	}
	length := sweep * radius
	if !(length > 0) || math.IsInf(length, 0) || math.IsNaN(length) {
		return piece{}, ErrDegenerate
	}
	return piece{start: start, length: length, at: func(s float64) r3.Vec {
		theta := s / radius
		return center.Add(e1.Scale(radius * math.Cos(theta))).Add(e2.Scale(radius * math.Sin(theta)))
	}, dir: func(s float64) r3.Vec {
		theta := s / radius
		return e2.Scale(math.Cos(theta)).Sub(e1.Scale(math.Sin(theta)))
	}}, nil
}

// resolveArc builds the arc from a through b to c. The centre is the point in
// the plane of the three that is equally far from all of them.
func resolveArc(a, b, c r3.Vec) (piece, error) {
	u := b.Sub(a)
	v := c.Sub(a)
	normal := u.Cross(v)
	nn := normal.Dot(normal)
	if nn == 0 {
		return piece{}, ErrDegenerate
	}
	offset := normal.Cross(u).Scale(v.Dot(v)).Add(v.Cross(normal).Scale(u.Dot(u))).Scale(1 / (2 * nn))
	centre := a.Add(offset)
	radius := offset.Len()
	axis, ok := normal.Normalize()
	if !ok {
		return piece{}, ErrDegenerate
	}
	e1 := a.Sub(centre).Scale(1 / radius)
	e2 := axis.Cross(e1)
	angleOf := func(p r3.Vec) float64 {
		d := p.Sub(centre)
		return math.Atan2(d.Dot(e2), d.Dot(e1))
	}
	// The axis is the normal of (a, b, c) in that order, so a to b to c turns
	// counter-clockwise about it and the sweep from angle 0 at a to c is the
	// angle of c taken into [0, 2π).
	sweep := angleOf(c)
	if sweep < 0 {
		sweep += 2 * math.Pi
	}
	return piece{start: a, length: sweep * radius, at: func(s float64) r3.Vec {
		theta := s / radius
		return centre.Add(e1.Scale(radius * math.Cos(theta))).Add(e2.Scale(radius * math.Sin(theta)))
	}, dir: func(s float64) r3.Vec {
		theta := s / radius
		return e2.Scale(math.Cos(theta)).Sub(e1.Scale(math.Sin(theta)))
	}}, nil
}

// turnTolerance is the sine of the angle below which two directions count as
// parallel (or opposed) for Turn.
const turnTolerance = 1e-12

// Turn returns the shortest rotation that takes the unit direction from onto
// the unit direction to: its axis, perpendicular to both, and its angle in
// radians in [0, π). Equal directions turn by angle zero about the zero axis.
// Opposite directions have no shortest turn and are ErrDegenerate.
func Turn(from, to r3.Vec) (axis r3.Vec, angle float64, err error) {
	cross := from.Cross(to)
	sin, cos := cross.Len(), from.Dot(to)
	if sin <= turnTolerance {
		if cos < 0 {
			return r3.Vec{}, 0, ErrDegenerate
		}
		return r3.Vec{}, 0, nil
	}
	return cross.Scale(1 / sin), math.Atan2(sin, cos), nil
}
