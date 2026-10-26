// Package pathsample places points at equal arc length along a path of
// straight and circular segments (agent-docs/path-pattern-design.md). It works
// on plain millimetre vectors and touches no body or document.
package pathsample

import (
	"errors"
	"math"

	"github.com/lestrrat-3d/r3"
)

// ErrDegenerate reports a segment that has no length or no circle.
var ErrDegenerate = errors.New("pathsample: degenerate segment")

// Segment is one piece of a path. A segment with Arc false is the straight
// line from the previous end to End. With Arc true it is the circular arc from
// the previous end through Through to End.
type Segment struct {
	Arc     bool
	Through r3.Vec
	End     r3.Vec
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
	pieces := make([]piece, 0, len(segments))
	total := 0.0
	prev := start
	for _, seg := range segments {
		p, err := resolve(prev, seg)
		if err != nil {
			return nil, err
		}
		pieces = append(pieces, p)
		total += p.length
		prev = seg.End
	}
	points := make([]Station, count)
	piece, offset := 0, 0.0
	for i := range points {
		want := total * float64(i) / float64(count-1)
		for piece < len(pieces)-1 && want > offset+pieces[piece].length {
			offset += pieces[piece].length
			piece++
		}
		p := pieces[piece]
		along := min(max(want-offset, 0), p.length)
		points[i] = Station{Point: p.at(along), Tangent: p.dir(along)}
	}
	points[0].Point = start
	points[count-1].Point = prev
	return points, nil
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
	return resolveArc(start, seg.Through, seg.End)
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
