package stepbrep

import (
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/stepread"
	"github.com/lestrrat-3d/r3"
)

// chordTolerance is how far, in millimetres, a circular edge may lie from the
// polygon that stands for it. An arc of radius R is cut into steps of at most
// 2·acos(1 − chordTolerance/R) radians, a function of the radius alone, so two
// edges of one radius and sweep take the same number of steps.
const chordTolerance = 0.01

// frame is a right-handed frame of a placement: origin and the unit axes.
type frame struct {
	o, x, y, z r3.Vec
}

// placement reads an AXIS2_PLACEMENT_3D into a frame, the origin in millimetres.
func (r *reader) placement(v stepread.Value) (frame, error) {
	e, err := r.entity(v, "placement")
	if err != nil {
		return frame{}, err
	}
	if e.Name != "AXIS2_PLACEMENT_3D" || len(e.Params) < 4 {
		return frame{}, fmt.Errorf("%w: #%d is not an AXIS2_PLACEMENT_3D", ErrMalformed, e.ID)
	}
	o, err := r.point(e.Params[1])
	if err != nil {
		return frame{}, err
	}
	direction := func(v stepread.Value, fallback r3.Vec) (r3.Vec, error) {
		if _, none := v.(stepread.Null); none {
			return fallback, nil
		}
		d, err := r.entity(v, "direction")
		if err != nil {
			return r3.Vec{}, err
		}
		if d.Name != "DIRECTION" || len(d.Params) < 2 {
			return r3.Vec{}, fmt.Errorf("%w: #%d is not a DIRECTION", ErrMalformed, d.ID)
		}
		return r.coords(d.Params[1], d.ID, 1)
	}
	axis, err := direction(e.Params[2], r3.Vec{Z: 1})
	if err != nil {
		return frame{}, err
	}
	z, ok := axis.Normalize()
	if !ok {
		return frame{}, fmt.Errorf("%w: a placement has a zero axis", ErrMalformed)
	}
	seed, err := direction(e.Params[3], r3.Vec{})
	if err != nil {
		return frame{}, err
	}
	// The reference direction is made square to the axis, and falls back to a
	// perpendicular of it when it is missing or parallel.
	x, ok := seed.Sub(z.Scale(seed.Dot(z))).Normalize()
	if !ok {
		if x, ok = perpendicular(z); !ok {
			return frame{}, fmt.Errorf("%w: a placement has no reference direction", ErrMalformed)
		}
	}
	return frame{o: o, x: x, y: z.Cross(x), z: z}, nil
}

// angle is the angle of p about the frame's axis, from its x axis.
func (f frame) angle(p r3.Vec) float64 {
	d := p.Sub(f.o)
	return math.Atan2(d.Dot(f.y), d.Dot(f.x))
}

// edgeSamples returns the points of an EDGE_CURVE from its start vertex to its
// end vertex: two for a line, and for a circle the start, the samples between
// and the end.
func (r *reader) edgeSamples(edge stepread.Entity) ([]r3.Vec, error) {
	if s, ok := r.samples[edge.ID]; ok {
		return s, nil
	}
	if edge.Name != "EDGE_CURVE" || len(edge.Params) < 5 {
		return nil, fmt.Errorf("%w: #%d is not an EDGE_CURVE", ErrMalformed, edge.ID)
	}
	vertexPoint := func(v stepread.Value) (r3.Vec, error) {
		e, err := r.entity(v, "vertex")
		if err != nil {
			return r3.Vec{}, err
		}
		if e.Name != "VERTEX_POINT" || len(e.Params) < 2 {
			return r3.Vec{}, fmt.Errorf("%w: #%d is not a VERTEX_POINT", ErrMalformed, e.ID)
		}
		return r.point(e.Params[1])
	}
	start, err := vertexPoint(edge.Params[1])
	if err != nil {
		return nil, err
	}
	end, err := vertexPoint(edge.Params[2])
	if err != nil {
		return nil, err
	}
	curve, err := r.entity(edge.Params[3], "curve")
	if err != nil {
		return nil, err
	}
	var samples []r3.Vec
	switch curve.Name {
	case "LINE":
		samples = []r3.Vec{start, end}
	case "CIRCLE":
		sameSense, _ := edge.Params[4].(stepread.Enum)
		if samples, err = r.circleSamples(curve, start, end, sameSense != "F"); err != nil {
			return nil, err
		}
	default:
		return nil, fmt.Errorf("%w: an edge is a %s, and only lines and circles are read", decaderr.ErrUnsupported, nameOf(curve))
	}
	r.samples[edge.ID] = samples
	return samples, nil
}

// circleSamples cuts the arc of a CIRCLE from start to end into chords. The arc
// runs counter-clockwise about the circle's axis when sameSense is set and
// clockwise when not, and is a full turn when its ends coincide.
func (r *reader) circleSamples(circle stepread.Entity, start, end r3.Vec, sameSense bool) ([]r3.Vec, error) {
	if len(circle.Params) < 3 {
		return nil, fmt.Errorf("%w: #%d is not a CIRCLE", ErrMalformed, circle.ID)
	}
	f, err := r.placement(circle.Params[1])
	if err != nil {
		return nil, err
	}
	radius, ok := circle.Params[2].(stepread.Number)
	if !ok || !(float64(radius) > 0) {
		return nil, fmt.Errorf("%w: a circle has no positive radius", ErrMalformed)
	}
	rad := float64(radius) * r.scale
	theta0, theta1 := f.angle(start), f.angle(end)
	ccw := math.Mod(theta1-theta0+4*math.Pi, 2*math.Pi)
	var sweep float64
	switch {
	case end.Sub(start).Len() < 1e-9*math.Max(1, rad):
		sweep = 2 * math.Pi
		if !sameSense {
			sweep = -2 * math.Pi
		}
	case sameSense:
		sweep = ccw
	default:
		sweep = -(2*math.Pi - ccw)
	}
	step := math.Pi
	if chordTolerance < rad {
		step = 2 * math.Acos(1-chordTolerance/rad)
	}
	n := max(int(math.Ceil(math.Abs(sweep)/step)), 1)
	samples := make([]r3.Vec, n+1)
	samples[0], samples[n] = start, end
	for k := 1; k < n; k++ {
		t := theta0 + sweep*float64(k)/float64(n)
		samples[k] = f.o.Add(f.x.Scale(rad * math.Cos(t))).Add(f.y.Scale(rad * math.Sin(t)))
	}
	return samples, nil
}
