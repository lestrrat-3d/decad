package decad

import (
	"fmt"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sweepinput"
	"github.com/lestrrat-3d/r3"
)

// PathSegment is one directed segment in a [Path]. The set is sealed to the
// segment kinds defined by decad.
type PathSegment interface {
	pathSegment()
}

// LineTo joins the preceding path point to End by one straight segment.
type LineTo struct {
	End r3.Vec
}

// ArcThrough joins the preceding path point to End by the unique circular arc
// through Through. Start, Through, and End must be distinct and non-collinear.
type ArcThrough struct {
	Through r3.Vec
	End     r3.Vec
}

// ArcAround follows the positive rotation about Axis from the preceding point
// to End on the circle centered at Center. Axis is an oriented direction. The
// endpoint radii must be exactly equal, nonzero and perpendicular to Axis. The
// endpoints must differ; one segment cannot describe a full turn.
type ArcAround struct {
	Center, Axis, End r3.Vec
}

func (LineTo) pathSegment()     {}
func (ArcThrough) pathSegment() {}
func (ArcAround) pathSegment()  {}

// Path is an immutable ordered spatial curve. Its coordinates are in
// millimetres.
type Path struct {
	// n is the recorded path internal packages read (internal/sweepinput).
	n sweepinput.Path
}

// NewPath records an ordered spatial path beginning at start. It requires at
// least one non-degenerate segment and copies every segment value.
func NewPath(start r3.Vec, segments ...PathSegment) (*Path, error) {
	if !proofbound.FiniteVec(start) {
		return nil, fmt.Errorf(`%w: a path start must be finite`, ErrNotFinite)
	}
	if len(segments) == 0 {
		return nil, fmt.Errorf(`%w: a path requires at least one segment`, ErrDegenerate)
	}

	owned := make([]sweepinput.Segment, len(segments))
	records := make([]sweepinput.PathRecord, len(segments))
	current := start
	for i, raw := range segments {
		segment, err := ownPathSegment(raw)
		if err != nil {
			return nil, fmt.Errorf(`path segment %d: %w`, i, err)
		}
		var input sweepinput.Segment
		switch segment := segment.(type) {
		case LineTo:
			input = sweepinput.Segment{End: segment.End}
		case ArcThrough:
			input = sweepinput.Segment{End: segment.End, Through: segment.Through, Arc: true}
		case ArcAround:
			input = sweepinput.Segment{End: segment.End, Center: segment.Center, Axis: segment.Axis, Arc: true, Around: true}
		}
		records[i], err = sweepinput.RecordSegment(current, input, i)
		if err != nil {
			return nil, err
		}
		current = records[i].End
		owned[i] = input
	}
	return &Path{n: sweepinput.Path{Start: start, End: current, Segments: owned, Records: records}}, nil
}

// Start returns the path's recorded initial point.
func (p *Path) Start() r3.Vec { return p.n.Start }

// End returns the path's recorded final point.
func (p *Path) End() r3.Vec { return p.n.End }

// Segments returns a copy of the path's ordered segment values.
func (p *Path) Segments() []PathSegment {
	return pathSegments(&p.n)
}

// pathSegments rebuilds the public segment values of a recorded path, in order:
// the values Segments returns, nil when it has none.
func pathSegments(p *sweepinput.Path) []PathSegment {
	if len(p.Segments) == 0 {
		return nil
	}
	out := make([]PathSegment, len(p.Segments))
	for i, s := range p.Segments {
		if s.Around {
			out[i] = ArcAround{Center: s.Center, Axis: s.Axis, End: s.End}
			continue
		}
		if s.Arc {
			out[i] = ArcThrough{Through: s.Through, End: s.End}
			continue
		}
		out[i] = LineTo{End: s.End}
	}
	return out
}

// pathNode is the recorded path p holds, or nil for a nil p, so a callee's own
// nil check still answers.
func pathNode(p *Path) *sweepinput.Path {
	if p == nil {
		return nil
	}
	return &p.n
}

// ownPathSegment rejects types outside decad's sealed set and copies pointer
// inputs into value segments, so caller mutation cannot change a Path.
func ownPathSegment(raw PathSegment) (PathSegment, error) {
	if raw == nil {
		return nil, fmt.Errorf(`%w: a nil path segment names no geometry`, ErrDegenerate)
	}

	switch segment := raw.(type) {
	case LineTo:
		return segment, nil
	case *LineTo:
		if segment == nil {
			return nil, fmt.Errorf(`%w: a nil line segment names no geometry`, ErrDegenerate)
		}
		return *segment, nil
	case ArcThrough:
		return segment, nil
	case *ArcThrough:
		if segment == nil {
			return nil, fmt.Errorf(`%w: a nil arc segment names no geometry`, ErrDegenerate)
		}
		return *segment, nil
	case ArcAround:
		return segment, nil
	case *ArcAround:
		if segment == nil {
			return nil, fmt.Errorf(`%w: a nil arc segment names no geometry`, ErrDegenerate)
		}
		return *segment, nil
	default:
		return nil, fmt.Errorf(`%w: %T is not a decad path segment`, ErrDegenerate, raw)
	}
}
