package decad

import (
	"context"
	"errors"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/pathsample"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/lestrrat-go/option/v3"
)

// PathPatternOption configures [PathPattern].
type PathPatternOption interface {
	option.Interface
	pathPatternOption()
}

type pathPatternOption struct{ option.Interface }

func (pathPatternOption) pathPatternOption() {}

type identPathFollow struct{}
type identPathPivot struct{}
type identPathCount struct{}   // value: int
type identPathSpacing struct{} // value: units.Value

// WithPathFollow turns each copy of [PathPattern] to follow the path.
// Copy i is rotated about the path's start by the shortest turn that takes the
// path's direction of travel there onto its direction at that copy's point,
// then moved to the point. The body is taken to sit at the path's start with
// its own forward along the path. A path that comes back on itself, so that
// the direction at a point is exactly opposite the start's, is ErrDegenerate:
// that turn has no shortest axis.
func WithPathFollow() PathPatternOption {
	return pathPatternOption{option.New(identPathFollow{}, true)}
}

// WithPathPivot turns each copy about pivot, a point in millimetres, instead of
// about the path's start. It sets the point that [WithPathFollow]'s turn
// leaves in place before the copy is moved to its station, so a body that sits
// away from the path's start, or a pivot at the body's own centre, swings
// about that point. The last of several pivots wins. A pivot with no
// [WithPathFollow] has no turn to centre and is ErrDegenerate, and a non-finite
// pivot is ErrNotFinite.
func WithPathPivot(pivot r3.Vec) PathPatternOption {
	return pathPatternOption{option.New(identPathPivot{}, pivot)}
}

// WithPathCount makes [PathPattern] place n instances, b and n-1 copies, at
// equal arc length along the path. The first is at the path's start and the
// last at its end (agent-docs/path-pattern-design.md §1). n is at least 2, and
// a smaller n is ErrDegenerate. A call takes exactly one of WithPathCount and
// [WithPathSpacing].
func WithPathCount(n int) PathPatternOption {
	return pathPatternOption{option.New(identPathCount{}, n)}
}

// WithPathSpacing makes [PathPattern] place instances spacing apart by arc
// length. Instance i sits i·spacing along the path, for every i within its
// length, and a spacing that divides the length puts the last instance at the
// path's end (agent-docs/path-pattern-design.md §6). spacing is a positive
// length no longer than the path. A wrong unit kind is ErrUnitKind and a
// negative spacing is ErrNegativeMagnitude. A zero spacing, one that leaves no
// copy, and one that would place over 100000 copies are ErrDegenerate. A call
// takes exactly one of WithPathSpacing and [WithPathCount].
func WithPathSpacing(spacing units.Value) PathPatternOption {
	return pathPatternOption{option.New(identPathSpacing{}, spacing)}
}

// PathPattern repeats b along path and returns the extra copies as separate
// live bodies, leaving b live (agent-docs/path-pattern-design.md). Instance 0
// is b, taken to sit at the path's start. [WithPathCount] or [WithPathSpacing]
// chooses the other instances' points. Exactly one of them is required, and
// neither, both, or either given twice is ErrDegenerate. Instance i is a [Copy]
// of b moved by the vector from the path's start to its point. Copies keep the
// orientation of b unless [WithPathFollow] turns them to follow the path.
//
// The path is the sweep [Path] of straight [LineTo] and circular
// [ArcThrough] and [ArcAround] segments. Points are spaced by arc length, not by segment.
// Join the copies with the base through one [Union], as with [PatternCopies].
//
// b MUST be live; path MUST NOT be nil. A nil context, a nil option, and a path
// segment with no length or no circle are ErrDegenerate. A failed or canceled
// call leaves b live and retires every copy it had already made, so no copy
// outlives the call. Each copy is a Copy, so it carries Copy's own refusals.
func PathPattern(ctx context.Context, b *Body, path *Path, opts ...PathPatternOption) (result []*Body, retErr error) {
	defer func() { retErr = operationError(OperationCopy, retErr) }()
	if ctx == nil {
		return nil, fmt.Errorf(`%w: a nil context cannot control a pattern`, ErrDegenerate)
	}
	if b == nil || b.n.Doc == nil {
		return nil, fmt.Errorf(`%w: the body belongs to no document`, ErrDegenerate)
	}
	if path == nil {
		return nil, fmt.Errorf(`%w: a path pattern needs a path`, ErrDegenerate)
	}
	settings, err := foldPathPatternOptions(opts)
	if err != nil {
		return nil, err
	}
	if err := documentHandle(b.n.Doc).requireLive(b); err != nil {
		return nil, err
	}
	samples, err := patternSegments(path)
	if err != nil {
		return nil, err
	}
	var stations []pathsample.Station
	if settings.hasCount {
		stations, err = pathsample.SampleStations(path.Start(), samples, settings.count)
		if err != nil {
			return nil, pathSampleError(err)
		}
		return pathCopies(ctx, b, stations, settings)
	}
	stations, err = pathsample.SampleSpaced(path.Start(), samples, settings.spacing)
	if err != nil {
		return nil, pathSampleError(err)
	}
	if len(stations) < 2 {
		return nil, fmt.Errorf(`%w: a spacing of %g mm leaves no copy along the path`, ErrDegenerate, settings.spacing)
	}
	return pathCopies(ctx, b, stations, settings)
}

// PathPatternCopies is [PathPattern] with [WithPathCount]; removed before this PR merges.
func PathPatternCopies(ctx context.Context, b *Body, path *Path, count int, opts ...PathPatternOption) ([]*Body, error) {
	return PathPattern(ctx, b, path, append([]PathPatternOption{WithPathCount(count)}, opts...)...)
}

// PathPatternSpaced is [PathPattern] with [WithPathSpacing]; removed before this PR merges.
func PathPatternSpaced(ctx context.Context, b *Body, path *Path, spacing units.Value, opts ...PathPatternOption) ([]*Body, error) {
	return PathPattern(ctx, b, path, append([]PathPatternOption{WithPathSpacing(spacing)}, opts...)...)
}

// pathPatternSettings are the options of a path pattern read.
type pathPatternSettings struct {
	count      int // instances, set when hasCount
	hasCount   bool
	spacing    float64 // millimetres, set when hasSpacing
	hasSpacing bool
	follow     bool
	pivot      r3.Vec
	hasPivot   bool
}

// foldPathPatternOptions reads and validates the options of a path pattern.
func foldPathPatternOptions(opts []PathPatternOption) (pathPatternSettings, error) {
	var out pathPatternSettings
	var spacing units.Value
	for _, o := range opts {
		if o == nil {
			return out, fmt.Errorf(`%w: a nil option names nothing to apply`, ErrDegenerate)
		}
		switch o.Ident().(type) {
		case identPathCount:
			if out.hasCount {
				return out, fmt.Errorf(`%w: WithPathCount was passed more than once`, ErrDegenerate)
			}
			out.count, _ = option.Get[int](o)
			out.hasCount = true
		case identPathSpacing:
			if out.hasSpacing {
				return out, fmt.Errorf(`%w: WithPathSpacing was passed more than once`, ErrDegenerate)
			}
			spacing, _ = option.Get[units.Value](o)
			out.hasSpacing = true
		case identPathFollow:
			out.follow = true
		case identPathPivot:
			out.pivot, _ = option.Get[r3.Vec](o)
			out.hasPivot = true
		}
	}
	switch {
	case out.hasCount && out.hasSpacing:
		return out, fmt.Errorf(`%w: a path pattern takes WithPathCount or WithPathSpacing, not both`, ErrDegenerate)
	case out.hasCount && out.count < 2:
		return out, fmt.Errorf(`%w: a path pattern needs a count of at least 2, got %d`, ErrDegenerate, out.count)
	case out.hasSpacing:
		step, err := sectionrecord.MagnitudeIn(spacing, units.Length, units.Millimeter, "the path pattern spacing")
		if err != nil {
			return out, err
		}
		out.spacing = step
	case !out.hasCount:
		return out, fmt.Errorf(`%w: a path pattern needs WithPathCount or WithPathSpacing to place its copies`, ErrDegenerate)
	}
	if out.hasPivot && !proofbound.FiniteVec(out.pivot) {
		return out, fmt.Errorf(`%w: the path pattern pivot is not finite`, ErrNotFinite)
	}
	if out.hasPivot && !out.follow {
		return out, fmt.Errorf(`%w: a path pattern pivot needs WithPathFollow, which is what turns the copies`, ErrDegenerate)
	}
	return out, nil
}

// patternSegments converts the segments of path for the sampler.
func patternSegments(path *Path) ([]pathsample.Segment, error) {
	segments := path.Segments()
	samples := make([]pathsample.Segment, len(segments))
	for i, seg := range segments {
		switch s := seg.(type) {
		case LineTo:
			samples[i] = pathsample.Segment{End: s.End}
		case ArcThrough:
			samples[i] = pathsample.Segment{Arc: true, Through: s.Through, End: s.End}
		case ArcAround:
			samples[i] = pathsample.Segment{Arc: true, Around: true, Center: s.Center, Axis: s.Axis,
				End: s.End, Sweep: path.n.Records[i].ArcPhi}
		default:
			return nil, fmt.Errorf(`%w: unknown path segment %T`, ErrDegenerate, seg)
		}
	}
	return samples, nil
}

// pathSampleError maps a sampler refusal to the public error it is.
func pathSampleError(err error) error {
	if errors.Is(err, pathsample.ErrDegenerate) || errors.Is(err, pathsample.ErrTooMany) {
		return fmt.Errorf(`%w: %s`, ErrDegenerate, err)
	}
	return err
}

// pathCopies makes a Copy of b at each station but the first. A failure
// retires the copies already made.
func pathCopies(ctx context.Context, b *Body, stations []pathsample.Station, settings pathPatternSettings) ([]*Body, error) {
	copies := make([]*Body, 0, len(stations)-1)
	for _, st := range stations[1:] {
		t, err := pathStep(stations[0], st, settings)
		if err != nil {
			removeTools(copies)
			return nil, err
		}
		c, err := Copy(ctx, b, WithCopyPlacement(t))
		if err != nil {
			removeTools(copies)
			return nil, err
		}
		copies = append(copies, c)
	}
	return copies, nil
}

// pathStep is the placement of the copy at station st: the move from the first
// station's point to st's, after the turn about the pivot (the first point unless
// one is set) that takes the first direction onto st's when follow is set.
func pathStep(first, st pathsample.Station, settings pathPatternSettings) (r3.Transform, error) {
	move, err := r3.Translation(st.Point.Sub(first.Point))
	if err != nil {
		return r3.Transform{}, fmt.Errorf(`%w: a pattern step is not finite: %s`, ErrNotFinite, err)
	}
	if !settings.follow {
		return move, nil
	}
	axis, angle, err := pathsample.Turn(first.Tangent, st.Tangent)
	if errors.Is(err, pathsample.ErrDegenerate) {
		return r3.Transform{}, fmt.Errorf(`%w: the path turns back on itself, so a copy has no shortest turn to follow it`, ErrDegenerate)
	}
	if err != nil {
		return r3.Transform{}, err
	}
	if angle == 0 {
		return move, nil
	}
	pivot := first.Point
	if settings.hasPivot {
		pivot = settings.pivot
	}
	turn, err := r3.RotationAround(pivot, axis, units.Radians(angle))
	if err != nil {
		return r3.Transform{}, fmt.Errorf(`%w: a pattern turn is not finite: %s`, ErrNotFinite, err)
	}
	placed, err := turn.Then(move)
	if err != nil {
		return r3.Transform{}, fmt.Errorf(`%w: a pattern step is not finite: %s`, ErrNotFinite, err)
	}
	return placed, nil
}
