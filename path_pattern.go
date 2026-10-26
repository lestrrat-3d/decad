package decad

import (
	"context"
	"errors"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/pathsample"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/lestrrat-go/option/v3"
)

// PathPatternOption configures [PathPatternCopies].
type PathPatternOption interface {
	option.Interface
	pathPatternOption()
}

type pathPatternOption struct{ option.Interface }

func (pathPatternOption) pathPatternOption() {}

type identPathFollow struct{}

// WithPathFollow turns each copy of [PathPatternCopies] to follow the path.
// Copy i is rotated about the path's start by the shortest turn that takes the
// path's direction of travel there onto its direction at that copy's point,
// then moved to the point. The body is taken to sit at the path's start with
// its own forward along the path. A path that comes back on itself, so that
// the direction at a point is exactly opposite the start's, is ErrDegenerate:
// that turn has no shortest axis.
func WithPathFollow() PathPatternOption {
	return pathPatternOption{option.New(identPathFollow{}, true)}
}

// PathPatternCopies repeats b along path and returns the count-1 extra copies
// as separate live bodies, leaving b live (agent-docs/path-pattern-design.md).
// Instance 0 is b, taken to sit at the path's start. Instance i is a [Copy] of
// b moved by the vector from the path's start to the point i/(count-1) of the
// way along the path's length, so the last instance sits at the path's end.
// Copies keep the orientation of b unless [WithPathFollow] turns them to follow
// the path.
//
// The path is the sweep [Path] of straight [LineTo] and circular
// [ArcThrough] segments. Points are spaced by arc length, not by segment.
// Join the copies with the base through one [Union], as with [PatternCopies].
//
// b MUST be live; path MUST NOT be nil. count is at least 2, and a smaller
// count is ErrDegenerate. A path segment with no length or no circle is
// ErrDegenerate. A failed or canceled call leaves b live and retires every copy it
// had already made, so no copy outlives the call. Each copy is a Copy, so it
// carries Copy's own refusals.
func PathPatternCopies(ctx context.Context, b *Body, path *Path, count int, opts ...PathPatternOption) (result []*Body, retErr error) {
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
	if count < 2 {
		return nil, fmt.Errorf(`%w: a path pattern needs a count of at least 2, got %d`, ErrDegenerate, count)
	}
	if err := documentHandle(b.n.Doc).requireLive(b); err != nil {
		return nil, err
	}
	follow := false
	for _, o := range opts {
		if o == nil {
			return nil, fmt.Errorf(`%w: a nil option names nothing to apply`, ErrDegenerate)
		}
		if _, ok := o.Ident().(identPathFollow); ok {
			follow = true
		}
	}
	segments := path.Segments()
	samples := make([]pathsample.Segment, len(segments))
	for i, seg := range segments {
		switch s := seg.(type) {
		case LineTo:
			samples[i] = pathsample.Segment{End: s.End}
		case ArcThrough:
			samples[i] = pathsample.Segment{Arc: true, Through: s.Through, End: s.End}
		default:
			return nil, fmt.Errorf(`%w: unknown path segment %T`, ErrDegenerate, seg)
		}
	}
	points, err := pathsample.SampleStations(path.Start(), samples, count)
	if errors.Is(err, pathsample.ErrDegenerate) {
		return nil, fmt.Errorf(`%w: %s`, ErrDegenerate, err)
	}
	if err != nil {
		return nil, err
	}
	copies := make([]*Body, 0, count-1)
	for _, st := range points[1:] {
		t, err := pathStep(points[0], st, follow)
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
// station's point to st's, after the turn about the first point that takes the
// first direction onto st's when follow is set.
func pathStep(first, st pathsample.Station, follow bool) (r3.Transform, error) {
	move, err := r3.Translation(st.Point.Sub(first.Point))
	if err != nil {
		return r3.Transform{}, fmt.Errorf(`%w: a pattern step is not finite: %s`, ErrNotFinite, err)
	}
	if !follow {
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
	turn, err := r3.RotationAround(first.Point, axis, units.Radians(angle))
	if err != nil {
		return r3.Transform{}, fmt.Errorf(`%w: a pattern turn is not finite: %s`, ErrNotFinite, err)
	}
	placed, err := turn.Then(move)
	if err != nil {
		return r3.Transform{}, fmt.Errorf(`%w: a pattern step is not finite: %s`, ErrNotFinite, err)
	}
	return placed, nil
}
