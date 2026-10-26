package decad

import (
	"context"
	"errors"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/pathsample"
	"github.com/lestrrat-3d/r3"
)

// PathPatternCopies repeats b along path and returns the count-1 extra copies
// as separate live bodies, leaving b live (agent-docs/path-pattern-design.md).
// Instance 0 is b, taken to sit at the path's start. Instance i is a [Copy] of
// b moved by the vector from the path's start to the point i/(count-1) of the
// way along the path's length, so the last instance sits at the path's end.
// Copies keep the orientation of b; none turns to follow the path.
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
func PathPatternCopies(ctx context.Context, b *Body, path *Path, count int) (result []*Body, retErr error) {
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
	points, err := pathsample.Sample(path.Start(), samples, count)
	if errors.Is(err, pathsample.ErrDegenerate) {
		return nil, fmt.Errorf(`%w: %s`, ErrDegenerate, err)
	}
	if err != nil {
		return nil, err
	}
	copies := make([]*Body, 0, count-1)
	for _, p := range points[1:] {
		t, err := r3.Translation(p.Sub(points[0]))
		if err != nil {
			removeTools(copies)
			return nil, fmt.Errorf(`%w: a pattern step is not finite: %s`, ErrNotFinite, err)
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
