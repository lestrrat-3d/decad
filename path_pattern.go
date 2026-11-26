package decad

import (
	"context"
	"errors"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/opcommand"
	"github.com/lestrrat-3d/decad/internal/pathsample"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// PathStations is where [PathPattern] puts its instances along the path: a
// [PathCount] or a [PathSpacing]. The set is sealed, so a call names exactly
// one of them (agent-docs/command-api-design.md rule M4).
type PathStations interface{ pathStations() }

// PathCount places Count instances, b and Count-1 copies, at equal arc length
// along the path. The first is at the path's start and the last at its end
// (agent-docs/path-pattern-design.md §1). Count is at least 2, and a smaller
// Count is ErrDegenerate.
type PathCount struct {
	Count int
}

// PathSpacing places instances Spacing apart by arc length. Instance i sits
// i·Spacing along the path, for every i within its length, and a Spacing that
// divides the length puts the last instance at the path's end
// (agent-docs/path-pattern-design.md §6). Spacing is a positive length no
// longer than the path. A wrong unit kind is ErrUnitKind and a negative
// Spacing is ErrNegativeMagnitude. A zero Spacing, one that leaves no copy, and
// one that would place over 100000 copies are ErrDegenerate.
type PathSpacing struct {
	Spacing units.Value
}

func (PathCount) pathStations()   {}
func (PathSpacing) pathStations() {}

// copyPathStations copies stations at the call that stores it
// (agent-docs/command-api-design.md rule A3): a pointer to a variant is
// replaced by a pointer to a copy of its value.
func copyPathStations(stations PathStations) PathStations {
	switch s := stations.(type) {
	case *PathCount:
		if s != nil {
			cp := *s
			return &cp
		}
	case *PathSpacing:
		if s != nil {
			cp := *s
			return &cp
		}
	}
	return stations
}

// PathPattern returns the command that repeats b along path. Do returns the
// extra copies as separate live bodies, leaving b live
// (agent-docs/path-pattern-design.md). Instance 0 is b, taken to sit at the
// path's start. stations is a [PathCount] or a [PathSpacing] and chooses the
// other instances' points; a nil stations is ErrDegenerate. Instance i is a
// [Copy] of b moved by the vector from the path's start to its point. Copies
// keep the orientation of b unless [PathPatternCommand.Follow] or
// [PathPatternCommand.FollowAbout] turns them to follow the path.
//
// The path is the sweep [Path] of straight [LineTo] and circular
// [ArcThrough] and [ArcAround] segments. Points are spaced by arc length, not by segment.
// Join the copies with the base through one [Union], as with [PatternCopies].
//
// b MUST be live; path MUST NOT be nil. A nil context, a nil b or path, and a
// path segment with no length or no circle are ErrDegenerate. A failed or
// canceled call leaves the document as it found it: b live, no copy live, and
// no history entry or producer number added. Each copy is a Copy, so it
// carries Copy's own refusals. Every error carries [OperationCopy].
func PathPattern(b *Body, path *Path, stations PathStations) *PathPatternCommand {
	c := &PathPatternCommand{b: b, path: path, stations: copyPathStations(stations)}
	switch {
	case b == nil:
		c.state.Record("body", errNilBody)
	case path == nil:
		c.state.Record("path", errNilPath)
	}
	return c
}

// errNilPath is the refusal of a nil path, which the constructor records
// because the operation reports it right after the nil body.
var errNilPath = fmt.Errorf(`%w: a path pattern needs a path`, ErrDegenerate)

// PathPatternCommand repeats a body along a path. Build it with [PathPattern].
type PathPatternCommand struct {
	state    opcommand.State          // consumed flag and first recorded fault
	b        *Body                    // stored as given
	path     *Path                    // stored as given
	stations PathStations             // copied at PathPattern
	follow   opcommand.Slot[pathTurn] // set by Follow or FollowAbout
}

// pathTurn is what [PathPatternCommand.Follow] and
// [PathPatternCommand.FollowAbout] store: whether the copies turn and about
// which point.
type pathTurn struct {
	pivot    r3.Vec
	hasPivot bool
}

// Follow turns each copy to follow the path. Copy i is rotated about the
// path's start by the shortest turn that takes the path's direction of travel
// there onto its direction at that copy's point, then moved to the point. The
// body is taken to sit at the path's start with its own forward along the
// path. A path that comes back on itself, so that the direction at a point is
// exactly opposite the start's, is ErrDegenerate: that turn has no shortest
// axis. Follow and [PathPatternCommand.FollowAbout] write one slot, and the
// last call wins.
func (c *PathPatternCommand) Follow() *PathPatternCommand {
	c.follow.Set(pathTurn{}, nil)
	return c
}

// FollowAbout is [PathPatternCommand.Follow] with the turn about pivot, a point
// in millimetres, instead of about the path's start. It sets the point that
// the turn leaves in place before the copy is moved to its station, so a body
// that sits away from the path's start, or a pivot at the body's own centre,
// swings about that point. A non-finite pivot is ErrNotFinite, returned by Do
// naming FollowAbout as its subject. A second call to Follow or FollowAbout
// replaces this one.
func (c *PathPatternCommand) FollowAbout(pivot r3.Vec) *PathPatternCommand {
	var err error
	if !proofbound.FiniteVec(pivot) {
		err = fmt.Errorf(`%w: the path pattern pivot is not finite`, ErrNotFinite)
	}
	c.follow.Set(pathTurn{pivot: pivot, hasPivot: true}, err)
	return c
}

// Do places the copies, or returns the error of the first gate that refuses
// with the document unchanged. A nil ctx is ErrDegenerate, and a second call
// returns ErrConsumed.
func (c *PathPatternCommand) Do(ctx context.Context) ([]*Body, error) {
	return runCommand(ctx, &c.state, OperationCopy, c.run)
}

func (c *PathPatternCommand) run(ctx context.Context) ([]*Body, error) {
	turn, turnErr := c.follow.Get()
	return pathPattern(ctx, c.b, c.path, c.stations, pathFollow{
		on: c.follow.IsSet(), pivot: turn.pivot, hasPivot: turn.hasPivot, err: turnErr, subject: "FollowAbout",
		stationsSubject: "stations",
	})
}

// pathFollow is the settings of a path pattern's implementation: the folded form
// of its follow methods and the subject of its stations. err is the fault of the
// pivot, refused where the option decode refused it, and subject names what that
// diagnostic points at. stationsSubject names the constructor argument a refused
// stations value points at. A composite that calls the implementation with a value
// it computed clears either subject or names its own argument
// (agent-docs/command-api-design.md §11).
type pathFollow struct {
	on              bool
	pivot           r3.Vec
	hasPivot        bool
	err             error
	subject         string
	stationsSubject string
}

// pathPattern is PathPattern's implementation. ctx is not nil, and the errors
// it returns carry no operation tag (agent-docs/command-api-design.md §11).
func pathPattern(ctx context.Context, b *Body, path *Path, stations PathStations, follow pathFollow) ([]*Body, error) {
	if b == nil || b.n.Doc == nil {
		return nil, fmt.Errorf(`%w: the body belongs to no document`, ErrDegenerate)
	}
	if path == nil {
		return nil, errNilPath
	}
	settings, err := foldPathPattern(stations, follow)
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
	var points []pathsample.Station
	if settings.hasCount {
		points, err = pathsample.SampleStations(path.Start(), samples, settings.count)
		if err != nil {
			return nil, pathSampleError(err)
		}
		return pathCopies(ctx, b, points, settings)
	}
	points, err = pathsample.SampleSpaced(path.Start(), samples, settings.spacing)
	if err != nil {
		return nil, pathSampleError(err)
	}
	if len(points) < 2 {
		return nil, fmt.Errorf(`%w: a spacing of %g mm leaves no copy along the path`, ErrDegenerate, settings.spacing)
	}
	return pathCopies(ctx, b, points, settings)
}

// pathPatternSettings are the stations and follow methods of a path pattern
// read.
type pathPatternSettings struct {
	count    int // instances, set when hasCount
	hasCount bool
	spacing  float64 // millimetres, set when the stations are a spacing
	follow   bool
	pivot    r3.Vec
	hasPivot bool
}

// foldPathPattern reads and validates the stations and the follow methods of a
// path pattern.
func foldPathPattern(stations PathStations, follow pathFollow) (pathPatternSettings, error) {
	var out pathPatternSettings
	errNil := subjectFault(OperationCopy, follow.stationsSubject,
		fmt.Errorf(`%w: a path pattern needs a PathCount or a PathSpacing to place its copies`, ErrDegenerate))
	var spacing units.Value
	switch s := stations.(type) {
	case PathCount:
		out.count, out.hasCount = s.Count, true
	case *PathCount:
		if s == nil {
			return out, errNil
		}
		out.count, out.hasCount = s.Count, true
	case PathSpacing:
		spacing = s.Spacing
	case *PathSpacing:
		if s == nil {
			return out, errNil
		}
		spacing = s.Spacing
	default:
		// The interface is sealed, so the only value left is nil.
		return out, errNil
	}
	if out.hasCount {
		if out.count < 2 {
			return out, subjectFault(OperationCopy, follow.stationsSubject,
				fmt.Errorf(`%w: a path pattern needs a count of at least 2, got %d`, ErrDegenerate, out.count))
		}
	} else {
		step, err := sectionrecord.MagnitudeIn(spacing, units.Length, units.Millimeter, "the path pattern spacing")
		if err != nil {
			return out, subjectFault(OperationCopy, follow.stationsSubject, err)
		}
		out.spacing = step
	}
	if follow.err != nil {
		return out, subjectError(OperationCopy, follow.subject, follow.err)
	}
	out.follow, out.pivot, out.hasPivot = follow.on, follow.pivot, follow.hasPivot
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
// restores the document to the state it had before the first copy, so no copy,
// history entry or producer number of the failed call remains.
func pathCopies(ctx context.Context, b *Body, stations []pathsample.Station,
	settings pathPatternSettings) (result []*Body, retErr error) {
	d := documentHandle(b.n.Doc)
	snap := d.snapshot(ctx)
	defer func() {
		if retErr != nil {
			d.restore(snap)
		}
	}()
	copies := make([]*Body, 0, len(stations)-1)
	for _, st := range stations[1:] {
		t, err := pathStep(stations[0], st, settings)
		if err != nil {
			return nil, err
		}
		c, err := copyBody(ctx, b, copyMotion{motion: t})
		if err != nil {
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
