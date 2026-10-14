package sweeptransport

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/sweepinput"
	"github.com/lestrrat-3d/r3"
)

type planeRecord = sectionrecord.PlaneRecord

type sweepTransportFrame = Frame

// TransportSweepFramesContext returns the transported section frame at every
// path point, including the source frame. Lines translate the frame without
// rotating it. Arcs apply the right-handed carrier rotation, which is the
// rotation-minimizing transport of a frame whose normal follows the path.
func TransportSweepFramesContext(
	ctx context.Context,
	path *sweepinput.Path,
	plane planeRecord,
) ([]sweepTransportFrame, error) {
	return transportSweepFrames(ctx, path, plane, false)
}

// TransportClosedSweepFrames is TransportSweepFramesContext for the closed
// composite path class.
func TransportClosedSweepFrames(ctx context.Context, path *sweepinput.Path, plane planeRecord) ([]sweepTransportFrame, error) {
	return transportSweepFrames(ctx, path, plane, true)
}

func transportSweepFrames(ctx context.Context, path *sweepinput.Path, plane planeRecord, closedComposite bool) ([]sweepTransportFrame, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	if path == nil {
		return nil, fmt.Errorf(`%w: a nil path has no frames to transport`, decaderr.ErrDegenerate)
	}
	if err := sweepinput.ValidateSweepPathGeometry(path, plane); err != nil {
		return nil, err
	}
	if sweepinput.SamePathPoint(path.Start, path.End) && !closedComposite {
		return nil, fmt.Errorf(`%w: closed sweep paths have no admitted end frame`, decaderr.ErrUnsupported)
	}

	frame, err := r3.NewFrame(plane.Origin, plane.U, plane.V)
	if err != nil {
		return nil, fmt.Errorf(`%w: the recorded plane is degenerate: %s`, decaderr.ErrDegenerate, err)
	}
	current, err := InitialFrame(frame)
	if err != nil {
		return nil, err
	}
	frames := make([]sweepTransportFrame, 1, len(path.Records)+1)
	frames[0] = current

	for i, record := range path.Records {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		if record.Arc == nil {
			current, err = Line(current, record.Start, record.End)
		} else {
			current, err = RecordArc(current, record)
		}
		if err != nil {
			return nil, fmt.Errorf(`sweep path span %d: %w`, i, err)
		}
		frames = append(frames, current)
	}
	if closedComposite {
		start, end := frames[0], frames[len(frames)-1]
		if end.OriginBound != 0 || end.UBound != 0 || end.VBound != 0 || end.NBound != 0 ||
			end.Frame.Origin() != start.Frame.Origin() || end.Frame.U() != start.Frame.U() ||
			end.Frame.V() != start.Frame.V() || end.Frame.N() != start.Frame.N() {
			return nil, fmt.Errorf(`%w: the closed sweep's transported section does not exactly match its start`, decaderr.ErrUnsupported)
		}
	}
	return frames, nil
}
