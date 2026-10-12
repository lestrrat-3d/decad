package decad

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/sweepinput"
	"github.com/lestrrat-3d/decad/internal/sweeptransport"

	"github.com/lestrrat-3d/decad/internal/proofbound"

	"github.com/lestrrat-3d/decad/internal/surfacenormal"
	"github.com/lestrrat-3d/r3"
)

// This file validates a recorded Sweep path and passes each span's geometry to
// internal/sweeptransport for rotation-minimizing frame transport.

type sweepTransportFrame = sweeptransport.Frame

// transportSweepFramesContext returns the transported section frame at every
// path point, including the source frame. Lines translate the frame without
// rotating it. Arcs apply the right-handed carrier rotation, which is the
// rotation-minimizing transport of a frame whose normal follows the path.
func transportSweepFramesContext(
	ctx context.Context,
	path *Path,
	plane planeRecord,
) ([]sweepTransportFrame, error) {
	return transportSweepFrames(ctx, path, plane, false)
}

func transportClosedRacetrackFrames(ctx context.Context, path *Path, plane planeRecord) ([]sweepTransportFrame, error) {
	return transportSweepFrames(ctx, path, plane, true)
}

func transportSweepFrames(ctx context.Context, path *Path, plane planeRecord, closedRacetrack bool) ([]sweepTransportFrame, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	if path == nil {
		return nil, fmt.Errorf(`%w: a nil path has no frames to transport`, ErrDegenerate)
	}
	if err := validateSweepPathGeometry(path, plane); err != nil {
		return nil, err
	}
	if samePathPoint(path.Start(), path.End()) && !closedRacetrack {
		return nil, fmt.Errorf(`%w: closed sweep paths have no admitted end frame`, ErrUnsupported)
	}

	frame, err := r3.NewFrame(plane.Origin, plane.U, plane.V)
	if err != nil {
		return nil, fmt.Errorf(`%w: the recorded plane is degenerate: %s`, ErrDegenerate, err)
	}
	current, err := sweeptransport.InitialFrame(frame)
	if err != nil {
		return nil, err
	}
	frames := make([]sweepTransportFrame, 1, len(path.records)+1)
	frames[0] = current

	for i, record := range path.records {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		if record.Arc == nil {
			current, err = sweeptransport.Line(current, record.Start, record.End)
		} else {
			current, err = transportSweepArc(current, record)
		}
		if err != nil {
			return nil, fmt.Errorf(`sweep path span %d: %w`, i, err)
		}
		frames = append(frames, current)
	}
	if closedRacetrack {
		start, end := frames[0], frames[len(frames)-1]
		if end.OriginBound != 0 || end.UBound != 0 || end.VBound != 0 || end.NBound != 0 ||
			end.Frame.Origin() != start.Frame.Origin() || end.Frame.U() != start.Frame.U() ||
			end.Frame.V() != start.Frame.V() || end.Frame.N() != start.Frame.N() {
			return nil, fmt.Errorf(`%w: the closed sweep's transported section does not exactly match its start`, ErrUnsupported)
		}
	}
	return frames, nil
}

// transportSweepArc resolves the recorded carrier before numeric transport.
func transportSweepArc(current sweepTransportFrame, record sweepinput.PathRecord) (sweepTransportFrame, error) {
	if record.Arc == nil {
		return sweepTransportFrame{}, fmt.Errorf(`%w: an arc span holds no circular carrier`, ErrDegenerate)
	}
	arc := *record.Arc
	axisExact, status := surfacenormal.UnitVec3(sweepRatIntervalVec(arc.Axis))
	if status != surfacenormal.Proven {
		return sweepTransportFrame{}, fmt.Errorf(`%w: an arc span has no certified axis direction`, ErrUnsupported)
	}
	sin, cos, ok := record.ArcAngle.SinCosFor(record.ArcPhi)
	if !ok {
		return sweepTransportFrame{}, fmt.Errorf(`%w: an arc span has no certified sine and cosine`, ErrUnsupported)
	}
	return sweeptransport.TransportArc(current, sweeptransport.Arc{
		CenterExact: sweepRatIntervalVec(arc.Center),
		AxisExact:   axisExact,
		Sin:         sin, Cos: cos,
		Phi: record.ArcPhi,
	})
}

func sweepRatIntervalVec(vector sweepRatVec) proofbound.IvVec3 {
	return proofbound.IvVec3{
		proofbound.PointInterval(vector[0]),
		proofbound.PointInterval(vector[1]),
		proofbound.PointInterval(vector[2]),
	}
}
