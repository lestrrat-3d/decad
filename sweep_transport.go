package decad

import (
	"context"
	"fmt"

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
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	if path == nil {
		return nil, fmt.Errorf(`%w: a nil path has no frames to transport`, ErrDegenerate)
	}
	if err := validateSweepPathGeometry(path, plane); err != nil {
		return nil, err
	}
	if samePathPoint(path.Start(), path.End()) {
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
		if record.arc == nil {
			current, err = sweeptransport.Line(current, record.start, record.end)
		} else {
			current, err = transportSweepArc(current, record)
		}
		if err != nil {
			return nil, fmt.Errorf(`sweep path span %d: %w`, i, err)
		}
		frames = append(frames, current)
	}
	return frames, nil
}

// transportSweepArc resolves the recorded carrier before numeric transport.
func transportSweepArc(current sweepTransportFrame, record pathSegmentRecord) (sweepTransportFrame, error) {
	if record.arc == nil {
		return sweepTransportFrame{}, fmt.Errorf(`%w: an arc span holds no circular carrier`, ErrDegenerate)
	}
	arc := *record.arc
	axisExact, status := surfacenormal.UnitVec3(sweepRatIntervalVec(arc.Axis))
	if status != surfacenormal.Proven {
		return sweepTransportFrame{}, fmt.Errorf(`%w: an arc span has no certified axis direction`, ErrUnsupported)
	}
	sin, cos, ok := record.arcAngle.SinCosFor(record.arcPhi)
	if !ok {
		return sweepTransportFrame{}, fmt.Errorf(`%w: an arc span has no certified sine and cosine`, ErrUnsupported)
	}
	return sweeptransport.TransportArc(current, sweeptransport.Arc{
		CenterExact: sweepRatIntervalVec(arc.Center),
		AxisExact:   axisExact,
		Sin:         sin, Cos: cos,
		Phi: record.arcPhi,
	})
}

func sweepRatIntervalVec(vector sweepRatVec) proofbound.IvVec3 {
	return proofbound.IvVec3{
		proofbound.PointInterval(vector[0]),
		proofbound.PointInterval(vector[1]),
		proofbound.PointInterval(vector[2]),
	}
}
