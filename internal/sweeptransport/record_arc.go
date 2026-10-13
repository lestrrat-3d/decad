package sweeptransport

import (
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/surfacenormal"
	"github.com/lestrrat-3d/decad/internal/sweeparc"
	"github.com/lestrrat-3d/decad/internal/sweepinput"
)

// RecordArc resolves the recorded carrier before numeric transport.
func RecordArc(current Frame, record sweepinput.PathRecord) (Frame, error) {
	if record.Arc == nil {
		return Frame{}, fmt.Errorf(`%w: an arc span holds no circular carrier`, decaderr.ErrDegenerate)
	}
	arc := *record.Arc
	axisExact, status := surfacenormal.UnitVec3(ratIntervalVec(arc.Axis))
	if status != surfacenormal.Proven {
		return Frame{}, fmt.Errorf(`%w: an arc span has no certified axis direction`, decaderr.ErrUnsupported)
	}
	sin, cos, ok := record.ArcAngle.SinCosFor(record.ArcPhi)
	if !ok {
		return Frame{}, fmt.Errorf(`%w: an arc span has no certified sine and cosine`, decaderr.ErrUnsupported)
	}
	return TransportArc(current, Arc{
		CenterExact: ratIntervalVec(arc.Center),
		AxisExact:   axisExact,
		Sin:         sin, Cos: cos,
		Phi: record.ArcPhi,
	})
}

func ratIntervalVec(vector sweeparc.RatVec) proofbound.IvVec3 {
	return proofbound.IvVec3{
		proofbound.PointInterval(vector[0]),
		proofbound.PointInterval(vector[1]),
		proofbound.PointInterval(vector[2]),
	}
}
