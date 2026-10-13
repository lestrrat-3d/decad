package decad

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/sweepinput"
	"github.com/lestrrat-3d/r3"
)

func evalClosedCyclicSweep(ctx context.Context, d *Document, ref producerID,
	profile profileRecord, plane planeRecord, frame r3.Frame, path *Path,
	work *freeform.FreeformWork,
) (*Body, error) {
	if !sweepinput.ClosedRacetrack(profile, plane, path.Start(), path.records) &&
		!sweepinput.ClosedPlanarTangentLoop(profile, plane, path.Start(), path.records) {
		return nil, fmt.Errorf(`%w: the closed sweep path is outside the admitted cyclic classes`, ErrUnsupported)
	}
	return evalCompositeSweepContextWithClosure(ctx, d, ref, profile, plane, frame, path, work, false, true)
}
