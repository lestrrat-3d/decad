package decad

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/revolveplan"
	"github.com/lestrrat-3d/decad/internal/sweeparc"
	"github.com/lestrrat-3d/decad/internal/sweepinput"
	"github.com/lestrrat-3d/r3"
)

// evalClosedCircleSweep reduces the structurally proven full turn to the
// existing full Revolve evaluator. That evaluator supplies axis-incidence,
// topology, four bounded readings, and an occupied-volume mesh proof.
func evalClosedCircleSweep(ctx context.Context, d *Document, ref producerID,
	profile profileRecord, plane planeRecord, frame r3.Frame, path *sweepinput.Path,
	work *freeform.FreeformWork,
) (*Body, error) {
	carrier, ok := sweepinput.ClosedCircle(path.Records)
	if !ok {
		return nil, fmt.Errorf(`%w: the closed sweep path is outside the exact two-arc full-circle class`, ErrUnsupported)
	}
	first := path.Records[0]
	geometry, err := sweeparc.Derive(first.Start, carrier, first.ArcPhi, first.ArcAngle, plane)
	if err != nil {
		return nil, err
	}
	ax, side, err := revolveplan.ResolveAxisSide(ctx, profile, geometry.Line, work)
	if err != nil {
		return nil, err
	}
	phi0, phi1, full, den, _, err := d.resolveAngularExtent(
		FullRevolution{}, d.angularStopCtx(frame, geometry.Line, ax))
	if err != nil {
		return nil, err
	}
	if side < 0 {
		phi0, phi1 = -phi1, -phi0
		den.Phi0, den.Phi1 = den.Phi1.Neg(), den.Phi0.Neg()
	}
	revolve := revolvePayload{
		Profile:     profile,
		Frame:       frame,
		Ax:          ax,
		Phi0:        phi0,
		Phi1:        phi1,
		Full:        full,
		Den:         den,
		Xform:       r3.Identity(),
		RadialProof: ax.RadialProof,
	}
	body, err := bodyResult(revolveplan.Build(ctx, envOf(documentNode(d), ref), revolve, work))
	if err != nil {
		return nil, err
	}
	witness := prismrecord.Payload{Profile: profile, Frame: frame, Xform: r3.Identity()}
	finishArcSweepBody(body, sweepPayload{
		Prism:        witness,
		Revolve:      revolve,
		Arc:          true,
		ClosedCircle: true,
		Path:         path,
	})
	return body, nil
}
