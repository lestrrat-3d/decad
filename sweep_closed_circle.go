package decad

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/sweeparc"
	"github.com/lestrrat-3d/r3"
)

// closedSweepCircle recognizes the two complementary arcs of one exact
// directed circle. The public Path record already proves that each arc is
// nondegenerate and shares its endpoint with the next arc. Matching exact
// centers and codirectional axes make their angles sum to one turn; matching
// the first and final tangents proves the section frame closes at the seam.
func closedSweepCircle(path *Path) (*sweeparc.Record, bool) {
	if len(path.records) != 2 || path.records[0].Arc == nil || path.records[1].Arc == nil {
		return nil, false
	}
	first, second := path.records[0], path.records[1]
	if !samePathPoint(first.Start, second.End) || !samePathPoint(first.End, second.Start) {
		return nil, false
	}
	for i := range first.Arc.Center {
		if first.Arc.Center[i].Cmp(second.Arc.Center[i]) != 0 {
			return nil, false
		}
	}
	if !sweeparc.IsZero(sweeparc.Cross(first.Arc.Axis, second.Arc.Axis)) ||
		sweeparc.Dot(first.Arc.Axis, second.Arc.Axis).Sign() <= 0 {
		return nil, false
	}
	if !sweeparc.IsZero(sweeparc.Cross(second.TangentOut, first.TangentIn)) ||
		sweeparc.Dot(second.TangentOut, first.TangentIn).Sign() <= 0 {
		return nil, false
	}
	return first.Arc, true
}

// evalClosedCircleSweep reduces the structurally proven full turn to the
// existing full Revolve evaluator. That evaluator supplies axis-incidence,
// topology, four bounded readings, and an occupied-volume mesh proof.
func evalClosedCircleSweep(ctx context.Context, d *Document, ref producerID,
	profile profileRecord, plane planeRecord, frame r3.Frame, path *Path,
	work *freeform.FreeformWork,
) (*Body, error) {
	carrier, ok := closedSweepCircle(path)
	if !ok {
		return nil, fmt.Errorf(`%w: the closed sweep path is outside the exact two-arc full-circle class`, ErrUnsupported)
	}
	first := path.records[0]
	geometry, err := sweeparc.Derive(first.Start, carrier, first.ArcPhi, first.ArcAngle, plane)
	if err != nil {
		return nil, err
	}
	ax, side, err := resolveAxisSide(ctx, profile, geometry.Line, work)
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
		profile:     profile,
		frame:       frame,
		ax:          ax,
		phi0:        phi0,
		phi1:        phi1,
		full:        full,
		den:         den,
		xform:       r3.Identity(),
		radialProof: ax.RadialProof,
	}
	body, err := evalRevolveContextWork(ctx, d, ref, revolve, work)
	if err != nil {
		return nil, err
	}
	witness := prismPayload{profile: profile, frame: frame, xform: r3.Identity()}
	finishArcSweepBody(body, sweepPayload{
		prism:        witness,
		revolve:      revolve,
		arc:          true,
		closedCircle: true,
		path:         path,
	})
	return body, nil
}
