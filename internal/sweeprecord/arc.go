package sweeprecord

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/revolveangle"
	"github.com/lestrrat-3d/decad/internal/revolveplan"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/sweeparc"
	"github.com/lestrrat-3d/decad/internal/sweepinput"
	"github.com/lestrrat-3d/decad/internal/topo"

	"github.com/lestrrat-3d/r3"
)

// This file owns the one-arc Sweep reduction. The circular path records one
// exact rational circle. Its float axis and angle are held
// beside rational enclosures of the exact values they publish.

func BuildArc(
	ctx context.Context,
	env topo.Env,
	profile momentinput.Profile,
	plane sectionrecord.PlaneRecord,
	frame r3.Frame,
	path *sweepinput.Path,
	pathRecord sweepinput.PathRecord,
	work *freeform.FreeformWork,
	surfaceResult bool,
) (*topo.Body, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	geometry, err := sweeparc.Derive(pathRecord.Start, pathRecord.Arc,
		pathRecord.ArcPhi, pathRecord.ArcAngle, plane)
	if err != nil {
		return nil, err
	}
	ax, side, err := revolveplan.ResolveAxisSide(ctx, profile, geometry.Line, work)
	if err != nil {
		return nil, err
	}

	phi0, phi1 := 0.0, geometry.Phi
	den := revolveangle.Sweep{Phi0: revolveangle.Zero(), Phi1: geometry.Angle}
	reverseCaps := false
	if side < 0 {
		phi0, phi1 = -phi1, -phi0
		den.Phi0, den.Phi1 = den.Phi1.Neg(), den.Phi0.Neg()
		reverseCaps = true
	}
	// The flag rides the REDUCED revolvePayload, not just the finishing
	// sweepPayload literal below: revolveplan.Build
	// reads rp.SurfaceResult to decide Kind()/solid, cap omission, topobuild.SheetLumps
	// and the area subtraction — including Table W's profile-meets-axis row —
	// which is what gives the arc reduction the whole sheet behaviour for free
	// (agent-docs/surface-design.md §4).
	revolve := revolveplan.Payload{
		Profile:       profile,
		Frame:         frame,
		Ax:            ax,
		Phi0:          phi0,
		Phi1:          phi1,
		Den:           den,
		Xform:         r3.Identity(),
		SurfaceResult: surfaceResult,
	}
	body, err := revolveplan.Build(ctx, env, revolve, work)
	if err != nil {
		return nil, err
	}

	// Verify's Sweep diameter arm reads this zero-height start-section witness.
	// Every one of its points lies on the real start cap, so its point-set
	// diameter is a sound lower bound on the swept body's diameter. It is not
	// itself built, so its own surfaceResult stays at its zero value.
	witness := prismrecord.Payload{
		Profile: profile,
		Frame:   frame,
		Xform:   r3.Identity(),
	}
	payload := Payload{
		Prism:          witness,
		Revolve:        revolve,
		Arc:            true,
		ReverseArcCaps: reverseCaps,
		Path:           path,
		SurfaceResult:  surfaceResult,
	}
	FinishArc(body, payload)
	return body, nil
}
