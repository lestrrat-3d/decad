package decad

import (
	"context"
	"math/big"
	"strings"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/revolveangle"
	"github.com/lestrrat-3d/decad/internal/sweeparc"
	"github.com/lestrrat-3d/decad/internal/sweepinput"

	"github.com/lestrrat-3d/r3"
)

// This file owns the one-arc Sweep reduction. ArcThrough's three recorded
// points derive one exact rational circle. Its float axis and angle are held
// beside rational enclosures of the exact values they publish.

type sweepRatVec = sweeparc.RatVec

func evalArcSweepContext(
	ctx context.Context,
	d *Document,
	ref producerID,
	profile profileRecord,
	plane planeRecord,
	frame r3.Frame,
	path *Path,
	pathRecord sweepinput.PathRecord,
	work *freeform.FreeformWork,
	surfaceResult bool,
) (*Body, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	geometry, err := sweeparc.Derive(pathRecord.Start, pathRecord.Arc,
		pathRecord.ArcPhi, pathRecord.ArcAngle, plane)
	if err != nil {
		return nil, err
	}
	ax, side, err := resolveAxisSide(ctx, profile, geometry.Line, work)
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
	// sweepPayload literal below: revolve_build.go's evalRevolveContextWork
	// reads rp.surfaceResult to decide Kind()/solid, cap omission, sheetLumps
	// and the area subtraction — including Table W's profile-meets-axis row —
	// which is what gives the arc reduction the whole sheet behaviour for free
	// (docs/surface-design.md §4).
	revolve := revolvePayload{
		profile:       profile,
		frame:         frame,
		ax:            ax,
		phi0:          phi0,
		phi1:          phi1,
		den:           den,
		xform:         r3.Identity(),
		surfaceResult: surfaceResult,
	}
	body, err := evalRevolveContextWork(ctx, d, ref, revolve, work)
	if err != nil {
		return nil, err
	}

	// Verify's Sweep diameter arm reads this zero-height start-section witness.
	// Every one of its points lies on the real start cap, so its point-set
	// diameter is a sound lower bound on the swept body's diameter. It is not
	// itself built, so its own surfaceResult stays at its zero value.
	witness := prismPayload{
		profile: profile,
		frame:   frame,
		xform:   r3.Identity(),
	}
	payload := sweepPayload{
		prism:          witness,
		revolve:        revolve,
		arc:            true,
		reverseArcCaps: reverseCaps,
		path:           path,
		surfaceResult:  surfaceResult,
	}
	finishArcSweepBody(body, payload)
	return body, nil
}

func sweepRatVecOf(v r3.Vec) sweepRatVec { return sweeparc.VecOf(v) }

func sweepRatSub(a, b sweepRatVec) sweepRatVec { return sweeparc.Sub(a, b) }

func sweepRatDot(a, b sweepRatVec) *big.Rat { return sweeparc.Dot(a, b) }

func sweepRatCross(a, b sweepRatVec) sweepRatVec { return sweeparc.Cross(a, b) }

func sweepRatIsZero(v sweepRatVec) bool { return sweeparc.IsZero(v) }

// finishArcSweepBody restores Table B's own role vocabulary on an arc
// reduction, which builds through the Revolve evaluator and so mints roles
// that know nothing of a path. Every wall role gains the path-span index ahead
// of the section's loop and segment indices, exactly as
// prefixSweepSpanZeroRole does for a straight reduction. A span whose Revolve
// axis was reoriented (reverseArcCaps) also has its two cap roles swapped back
// into PATH order, so capStart always names the section the path leaves and
// capEnd the one it arrives at — the orientation sweep_composite.go's own
// join pairing reads when it takes one span's end cap against the next span's
// start cap.
func finishArcSweepBody(body *Body, payload sweepPayload) {
	if built, ok := body.payload.(revolvePayload); ok {
		payload.revolve = built
	}
	for _, face := range body.Faces() {
		for i, origin := range face.origins {
			switch {
			case strings.HasPrefix(origin.Role, "side("):
				origin.Role = "side(0," + strings.TrimPrefix(origin.Role, "side(")
			case payload.reverseArcCaps && origin.Role == roleCapStart:
				origin.Role = roleCapEnd
			case payload.reverseArcCaps && origin.Role == roleCapEnd:
				origin.Role = roleCapStart
			}
			face.origins[i] = origin
		}
	}
	body.payload = payload
}
