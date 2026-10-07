package decad

import (
	"context"
	"fmt"
	"math/big"
	"strings"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/sweeparc"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
)

// This file owns the one-arc Sweep reduction. ArcThrough's three recorded
// points derive one exact rational circle. Its float axis and angle are held
// beside rational enclosures of the exact values they publish.

type sweepArcGeometry struct {
	line axisLine2
	phi  float64
	den  angleDenotation
}

type sweepRatVec = sweeparc.RatVec
type sweepArcRecord = sweeparc.Record

func evalArcSweepContext(
	ctx context.Context,
	d *Document,
	ref producerID,
	profile ProfileRecord,
	plane PlaneRecord,
	frame r3.Frame,
	path *Path,
	pathRecord pathSegmentRecord,
	work *freeform.FreeformWork,
	surfaceResult bool,
) (*Body, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	geometry, err := deriveSweepArc(pathRecord, plane)
	if err != nil {
		return nil, err
	}
	ax, side, err := resolveAxisSide(ctx, profile, geometry.line, work)
	if err != nil {
		return nil, err
	}

	phi0, phi1 := 0.0, geometry.phi
	den := sweepDenotation{phi0: zeroAngleDenotation(), phi1: geometry.den}
	reverseCaps := false
	if side < 0 {
		phi0, phi1 = -phi1, -phi0
		den.phi0, den.phi1 = den.phi1.neg(), den.phi0.neg()
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

func deriveSweepArc(pathRecord pathSegmentRecord, plane PlaneRecord) (sweepArcGeometry, error) {
	start := pathRecord.start
	normal := proofarith.DvCross(proofarith.DyVec(plane.U), proofarith.DyVec(plane.V))
	relStart := proofarith.DvSub(proofarith.DyVec(start), proofarith.DyVec(plane.Origin))
	if !proofarith.DvDot(relStart, normal).IsZero() {
		return sweepArcGeometry{}, fmt.Errorf(`%w: the sweep path must start in the profile plane`, ErrDegenerate)
	}

	if pathRecord.arc == nil {
		return sweepArcGeometry{}, fmt.Errorf(`%w: a sweep arc path record holds no circular carrier`, ErrDegenerate)
	}
	record := *pathRecord.arc
	centerRat := record.Center
	r0, rm, r1 := record.RadiusStart, record.RadiusMiddle, record.RadiusEnd
	axisRat := record.Axis
	tangent := sweepRatCross(axisRat, r0)
	normalRat := sweepRatFromDyadic(normal)
	if !sweepRatIsZero(sweepRatCross(tangent, normalRat)) || sweepRatDot(tangent, normalRat).Sign() <= 0 {
		return sweepArcGeometry{}, fmt.Errorf(`%w: the sweep path's initial tangent must follow the profile plane's positive normal`, ErrDegenerate)
	}
	if sweepRatDot(sweepRatSub(centerRat, sweepRatVecOf(plane.Origin)), normalRat).Sign() != 0 {
		return sweepArcGeometry{}, fmt.Errorf(`%w: the sweep arc's derived axis does not lie in the profile plane`, ErrDegenerate)
	}

	// The carrier was solved from all three points. This equality catches an
	// arithmetic regression before a derived angle reaches the evaluator.
	radiusSquared := sweepRatDot(r0, r0)
	if sweepRatDot(rm, rm).Cmp(radiusSquared) != 0 || sweepRatDot(r1, r1).Cmp(radiusSquared) != 0 {
		return sweepArcGeometry{}, fmt.Errorf(`%w: the sweep arc points do not share one exact carrier`, ErrUnsupported)
	}
	line, err := sweepArcAxisLine(centerRat, axisRat, plane)
	if err != nil {
		return sweepArcGeometry{}, err
	}

	return sweepArcGeometry{
		line: line,
		phi:  pathRecord.arcPhi,
		den:  pathRecord.arcAngle,
	}, nil
}

func recordSweepArc(start, through, end r3.Vec) (sweepArcRecord, error) {
	return sweeparc.RecordArc(start, through, end)
}

func sweepArcAxisLine(center, axis sweepRatVec, plane PlaneRecord) (axisLine2, error) {
	line, err := sweeparc.AxisLine(center, axis, plane.Origin, plane.U, plane.V)
	if err != nil {
		return axisLine2{}, err
	}
	return axisLine2{
		aU: line.AU, aV: line.AV,
		aUBound: line.AUBound, aVBound: line.AVBound,
		dU: line.DU, dV: line.DV,
		dUBound: line.DUBound, dVBound: line.DVBound,
	}, nil
}

func sweepArcAngle(r0, r1, axis sweepRatVec) (float64, angleDenotation, error) {
	phi, angle, err := sweeparc.ArcAngle(r0, r1, axis)
	return phi, angleFromInternal(angle), err
}

func sweepRatHeld(value *big.Rat) (float64, float64, bool) {
	return sweeparc.Held(value)
}

func sweepRatVecOf(v r3.Vec) sweepRatVec { return sweeparc.VecOf(v) }

func sweepRatFromDyadic(v proofarith.DyV3) sweepRatVec { return sweeparc.FromDyadic(v) }

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
