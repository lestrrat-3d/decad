package decad

import (
	"context"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/loftface"
)

// A consumer must rederive a stored occupied arc before using its source
// endpoints. This binds its exact band time and both root brackets to the
// current Sketch, Revolve section, cone trims, and retained source faces.
func revolveLoftFacesToolCapOccupiedArcCurrent(ctx context.Context,
	proof *revolveLoftFacesSourceCertificate,
	recorded *revolveLoftFacesToolCapOccupiedArc) (bool, error) {
	if ctx == nil || proof == nil || recorded == nil || recorded.circle.tool != 1 {
		return false, nil
	}
	if err := ctx.Err(); err != nil {
		return false, err
	}
	circles, current := revolveLoftFacesToolCapSidesOf(proof)
	if !current {
		return false, nil
	}
	for _, circle := range circles.circles {
		if !revolveLoftFacesToolCapCircleEqual(circle, recorded.circle) {
			continue
		}
		span := proof.section.Spans[circle.span]
		var fresh *revolveLoftFacesToolCapOccupiedArc
		var proved bool
		var err error
		if span.Start.U == span.End.U {
			fresh, proved, err = revolveLoftFacesHeelVerticalOccupiedArc(ctx, proof, circle)
		} else {
			fresh, proved, err = revolveLoftFacesHeelSlopedOccupiedArc(ctx, proof, circle)
		}
		if err != nil || !proved {
			return false, err
		}
		return revolveLoftFacesToolCapOccupiedArcEqual(fresh, recorded), nil
	}
	return false, nil
}

// revolveLoftFacesToolCapOccupiedArcCurrentIn is
// revolveLoftFacesToolCapOccupiedArcCurrent for a caller that holds the
// census of this proof from earlier in its call. For an arc that census
// derived itself, it compares the arc with the census's private snapshot
// of that derivation; any other arc is rederived from the live source.
func revolveLoftFacesToolCapOccupiedArcCurrentIn(ctx context.Context,
	proof *revolveLoftFacesSourceCertificate, census *revolveLoftFacesPairCensus,
	recorded *revolveLoftFacesToolCapOccupiedArc) (bool, error) {
	if ctx != nil && proof != nil && census != nil && recorded != nil {
		for i := range census.ownArcs {
			if census.ownArcs[i].arc != recorded {
				continue
			}
			if err := ctx.Err(); err != nil {
				return false, err
			}
			return revolveLoftFacesToolCapOccupiedArcEqual(&census.ownArcs[i].derived, recorded), nil
		}
	}
	return revolveLoftFacesToolCapOccupiedArcCurrent(ctx, proof, recorded)
}

func revolveLoftFacesToolCapOccupiedArcEqual(a, b *revolveLoftFacesToolCapOccupiedArc) bool {
	return a != nil && b != nil &&
		revolveLoftFacesToolCapCircleEqual(a.circle, b.circle) &&
		a.band == b.band && loftface.RatEqual(a.time, b.time) &&
		revolveLoftFacesToolCapBoundaryEqual(a.negativeZ, b.negativeZ) &&
		revolveLoftFacesToolCapBoundaryEqual(a.positiveZ, b.positiveZ) &&
		a.throughPositiveY == b.throughPositiveY
}

func revolveLoftFacesToolCapCircleEqual(a, b revolveLoftFacesToolCapCircle) bool {
	return a.blankFace == b.blankFace && a.capFace == b.capFace &&
		a.capOrigin == b.capOrigin && a.span == b.span && a.tool == b.tool &&
		a.sourceMiss == b.sourceMiss &&
		loftface.RatEqual(a.x, b.x) &&
		loftface.RatEqual(a.radius, b.radius)
}

func revolveLoftFacesToolCapBoundaryEqual(a, b revolveLoftFacesToolCapBoundary) bool {
	if a.segment != b.segment || a.span != b.span || a.ref != b.ref ||
		a.face != b.face || a.outward != b.outward ||
		!loftface.IntervalEqual(a.parameter, b.parameter) {
		return false
	}
	for axis := range a.box {
		if !loftface.IntervalEqual(a.box[axis], b.box[axis]) {
			return false
		}
	}
	return true
}

// A field added to the arc, circle or boundary must be copied here and
// compared in the Equal functions above.
func revolveLoftFacesCloneToolCapOccupiedArc(
	a *revolveLoftFacesToolCapOccupiedArc) revolveLoftFacesToolCapOccupiedArc {
	out := *a
	out.circle.x = revolveLoftFacesCloneRat(a.circle.x)
	out.circle.radius = revolveLoftFacesCloneRat(a.circle.radius)
	out.time = revolveLoftFacesCloneRat(a.time)
	out.negativeZ = revolveLoftFacesCloneToolCapBoundary(a.negativeZ)
	out.positiveZ = revolveLoftFacesCloneToolCapBoundary(a.positiveZ)
	return out
}

func revolveLoftFacesCloneToolCapBoundary(
	b revolveLoftFacesToolCapBoundary) revolveLoftFacesToolCapBoundary {
	b.parameter = loftFaceInterval{Lo: revolveLoftFacesCloneRat(b.parameter.Lo), Hi: revolveLoftFacesCloneRat(b.parameter.Hi)}
	for i := range b.box {
		b.box[i] = loftFaceInterval{Lo: revolveLoftFacesCloneRat(b.box[i].Lo), Hi: revolveLoftFacesCloneRat(b.box[i].Hi)}
	}
	return b
}

func revolveLoftFacesCloneRat(x *big.Rat) *big.Rat {
	if x == nil {
		return nil
	}
	return new(big.Rat).Set(x)
}
