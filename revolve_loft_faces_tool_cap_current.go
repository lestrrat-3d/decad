package decad

import (
	"context"
	"math/big"
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
		span := proof.section.spans[circle.span]
		var fresh *revolveLoftFacesToolCapOccupiedArc
		var proved bool
		var err error
		if span.start.U == span.end.U {
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

func revolveLoftFacesToolCapOccupiedArcEqual(a, b *revolveLoftFacesToolCapOccupiedArc) bool {
	return a != nil && b != nil &&
		revolveLoftFacesToolCapCircleEqual(a.circle, b.circle) &&
		a.band == b.band && revolveLoftFacesToolCapRatEqual(a.time, b.time) &&
		revolveLoftFacesToolCapBoundaryEqual(a.negativeZ, b.negativeZ) &&
		revolveLoftFacesToolCapBoundaryEqual(a.positiveZ, b.positiveZ) &&
		a.throughPositiveY == b.throughPositiveY
}

func revolveLoftFacesToolCapCircleEqual(a, b revolveLoftFacesToolCapCircle) bool {
	return a.blankFace == b.blankFace && a.capFace == b.capFace &&
		a.capOrigin == b.capOrigin && a.span == b.span && a.tool == b.tool &&
		a.sourceMiss == b.sourceMiss &&
		revolveLoftFacesToolCapRatEqual(a.x, b.x) &&
		revolveLoftFacesToolCapRatEqual(a.radius, b.radius)
}

func revolveLoftFacesToolCapBoundaryEqual(a, b revolveLoftFacesToolCapBoundary) bool {
	if a.segment != b.segment || a.span != b.span || a.ref != b.ref ||
		a.face != b.face || a.outward != b.outward ||
		!revolveLoftFacesToolCapIntervalEqual(a.parameter, b.parameter) {
		return false
	}
	for axis := range a.box {
		if !revolveLoftFacesToolCapIntervalEqual(a.box[axis], b.box[axis]) {
			return false
		}
	}
	return true
}

func revolveLoftFacesToolCapIntervalEqual(a, b loftFaceInterval) bool {
	return revolveLoftFacesToolCapRatEqual(a.Lo, b.Lo) &&
		revolveLoftFacesToolCapRatEqual(a.Hi, b.Hi)
}

func revolveLoftFacesToolCapRatEqual(a, b *big.Rat) bool {
	if a == nil || b == nil {
		return a == nil && b == nil
	}
	return a.Cmp(b) == 0
}
