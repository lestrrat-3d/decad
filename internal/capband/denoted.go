package capband

import (
	"math"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// DenotedNormalAllow bounds how far a band patch's built surface turns from
// the surface its records denote. departure.go bounds the tag against the
// built patch. This term covers the built patch against the denoted one: the
// cap-level directrix is an offset the build solved, held within delta of the
// denoted contour in the plane and within capDelta of the denoted cap level
// along the axis, and the side directrix sits on a level held within
// levelDelta. dz is the patch's axial height, capZ − sideZ, and dr its radial
// change, capRadius − sideRadius (zero for a flat patch).
//
// A flat patch's tag passes through its side edge A→B and one cap corner D.
// Both side ends share one level, so shifting that level is a translation of
// the line AB. Read relative to AB, D therefore moves by at most
// e = delta + capDelta + levelDelta. Moving D by e turns the normal of
// (B − A)×(D − A) by at most 2|e| over D's distance from the line AB, and
// that distance is at least the axial height h, since the line lies in the
// side level.
//
// A circular patch is coaxial with the cone it denotes, because both
// directrices keep the recorded centre. Its normal at a given azimuth is
// fixed by its half angle atan(|dr|/h), which moves by at most the change of
// that ratio: (|e_r| + |dr|·|e_z|/h)/h. One formula covers both kinds:
// 2·(delta + axial·max(1, |dr|/h))/h, with h read at the bottom of its span.
// dz and dr arrive as rounded float differences, so each is widened by one
// step before it is read. A height that span cannot keep positive bounds
// nothing, and the reading is +Inf rather than a number that would
// understate it.
//
// The draft walls of a tapered extrude share the same construction and read
// the same term.
func DenotedNormalAllow(delta, capDelta, levelDelta, dz, dr float64) float64 {
	axial := proofbound.AbsSumUpper(capDelta, levelDelta)
	hLow := freeform.DownRound(freeform.DownRound(math.Abs(dz)) - axial)
	if !(hLow > 0) {
		return math.Inf(1)
	}
	lever := math.Max(1, proofbound.DivUpper(proofbound.UpRound(math.Abs(dr)), hLow))
	shift := proofbound.AbsSumUpper(delta, proofbound.ProductUpper(axial, lever))
	return proofbound.DivUpper(proofbound.ProductUpper(2, shift), hLow)
}
