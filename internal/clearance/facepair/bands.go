package facepair

import (
	"math"

	"github.com/lestrrat-3d/decad/internal/clearance"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// tiltedCrossingExcluded is the crossing annulus of
// agent-docs/clearance-band-design.md §4.2: it proves a plane face f (unit
// normal n, offset c) never crosses the carrier of a cylinder face g (anchor
// a₀, unit axis a, radius R) whose axis the oracle does not prove parallel to
// n. The axis meets the plane at P₀ = a₀ + a·(c − n·a₀)/(n·a). A point
// x = P₀ + w of the plane, w ⊥ n, lies √(|w|² − (w·a)²) from the axis, and
// |w·a| ≤ |w|·s with s the tilt sine of a and n, so a point of the crossing
// satisfies R ≤ |w| ≤ R/√(1 − s²): the crossing lies in the annulus about P₀
// between those radii. The reading holds only for |n·a| ≥ ½, where P₀ is well
// conditioned. It reports true when the annulus misses f's trim by more than
// the tolerance and the centre's charge, and false for every other outcome.
func (k *Kernel) tiltedCrossingExcluded(f, g *clearance.CFace) bool {
	na := f.N.Dot(g.Axis)
	if !(math.Abs(na) >= 0.5) {
		return false
	}
	cos := clearance.TiltCosine(clearance.TiltSine(g.Axis, f.N))
	rOut := proofbound.DivUpper(g.Radius, cos)
	if proofbound.IsNonFinite(rOut) {
		return false
	}
	offset := f.PlaneOffset() - f.N.Dot(g.Anchor)
	centre := g.Anchor.Add(g.Axis.Scale(offset / na))
	charge := proofbound.DivUpper(
		clearance.EnvelopeCharge([]r3.Vec{g.Anchor, f.O}, g.Radius, math.Abs(offset)),
		freeform.DownRound(math.Abs(na)))
	cx, cy := f.PlaneCoords(centre)
	return clearance.AnnulusRegionMiss(f.Region, cx, cy, g.Radius, rOut, proofbound.AbsSumUpper(k.tol, charge))
}
