package crossbore

import (
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/facetproof"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// ProveChamferBounds encloses source-to-held boundary motion and the volume
// and area differences after the caller has admitted the source pose and
// proved every cross-bore cell.
func ProveChamferBounds(rec *facetedrecord.CrossCylinderBoreRecord, setback float64,
	mesh *Mesh, maximum CellResult) (float64, float64, float64, error) {
	const n = 2 * HalfStations
	// Every vertical wall patch pairs source and held points at the same
	// height fraction. The band pairs them at the same angle and transverse
	// fraction. Two endpoint curves in one cell have coordinate spread at
	// most width; the triangle diagonal adds one further spread. Three box
	// diagonals enclose all such pairings and their rounded sample vertices.
	R, r, H, d := rec.Radius, rec.Bore, rec.Height, setback
	xMinSquared := new(big.Rat).Sub(
		new(big.Rat).Mul(proofarith.FloatRat(R), proofarith.FloatRat(R)),
		new(big.Rat).Mul(proofarith.FloatRat(r), proofarith.FloatRat(r)))
	xMin := proofbound.RatSqrtDown(xMinSquared)
	width := proofbound.RatFloatUp(new(big.Rat).Sub(proofarith.FloatRat(R), proofarith.FloatRat(xMin)))
	width = proofbound.AbsSumUpper(width, proofbound.ProductUpper(2, d), rec.SourceBound)
	span := proofbound.AbsSumUpper(proofbound.ProductUpper(2, r), proofbound.ProductUpper(2, d),
		proofbound.ProductUpper(2, rec.SourceBound))
	collar := proofbound.ProductUpper(width, proofbound.ProductUpper(span, span))
	piUpper := proofbound.RatFloatUp(proofbound.PiUpper)
	// The mesh builder certifies the maximum exact angular gap of every
	// station, including the mouth schedules and the full-turn wrap. It also
	// encloses every held outer-ring vertex against that source-circle
	// parameter, covering both radius and angle-conversion rounding.
	outerChord := proofbound.ProductUpper(R,
		proofbound.DivUpper(proofbound.ProductUpper(mesh.OuterAngularGap, mesh.OuterAngularGap), 8))
	// The bore wall is a ruled cylinder patch whose far loop rotates by
	// delta_b=d*cos(theta)*sin(theta)/Q. A triangle spans the station gap
	// PLUS that cross-wall twist. |cos*sin|<=1/2 and Q>=xMin, so twice the
	// maximum twist is at most d/xMin. The doubled allowance also covers a
	// possible sign change of delta_b across a station cell.
	boreStep := proofbound.AbsSumUpper(
		proofbound.DivUpper(proofbound.ProductUpper(2, piUpper), n),
		proofbound.DivUpper(d, xMin))
	boreChord := proofbound.ProductUpper(r,
		proofbound.DivUpper(proofbound.ProductUpper(boreStep, boreStep), 8))
	bound := proofbound.AbsSumUpper(
		proofbound.ProductUpper(3, proofbound.Radius3D(maximum.Width)),
		maximum.SampleError, proofbound.ProductUpper(4, rec.SourceBound),
		mesh.OuterStationError, outerChord, boreChord)
	if !(bound < 0.1) || proofbound.IsNonFinite(bound) {
		return 0, 0, 0, fmt.Errorf("%w: the cross-bore chamfer exceeds its 0.1 mm proof plan", decaderr.ErrUnsupported)
	}
	outerShell := proofbound.ProductUpper(4*piUpper,
		proofbound.ProductUpper(R, proofbound.ProductUpper(H,
			proofbound.AbsSumUpper(outerChord, mesh.OuterStationError, rec.SourceBound))))
	boreShell := proofbound.ProductUpper(8*piUpper,
		proofbound.ProductUpper(R, proofbound.ProductUpper(r,
			proofbound.AbsSumUpper(boreChord, rec.SourceBound))))
	volumeGap := proofbound.AbsSumUpper(collar, outerShell, boreShell)
	// Surface area is bounded by the complete outer cylinder and caps, the
	// complete bore wall, and the band's derivative times its width. Adding
	// the held area bounds their absolute difference without assuming equal
	// areas at a mesh cut.
	sourceArea := proofbound.AbsSumUpper(
		proofbound.ProductUpper(2*piUpper, proofbound.ProductUpper(R, H)),
		proofbound.ProductUpper(2*piUpper, proofbound.ProductUpper(R, R)),
		proofbound.ProductUpper(4*piUpper, proofbound.ProductUpper(R, r)),
		proofbound.ProductUpper(2*piUpper,
			proofbound.ProductUpper(maximum.Speed, maximum.BandLength)))
	heldArea := facetproof.MeshAreaUpper(mesh.Verts, mesh.Tris)
	areaSlack := proofbound.AbsSumUpper(sourceArea, heldArea)
	if proofbound.IsNonFinite(volumeGap) || proofbound.IsNonFinite(areaSlack) {
		return 0, 0, 0, fmt.Errorf("%w: the cross-bore chamfer has no finite volume or area proof", decaderr.ErrUnsupported)
	}
	return bound, volumeGap, areaSlack, nil
}
