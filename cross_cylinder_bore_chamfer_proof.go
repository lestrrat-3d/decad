package decad

import (
	"context"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/crossbore"
	"github.com/lestrrat-3d/decad/internal/facetproof"
	"github.com/lestrrat-3d/decad/internal/orderedwork"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// crossBoreMirrorPose reports whether rec is exactly the record admission
// builds from its own two source bodies. Admission takes only the cardinal
// pose, compared without tolerance: the target circle centred on the Z axis
// in the XY frame, the tool circle centred at Y = 0 in the YZ frame, both
// unplaced. Both source walls, x²+y² = R² and y²+(z−c)² = r², are then
// exactly symmetric under y → −y and z−c → −(z−c), which is what
// crossbore.MirroredCellMaxima's mirror map needs. A pose that is symmetric
// only to rounding fails one of admission's exact comparisons.
func crossBoreMirrorPose(rec *crossCylinderBoreRecord) bool {
	again := recognizeCrossCylinderBore(bodyHandle(rec.Target), bodyHandle(rec.Tool))
	return again != nil && *again == *rec
}

// proveCrossBoreCells proves the 4,096 whole-turn cells of rec's positive
// mouth. It refuses a record that is not exactly admission's mirror-symmetric
// pose before proving any cell. It then proves the 1,025 quarter-turn base
// cells and compares every cell's held samples with its base cell's mirrored
// enclosures (crossbore.MirroredCellMaxima). Both passes run concurrently and
// fold in cell order.
func proveCrossBoreCells(ctx context.Context, rec *crossCylinderBoreRecord,
	setback float64, mesh *crossbore.Mesh) (crossbore.CellResult, error) {
	const n = 2 * crossbore.HalfStations
	if !crossBoreMirrorPose(rec) {
		return crossbore.CellResult{}, fmt.Errorf("%w: the cross-bore record is not the exact mirror-symmetric pose",
			ErrUnsupported)
	}
	gap := new(big.Rat).Quo(new(big.Rat).Mul(big.NewRat(2, 1), proofbound.PiUpper), big.NewRat(n, 1))
	return crossbore.MirroredCellMaxima(ctx, orderedwork.EachWorkers(n/4+1, crossbore.CellParallelMin),
		func(k int) (crossbore.CellExact, error) { return crossbore.CellExactProof(rec, setback, k, gap) },
		func(j int, base *crossbore.CellExact, flipY, flipZ bool) float64 {
			cell := mesh.CellHeld[j]
			held := [3]r3.Vec{mesh.Verts[cell[0]], mesh.Verts[cell[1]], mesh.Verts[cell[2]]}
			return crossbore.CellSampleError(&held, rec.CenterZ, &base.Coords, flipY, flipZ)
		})
}

// proveCrossBoreChamfer encloses all source-to-held boundary motion. Its
// volume proof charges the entire positive-mouth collar, where the ideal
// chamfer and mesh can differ arbitrarily, then two radial chord annuli for
// unchanged outer and bore walls. A collar wider than the actual removed
// region is deliberate; it keeps the occupied-volume argument independent
// of the exact band integral. Each source foot is at developed-cylinder
// distance d from its seam point, so its Euclidean distance is at most d;
// the ruled band lies in the resulting positive-mouth collar.
func proveCrossBoreChamfer(ctx context.Context, rec *crossCylinderBoreRecord,
	setback float64, mesh *crossbore.Mesh) (float64, float64, float64, error) {
	const n = 2 * crossbore.HalfStations
	maximum, err := proveCrossBoreCells(ctx, rec, setback, mesh)
	if err != nil {
		return 0, 0, 0, err
	}
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
		return 0, 0, 0, fmt.Errorf("%w: the cross-bore chamfer exceeds its 0.1 mm proof plan", ErrUnsupported)
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
		return 0, 0, 0, fmt.Errorf("%w: the cross-bore chamfer has no finite volume or area proof", ErrUnsupported)
	}
	return bound, volumeGap, areaSlack, nil
}
