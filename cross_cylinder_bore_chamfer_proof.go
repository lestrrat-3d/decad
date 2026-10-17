package decad

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/crossbore"
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
	if !crossBoreMirrorPose(rec) {
		return crossbore.CellResult{}, fmt.Errorf("%w: the cross-bore record is not the exact mirror-symmetric pose",
			ErrUnsupported)
	}
	return crossbore.ProveCells(ctx, rec, setback, mesh)
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
	maximum, err := proveCrossBoreCells(ctx, rec, setback, mesh)
	if err != nil {
		return 0, 0, 0, err
	}
	return crossbore.ProveChamferBounds(rec, setback, mesh, maximum)
}
