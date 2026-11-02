package tessellation

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// RectangleClearanceReplay runs the ordinary section distance test on the
// two non-adjacent pairs of one four-point loop, without dynamic containers.
func RectangleClearanceReplay(ctx context.Context, pts []SectionPoint, sag []float64, b *proofbound.ReplayBudget) error {
	if b == nil || len(pts) != 4 || len(sag) != 4 {
		return fmt.Errorf("%w: bounded section clearance requires four points", decaderr.ErrUnsupported)
	}
	if err := b.SpendWork(ctx, 64); err != nil {
		return err
	}
	minU, maxU := math.Inf(1), math.Inf(-1)
	minV, maxV := math.Inf(1), math.Inf(-1)
	maxAbs := 0.0
	for _, p := range pts {
		if proofbound.IsNonFinite(p.U) || proofbound.IsNonFinite(p.V) {
			return fmt.Errorf("%w: bounded section point is nonfinite", decaderr.ErrUnsupported)
		}
		minU, maxU = math.Min(minU, p.U), math.Max(maxU, p.U)
		minV, maxV = math.Min(minV, p.V), math.Max(maxV, p.V)
		maxAbs = math.Max(maxAbs, math.Max(math.Abs(p.U), math.Abs(p.V)))
	}
	floor := 1e-9*math.Hypot(maxU-minU, maxV-minV) + 4*(math.Nextafter(maxAbs, math.Inf(1))-maxAbs)
	if proofbound.IsNonFinite(floor) {
		return fmt.Errorf("%w: bounded section clearance is nonfinite", decaderr.ErrUnsupported)
	}
	for i := range 2 {
		gate := sag[i] + sag[i+2] + floor
		distance := sectionSegmentDistance(pts[i], pts[i+1], pts[i+2], pts[(i+3)%4])
		if proofbound.IsNonFinite(gate) || proofbound.IsNonFinite(distance) || distance <= gate {
			return fmt.Errorf("%w: bounded section clearance cannot be established", decaderr.ErrUnsupported)
		}
	}
	return nil
}
