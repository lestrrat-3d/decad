package prismtess

import (
	"context"
	"math"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/massmoment"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/units"
)

// circleMeasuresReplay bounds area and occupied-volume error for the
// zero-displacement recorded cylinder. The mesh must separately pass contact audits.
func circleMeasuresReplay(ctx context.Context, p prismrecord.Payload, seg sectionrecord.CircleSeg, samples tessellation.ChordSamples[*topo.Face], mesh *topo.Mesh, store []float64, charge massmoment.MapCharge, b *proofbound.ReplayBudget) (float64, float64, error) {
	if b == nil {
		return 0, 0, decaderr.ErrUnsupported
	}
	if err := b.SpendWork(ctx, 128); err != nil {
		return 0, 0, err
	}
	if mesh == nil || len(store) != len(mesh.Vertices) || p.SectionDelta != 0 || p.Z0Delta != 0 || p.Z1Delta != 0 || !(p.Z1 > p.Z0) || !seg.CCW || seg.TStart != 0 || seg.TEnd != 1 || seg.Radius.Unit() != units.Millimeter || charge.Stretch < 0 || charge.Volume < 0 || proofbound.IsNonFinite(charge.Stretch) || proofbound.IsNonFinite(charge.Volume) {
		return 0, 0, decaderr.ErrUnsupported
	}
	radius, err := seg.Radius.In(units.Millimeter)
	if err != nil || !(radius > 0) || proofbound.IsNonFinite(radius) {
		return 0, 0, decaderr.ErrUnsupported
	}
	height := math.Nextafter(math.Abs(p.Z1-p.Z0), math.Inf(1))
	if proofbound.IsNonFinite(height) || !(height > 0) {
		return 0, 0, decaderr.ErrUnsupported
	}
	for _, v := range [...]float64{samples.WallSlack, samples.CapSlack, samples.SegmentArea} {
		if v < 0 || proofbound.IsNonFinite(v) {
			return 0, 0, decaderr.ErrUnsupported
		}
	}
	local := proofbound.AbsSumUpper(samples.WallSlack, samples.CapSlack, samples.CapSlack)
	trueArea := proofbound.ProductUpper(proofbound.ProductUpper(proofbound.TwoPiUpper(), radius), proofbound.AbsSumUpper(radius, height))
	area := proofbound.AbsSumUpper(proofbound.ProductUpper(local, proofbound.AbsSumUpper(1, charge.Stretch)), proofbound.ProductUpper(charge.Stretch, trueArea))
	allowance, err := storeAreaReplay(ctx, mesh, store, b)
	if err != nil {
		return 0, 0, err
	}
	area = proofbound.AbsSumUpper(area, allowance)
	maxStore := 0.0
	for _, v := range store {
		if err = b.SpendWork(ctx, 4); err != nil {
			return 0, 0, err
		}
		if v < 0 || proofbound.IsNonFinite(v) {
			return 0, 0, decaderr.ErrUnsupported
		}
		maxStore = max(maxStore, v)
	}
	perturbed, err := perturbedAreaReplay(ctx, mesh, maxStore, b)
	if err != nil {
		return 0, 0, err
	}
	volume := proofbound.AbsSumUpper(proofbound.ProductUpper(proofbound.ProductUpper(height, samples.SegmentArea), proofbound.AbsSumUpper(1, charge.Volume)), proofbound.SweptVolumeAllow(maxStore, perturbed))
	if proofbound.IsNonFinite(area) || proofbound.IsNonFinite(volume) {
		return 0, 0, decaderr.ErrUnsupported
	}
	return area, volume, nil
}
