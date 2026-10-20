package capblend

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// ProveLoftBandVolume covers the narrow loft cap band: straight held outer
// walls, zero or one unmodified circular bore, and the complete outer loop
// chamfered on both caps. Outside the two cap-height slabs, the ordinary cap-blend
// chord and vertex-motion proof applies. The reflex apex fans live wholly in
// those slabs. Their unknown exact angular windows cannot escape the cylinder
// containing both the analytic body and its held mesh, so the whole cylinder
// slab volume bounds every unpaired reflex cell and cap sliver at once.
// This deliberately coarse term never substitutes zero motion for an apex
// sample without charging the slab that contains it. A term this build cannot
// state refuses (docs/tessellation-design.md §12) rather than publishing a
// symmetric-difference bound a Boolean would then compose into a result.
func ProveLoftBandVolume(ctx context.Context, mesh *topo.Mesh, body *topo.Body,
	cbp Payload, lms []LoopMesh, motion []float64) error {
	if len(lms) < 1 || len(lms) > 2 || !cbp.StartLoops[0] || !cbp.EndLoops[0] ||
		len(cbp.StartLoops) != 1 || len(cbp.EndLoops) != 1 {
		return fmt.Errorf("%w: loft cap-band volume proof requires both complete outer bands and at most one bore", decaderr.ErrUnsupported)
	}
	if len(lms) == 2 && (!lms[1].Whole || len(lms[1].Walks) != 1 || !lms[1].Walks[0].IsCircular()) {
		return fmt.Errorf("%w: loft cap-band volume proof requires an unchanged circular bore", decaderr.ErrUnsupported)
	}
	for _, walk := range lms[0].Walks {
		if !walk.IsLine() {
			return fmt.Errorf("%w: loft cap-band volume proof requires straight held outer walls", decaderr.ErrUnsupported)
		}
	}
	for i := range lms[0].ArcCount {
		if lms[0].ArcCount[i] <= 0 && lms[0].Joins[i].Arc {
			return fmt.Errorf("%w: loft cap-band reflex corner has no chorded apex fan", decaderr.ErrUnsupported)
		}
	}
	outer := lms[0]
	motionMax, err := tessellation.LoftCapBandFiniteMotion(
		outer.ArcCount, outer.CapArcStart, outer.CapLoV, outer.CapHiV,
		len(outer.CapPts), mesh.Vertices, motion)
	if err != nil {
		return err
	}
	slab, err := tessellation.LoftCapBandSlabVolume(ctx,
		cbp.PrismLike(0, 0).Point(0, 0, cbp.Z0),
		body.Bounds.Min, body.Bounds.Max, body.Bounds.Bound.Base(),
		mesh.Vertices, cbp.AxialDelta(), mesh.Bound,
		cbp.Start.AxialUpper(), cbp.End.AxialUpper())
	if err != nil {
		return err
	}
	total, err := tessellation.SymDiff([]float64{
		ChordVolume(cbp, lms),
		proofbound.SweptVolumeAllow(motionMax,
			proofbound.PerturbedAreaUpper(mesh.Vertices, mesh.Triangles, motionMax)),
		slab,
		cbp.LoftSource.Proof.VolSymDiff,
	})
	if err != nil {
		return err
	}
	mesh.VolSymDiff = total
	mesh.SymDiffOK = true
	return nil
}
