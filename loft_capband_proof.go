package decad

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/tessellation"
)

// proveLoftCapBandVolume covers the narrow loft cap band: straight held outer
// walls, zero or one unmodified circular bore, and the complete outer loop
// chamfered on both caps. Outside the two cap-height slabs, the ordinary cap-blend
// chord and vertex-motion proof applies. The reflex apex fans live wholly in
// those slabs. Their unknown exact angular windows cannot escape the cylinder
// containing both the analytic body and its held mesh, so the whole cylinder
// slab volume bounds every unpaired reflex cell and cap sliver at once.
// This deliberately coarse term never substitutes zero motion for an apex
// sample without charging the slab that contains it.
func proveLoftCapBandVolume(ctx context.Context, mesh *Mesh, body *Body,
	cbp capBlendPayload, lms []capBlendLoopMesh, motion []float64) error {
	if len(lms) < 1 || len(lms) > 2 || !cbp.StartLoops[0] || !cbp.EndLoops[0] ||
		len(cbp.StartLoops) != 1 || len(cbp.EndLoops) != 1 {
		return fmt.Errorf("%w: loft cap-band volume proof requires both complete outer bands and at most one bore", ErrUnsupported)
	}
	if len(lms) == 2 && (!lms[1].whole || len(lms[1].walks) != 1 || !lms[1].walks[0].IsCircular()) {
		return fmt.Errorf("%w: loft cap-band volume proof requires an unchanged circular bore", ErrUnsupported)
	}
	for _, walk := range lms[0].walks {
		if !walk.IsLine() {
			return fmt.Errorf("%w: loft cap-band volume proof requires straight held outer walls", ErrUnsupported)
		}
	}
	for i := range lms[0].arcCount {
		if lms[0].arcCount[i] <= 0 && lms[0].joins[i].Arc {
			return fmt.Errorf("%w: loft cap-band reflex corner has no chorded apex fan", ErrUnsupported)
		}
	}
	outer := lms[0]
	motionMax, err := tessellation.LoftCapBandFiniteMotion(
		outer.arcCount, outer.capArcStart, outer.capLoV, outer.capHiV,
		len(outer.capPts), mesh.n.Vertices, motion)
	if err != nil {
		return err
	}
	slab, err := tessellation.LoftCapBandSlabVolume(ctx,
		cbp.PrismLike(0, 0).Point(0, 0, cbp.Z0),
		body.n.Bounds.Min, body.n.Bounds.Max, body.n.Bounds.Bound.Base(),
		mesh.n.Vertices, cbp.AxialDelta(), mesh.n.Bound,
		cbp.Start.AxialUpper(), cbp.End.AxialUpper())
	if err != nil {
		return err
	}
	return publishSymDiff(mesh, []float64{
		capBlendChordVolume(cbp, lms),
		proofbound.SweptVolumeAllow(motionMax,
			proofbound.PerturbedAreaUpper(mesh.n.Vertices, mesh.n.Triangles, motionMax)),
		slab,
		cbp.LoftSource.Proof.VolSymDiff,
	})
}
