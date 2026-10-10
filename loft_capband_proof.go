package decad

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// proveLoftCapBandVolume covers the narrow loft cap band: straight held outer
// walls, one unmodified circular bore, and the complete outer loop chamfered
// on both caps. Outside the two cap-height slabs, the ordinary cap-blend
// chord and vertex-motion proof applies. The reflex apex fans live wholly in
// those slabs. Their unknown exact angular windows cannot escape the cylinder
// containing both the analytic body and its held mesh, so the whole cylinder
// slab volume bounds every unpaired reflex cell and cap sliver at once.
// This deliberately coarse term never substitutes zero motion for an apex
// sample without charging the slab that contains it.
func proveLoftCapBandVolume(ctx context.Context, mesh *Mesh, body *Body,
	cbp capBlendPayload, lms []capBlendLoopMesh, motion []float64) error {
	if len(lms) != 2 || !cbp.startLoops[0] || !cbp.endLoops[0] ||
		len(cbp.startLoops) != 1 || len(cbp.endLoops) != 1 || !lms[1].whole ||
		len(lms[1].walks) != 1 || !lms[1].walks[0].IsCircular() {
		return fmt.Errorf("%w: loft cap-band volume proof requires two complete outer bands and one circular bore", ErrUnsupported)
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
	motionMax, err := loftCapBandFiniteMotion(lms[0], mesh.vertices, motion)
	if err != nil {
		return err
	}
	slab, err := loftCapBandSlabVolume(ctx, body, mesh, cbp)
	if err != nil {
		return err
	}
	return publishSymDiff(mesh, []float64{
		capBlendChordVolume(cbp, lms),
		proofbound.SweptVolumeAllow(motionMax,
			proofbound.PerturbedAreaUpper(mesh.vertices, mesh.triangles, motionMax)),
		slab,
		cbp.loftSource.proof.VolSymDiff,
	})
}

// loftCapBandFiniteMotion replaces only the two cap-level copies of each
// reflex connector station. capBlendVertices appends one motion entry when it
// appends each vertex, so its vertex indices also index motion.
func loftCapBandFiniteMotion(outer capBlendLoopMesh, vertices []r3.Vec, motion []float64) (float64, error) {
	if len(motion) != len(vertices) || len(outer.capLoV) != len(outer.capPts) ||
		len(outer.capHiV) != len(outer.capPts) {
		return 0, fmt.Errorf("%w: loft cap-band motion does not match its vertices", ErrUnsupported)
	}
	reflex := make([]bool, len(motion))
	for i, count := range outer.arcCount {
		if count == 0 {
			continue
		}
		start := outer.capArcStart[i]
		if start < 0 || start+count > len(outer.capPts) {
			return 0, fmt.Errorf("%w: loft cap-band reflex stations are incomplete", ErrUnsupported)
		}
		for k := start; k < start+count; k++ {
			lo, hi := outer.capLoV[k], outer.capHiV[k]
			if lo < 0 || lo >= len(motion) || hi < 0 || hi >= len(motion) || lo == hi || reflex[lo] || reflex[hi] {
				return 0, fmt.Errorf("%w: loft cap-band reflex vertices are incomplete", ErrUnsupported)
			}
			reflex[lo], reflex[hi] = true, true
		}
	}
	worst := 0.0
	for i, value := range motion {
		if reflex[i] {
			if !math.IsInf(value, 1) {
				return 0, fmt.Errorf("%w: loft cap-band reflex motion is not an apex sample", ErrUnsupported)
			}
			motion[i] = 0
			continue
		}
		if proofbound.IsNonFinite(value) || value < 0 {
			return 0, fmt.Errorf("%w: loft cap-band vertex motion is not finite", ErrUnsupported)
		}
		worst = math.Max(worst, value)
	}
	return worst, nil
}

// loftCapBandSlabVolume bounds the uncertain part by two axial slabs inside
// one cylinder. A world-space sphere enclosing the body and mesh gives a
// radius for the cylinder around any sweep axis, including after placement.
// The body box's own error is charged at each coordinate; the mesh vertices
// are the exact held points. Both cap levels are widened by their inherited
// axial error and the mesh's full displacement, so the slab includes all
// floating-point level motion as well as every reflex apex fan.
func loftCapBandSlabVolume(ctx context.Context, body *Body, mesh *Mesh, cbp capBlendPayload) (float64, error) {
	center := cbp.prismLike(0, 0).point(0, 0, cbp.z0)
	boxError := body.bounds.Bound.Base()
	axisMax := 0.0
	include := func(p [3]float64, errorBound float64) {
		coords := [3]float64{center.X, center.Y, center.Z}
		for i := range p {
			diff := proofbound.BoundedSub(proofbound.MeasuredScalar(p[i], errorBound),
				proofbound.ExactScalar(coords[i]))
			axisMax = math.Max(axisMax, proofbound.AbsSumUpper(diff.Value, diff.Bound))
		}
	}
	include([3]float64{body.bounds.Min.X, body.bounds.Min.Y, body.bounds.Min.Z}, boxError)
	include([3]float64{body.bounds.Max.X, body.bounds.Max.Y, body.bounds.Max.Z}, boxError)
	for i, p := range mesh.vertices {
		if i%1024 == 0 {
			if err := ctx.Err(); err != nil {
				return 0, err
			}
		}
		include([3]float64{p.X, p.Y, p.Z}, 0)
	}
	radius := proofbound.Radius3D(axisMax)
	// The true band and its mesh can each move on both sides of a nominal
	// level. Two copies of the inherited axial and tessellation allowances
	// cover those opposite motions before the two cap slabs are summed.
	levelError := proofbound.AbsSumUpper(cbp.axialDelta(), mesh.bound)
	startHeight := proofbound.AbsSumUpper(cbp.start.axialUpper(), proofbound.ProductUpper(2, levelError))
	endHeight := proofbound.AbsSumUpper(cbp.end.axialUpper(), proofbound.ProductUpper(2, levelError))
	area := proofbound.ProductUpper(4, proofbound.ProductUpper(radius, radius))
	volume := proofbound.ProductUpper(area, proofbound.AbsSumUpper(startHeight, endHeight))
	if proofbound.IsNonFinite(volume) {
		return 0, fmt.Errorf("%w: loft cap-band slab has no finite volume bound", ErrUnsupported)
	}
	return volume, nil
}
