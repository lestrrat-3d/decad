package booleaneval

import (
	"context"
	"errors"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/facetproof"
	"github.com/lestrrat-3d/decad/internal/gaptol"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"
)

// retryRadialInsetUnion proposes one alternative triangulation before a gap shift.
// The public operands, order and their source patches stay the same. Every
// ordinary Boolean gate still runs, and the result must certify positive
// overlap volume independently from its boundary classification.
func retryRadialInsetUnion(ctx context.Context, a, b *topo.Body, meshes OperandMesher,
	twins *operandTwins, selectedB *topo.Mesh, tolMM, dPair, chordScale float64) (Evaluation, bool, error) {
	if len(a.Lumps) != 1 || len(b.Lumps) != 1 {
		return Evaluation{}, false, nil
	}
	disk, original := b, selectedB
	if _, faceted := a.Payload.(facetedrecord.Payload); !faceted {
		if _, faceted = b.Payload.(facetedrecord.Payload); !faceted {
			return Evaluation{}, false, nil
		}
		disk, original = a, nil
	}
	if !disk.HasStrictPrismBuildSeal() {
		return Evaluation{}, false, nil
	}
	chord := operandChord(ctx, disk, tolMM*chordScale, chordDiameter(ctx, meshbool.OpUnion, a, b, dPair), chordScale)
	if original == nil {
		var err error
		original, _, err = meshes.TessellateOperand(ctx, disk, chord)
		if err != nil {
			return Evaluation{}, false, err
		}
	}
	budget := proofbound.NewReplayTrialBudget()
	alternative, ready, err := radialInsetMesh(ctx, disk, original, gaptol.Reach(ctx), budget)
	if err != nil || !ready {
		return Evaluation{}, false, err
	}
	alt := &radialInsetMesher{meshes, disk, alternative, chord, budget, false}
	result, err := evaluate(ctx, meshbool.OpUnion, a, b, alt, chordScale, nil, nil, twins, nil, nil, nil, nil, 0)
	if err != nil {
		if ctx.Err() != nil {
			return Evaluation{}, false, ctx.Err()
		}
		var limit *proofbound.WorkLimitError
		var expected *meshbool.BooleanExpectedError
		if errors.As(err, &limit) || !errors.As(err, &expected) {
			return Evaluation{}, false, err
		}
		return Evaluation{}, false, nil
	}
	if !radialOverlap(a, b, result) {
		return Evaluation{}, false, nil
	}
	return result, true, nil
}

type radialInsetMesher struct {
	OperandMesher
	body       *topo.Body
	mesh       *topo.Mesh
	chord      float64
	budget     *proofbound.ReplayBudget
	stockFirst bool
}

func (m *radialInsetMesher) TessellateOperand(ctx context.Context, body *topo.Body, chord float64) (*topo.Mesh, bool, error) {
	if body == m.body && chord == m.chord {
		return m.mesh, false, nil
	}
	return m.OperandMesher.TessellateOperand(ctx, body, chord)
}

// radialInsetMesh keeps the original disk, trims and axial interval. It
// changes only candidate facet vertices. Every triangular patch is paired by
// barycentric coordinates with the corresponding previously verified patch.
func radialInsetMesh(ctx context.Context, source *topo.Body, original *topo.Mesh, maxMove float64, budget *proofbound.ReplayBudget) (*topo.Mesh, bool, error) {
	if source == nil || !source.HasStrictPrismBuildSeal() || original == nil || !original.BoundaryOK || !original.SymDiffOK || (!(maxMove > 0) || proofbound.IsNonFinite(maxMove)) {
		return nil, false, nil
	}
	nv, nt := len(original.Vertices), len(original.Triangles)
	if (original.VertexBound != nil && len(original.VertexBound) != nv) || nv < 4 || nv > 4096 || nt < 4 || nt > 8192 || len(original.Source) != nt || len(original.FaceBound) != 3 {
		return nil, false, nil
	}
	p, ok := prismrecord.OwnedPayload(source).(prismrecord.Payload)
	if !ok || p.Xform != r3.Identity() || p.SurfaceResult || p.SectionDelta != 0 || len(p.Profile.Holes) != 0 || len(p.Profile.Outer.Segments) != 1 {
		return nil, false, nil
	}
	c, ok := p.Profile.Outer.Segments[0].(sectionrecord.CircleSeg)
	if !ok || !c.CCW || c.TStart != 0 || c.TEnd != 1 || !proofbound.FiniteVec(p.Frame.N()) {
		return nil, false, nil
	}
	radius := c.Radius.Base()
	if !(radius > 0) || proofbound.IsNonFinite(radius) {
		return nil, false, nil
	}
	sealed, err := source.ReplayPrismBuildPayload(ctx, p.Frame, p.Xform, p.Z0, p.Z1, p.Z0Delta, p.Z1Delta, p.SectionDelta, p.Z0Denoted, p.Z1Denoted, []sectionrecord.AnalyticProfileView{p.Profile.AnalyticSealView()}, budget)
	if err != nil || !sealed {
		return nil, false, err
	}
	center := p.Frame.Origin().Add(p.Frame.U().Scale(c.Center.U)).Add(p.Frame.V().Scale(c.Center.V))
	normal := p.Frame.N()
	axis := -1
	for i, n := range []float64{normal.X, normal.Y, normal.Z} {
		if n == 1 || n == -1 {
			if axis >= 0 {
				return nil, false, nil
			}
			axis = i
		} else if n != 0 {
			return nil, false, nil
		}
	}
	if axis < 0 || !proofbound.FiniteVec(center) {
		return nil, false, nil
	}
	// This fraction selects a candidate only. The measured movement and
	// subsequent proofs decide acceptance. The private operand still carries
	// the entire extra error even when its original chord request was smaller.
	desired := min(original.Bound/8, maxMove/100)
	scale := 1 - desired/radius
	if !(scale > 0 && scale < 1) {
		return nil, false, nil
	}
	if err := budget.ReserveBundle(ctx, uint64(nv)*128+uint64(nt)*64, uint64(nv)*8+uint64(nt)*2, uint64(nv)*1024+uint64(nt)*256); err != nil {
		return nil, false, err
	}
	out := *original
	out.Owner = nil
	out.ReplayFaceBounds = nil
	out.SetPlanarRecord(nil)
	out.Vertices = make([]r3.Vec, nv)
	for i, t := range original.Triangles {
		if i%128 == 0 {
			if e := ctx.Err(); e != nil {
				return nil, false, e
			}
		}
		if original.Source[i] == nil || original.Source[i].Body != source {
			return nil, false, nil
		}
		for _, id := range t {
			if id < 0 || id >= nv {
				return nil, false, nil
			}
		}
	}
	beta, err := topobuild.VertexBounds(original)
	if err != nil {
		return nil, false, err
	}
	if len(beta) != nv {
		return nil, false, nil
	}
	out.VertexBound = make([]float64, nv)

	out.FaceBound = make(map[*topo.Face]float64, 3)
	out.Bound = 0
	changed := make([]bool, nv)
	var worst float64
	for i, v := range original.Vertices {
		if i%128 == 0 {
			if e := ctx.Err(); e != nil {
				return nil, false, e
			}
		}
		if !proofbound.FiniteVec(v) {
			return nil, false, nil
		}
		q := center.Add(v.Sub(center).Scale(scale))
		switch axis {
		case 0:
			q.X = v.X
		case 1:
			q.Y = v.Y
		case 2:
			q.Z = v.Z
		}
		if !proofbound.FiniteVec(q) {
			return nil, false, nil
		}
		d := proofbound.DvLenUpper(proofbound.HeldDelta(q, v))
		if !(d <= maxMove) {
			return nil, false, nil
		}

		out.Vertices[i] = q

		worst = max(worst, d)
		changed[i] = q != v
	}
	if !(worst > 0) {
		return nil, false, nil
	}
	for i, old := range beta {
		if old < 0 || proofbound.IsNonFinite(old) {
			return nil, false, nil
		}
		out.VertexBound[i] = proofbound.AbsSumUpper(old, worst)
	}
	out.CoordBound = proofbound.AbsSumUpper(original.CoordBound, worst)
	areaDelta := 0.0
	for i, t := range out.Triangles {
		if i%128 == 0 {
			if e := ctx.Err(); e != nil {
				return nil, false, e
			}
		}
		f := out.Source[i]
		if f == nil || f.Body != source {
			return nil, false, nil
		}
		if old, ok := original.FaceBound[f]; ok {
			out.FaceBound[f] = max(out.FaceBound[f], proofbound.AbsSumUpper(old, worst))
		} else {
			return nil, false, nil
		}
		for _, v := range t {
			if v < 0 || v >= nv {
				return nil, false, nil
			}
			out.FaceBound[f] = max(out.FaceBound[f], out.VertexBound[v])
		}
		out.Bound = max(out.Bound, out.FaceBound[f])
	}
	// The final float mesh must still be embedded and oriented. The original
	// arrays and all their cached records are left untouched.
	audit, admitted, err := facetproof.AuditFacetedMeshReplay(ctx, budget.WorkBudget(ctx), out.Vertices, out.Triangles)
	if err != nil || !admitted {
		return nil, false, err
	}
	h := meshbool.HeldRounding{Verts: out.Vertices, Tris: out.Triangles, Exact: audit.XVerts, Moved: changed, Movable: make([]bool, nv)}
	folded, err := meshbool.HasHeldFoldReplay(ctx, h, budget.WorkBudget(ctx))
	if err != nil || folded {
		return nil, false, err
	}
	areaUpper, err := proofbound.PerturbedAreaUpperReplay(ctx, budget.WorkBudget(ctx), out.Vertices, out.Triangles, worst)
	swept := proofbound.SweptVolumeAllow(worst, areaUpper)
	for i, t := range out.Triangles {
		if i%128 == 0 {
			if e := ctx.Err(); e != nil {
				return nil, false, e
			}
		}
		v := proofbound.PerturbedTriangleAreaAllow(out.Vertices[t[0]], out.Vertices[t[1]], out.Vertices[t[2]], worst)
		areaDelta = proofbound.AbsSumUpper(areaDelta, v)
	}
	if err != nil {
		return nil, false, err
	}
	out.VolSymDiff = proofbound.AbsSumUpper(original.VolSymDiff, swept)
	out.AreaSlack = proofbound.AbsSumUpper(original.AreaSlack, areaDelta)
	if !proofbound.FiniteVec(r3.Vec{X: out.VolSymDiff, Y: out.AreaSlack, Z: out.Bound}) {
		return nil, false, nil
	}
	return &out, true, nil
}

// The mesh union's occupied-volume bound comes from both independently
// verified operand symmetric-difference bounds and the stitch rounding.
// Its overlap lower bound remains valid independently of a boundary contact.
func radialOverlap(a, b *topo.Body, out Evaluation) bool {
	if out.Audit == nil || len(out.Audit.Members) != 1 {
		return false
	}
	lower := new(big.Rat)
	for _, v := range []float64{a.Volume.Value.Base(), b.Volume.Value.Base()} {
		q := new(big.Rat).SetFloat64(v)
		if q == nil || q.Sign() <= 0 {
			return false
		}
		lower.Add(lower, q)
	}
	for _, v := range []float64{a.Volume.Bound.Base(), b.Volume.Bound.Base(), out.Volume, out.VolumeBound} {
		q := new(big.Rat).SetFloat64(v)
		if q == nil || q.Sign() < 0 {
			return false
		}
		lower.Sub(lower, q)
	}
	return lower.Sign() > 0
}
