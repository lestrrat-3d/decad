package capblend

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/capband"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/decad/internal/triangulation"
)

// This builder owns tessellation increment T7 (docs/tessellation-reach-design.md
// §7). Each wall walk is chorded once at one count. The trimmed side wall,
// chamfer band and cap contour reuse that count and its vertex indices, so
// their shared boundary closes without welding or dropping cells.

// TessellateMesh builds and audits the cap-blend mesh node. Its body and mesh
// arguments must not be nil. Loft occupied-volume proof stays with the root
// handle and runs after this common build returns.
func TessellateMesh(ctx context.Context, b *topo.Body, cbp Payload, chord float64, proveVolume bool, mesh *topo.Mesh) ([]LoopMesh, []float64, error) {
	if err := ctx.Err(); err != nil {
		return nil, nil, err
	}
	if b == nil {
		return nil, nil, fmt.Errorf("%w: cap-blend mesh body is nil", decaderr.ErrDegenerate)
	}
	if mesh == nil {
		return nil, nil, fmt.Errorf("%w: cap-blend mesh output is nil", decaderr.ErrDegenerate)
	}
	byRole := map[string]*topo.Face{}
	for _, f := range topobuild.Faces(b) {
		for _, o := range f.Origins {
			byRole[o.Role] = f
		}
	}
	faceOfRole := func(role string) (*topo.Face, error) {
		f, ok := byRole[role]
		if !ok {
			return nil, fmt.Errorf(`%w: the body carries no face for role %q`, decaderr.ErrDegenerate, role)
		}
		return f, nil
	}
	geomOfRole := map[string]capband.Patch{}
	for _, p := range cbp.Patches {
		geomOfRole[p.Role] = p.Geom
	}
	capStart, err := faceOfRole(provenance.RoleCapStart)
	if err != nil {
		return nil, nil, err
	}
	capEnd, err := faceOfRole(provenance.RoleCapEnd)
	if err != nil {
		return nil, nil, err
	}

	pl := cbp.PrismLike(0, 0)
	work := freeform.NewFreeformWork()
	budget := proofbound.NewWorkBudget(ctx)
	loops := cbp.Loops()
	lms := make([]LoopMesh, len(loops))
	for li, loop := range loops {
		lm, err := ChordLoop(ctx, budget, cbp, li, loop, chord, work)
		if err != nil {
			return nil, nil, err
		}
		lms[li] = lm
	}

	if proveVolume {
		for li := range lms {
			if err := CapMotion(budget, cbp, &lms[li]); err != nil {
				return nil, nil, err
			}
		}
	}

	store, motion, err := Vertices(budget, cbp, pl, lms, mesh, proveVolume)
	if err != nil {
		return nil, nil, err
	}
	if _, err := tessellation.StoreMax(store); err != nil {
		return nil, nil, err
	}

	// faceExtra is every displacement a face carries BESIDE its own vertices'
	// store term, which tessellation.ComposeCapBlendBounds adds per face at the end.
	faceExtra := map[*topo.Face]float64{}
	bump := func(f *topo.Face, v float64) {
		if v > faceExtra[f] {
			faceExtra[f] = v
		}
	}

	// Trimmed side walls: the prism's own cells between the loop's two rings.
	// A draft view has none: its band runs the whole sweep, and its one side
	// ring is the near cap's rim (capblend.DraftNearRing).
	if !cbp.Draft {
		for li := range lms {
			lm := &lms[li]
			axial := math.Max(lm.ZLo.Bound, lm.ZHi.Bound)
			height := math.Abs(lm.ZHi.Value - lm.ZLo.Value)
			n := len(lm.Walks)
			for i, w := range lm.Walks {
				if err := budget.Step(); err != nil {
					return nil, nil, err
				}
				face, err := faceOfRole(fmt.Sprintf("side(%d,%d)", lm.LoopIndex, w.Segs[0]))
				if err != nil {
					return nil, nil, err
				}
				bump(face, proofbound.AbsSumUpper(lm.SideSag[i], axial))
				mesh.AreaSlack = proofbound.AbsSumUpper(mesh.AreaSlack, tessellation.WalkWallSlack(w.SegmentWalk, lm.Count[i], height))
				for k := range lm.Count[i] {
					g0 := lm.SideStart[i] + k
					g1 := lm.SideStart[i] + k + 1
					if k == lm.Count[i]-1 {
						g1 = lm.SideStart[(i+1)%n]
					}
					topobuild.AddTriangle(mesh, [3]int{lm.SideLo[g0], lm.SideLo[g1], lm.SideHi[g1]}, face)
					topobuild.AddTriangle(mesh, [3]int{lm.SideLo[g0], lm.SideHi[g1], lm.SideHi[g0]}, face)
				}
			}
		}
	}

	// Chamfer bands: the patch cells between the side ring at the band's own
	// side level and the cap contour ring at its cap level.
	for li := range lms {
		lm := &lms[li]
		if lm.OnStart {
			if err := EmitBand(budget, mesh, cbp, lm, true, faceOfRole, geomOfRole, bump, nil); err != nil {
				return nil, nil, err
			}
		}
		if lm.OnEnd {
			if err := EmitBand(budget, mesh, cbp, lm, false, faceOfRole, geomOfRole, bump, nil); err != nil {
				return nil, nil, err
			}
		}
	}

	// Cap faces. Each is bounded, per loop, by that loop's contour ring where it
	// is chamfered on this cap and by its original ring where it is not — the
	// mixed profile capblend.Build builds the same two faces from.
	if err := emitCap(ctx, mesh, cbp, lms, true, capStart, bump); err != nil {
		return nil, nil, err
	}
	if err := emitCap(ctx, mesh, cbp, lms, false, capEnd, bump); err != nil {
		return nil, nil, err
	}

	// A reflected placement flips handedness, turning every counter-clockwise
	// winding clockwise; reversing them restores outward orientation.
	if pl.Reflected() {
		for i := range mesh.Triangles {
			mesh.Triangles[i][1], mesh.Triangles[i][2] = mesh.Triangles[i][2], mesh.Triangles[i][1]
		}
	}

	if err := tessellation.RequireClosedMesh(mesh.Triangles); err != nil {
		return nil, nil, fmt.Errorf(`%w: this %s's cells do not close into a watertight boundary`, decaderr.ErrUnsupported, cbp.Noun())
	}
	if err := tessellation.RequireVertexLinks(ctx, len(mesh.Vertices), mesh.Triangles); err != nil {
		return nil, nil, err
	}
	if err := tessellation.RequireCapBlendFacetAreas(mesh.Vertices, mesh.Triangles, cbp.Noun()); err != nil {
		return nil, nil, err
	}
	if tessellation.OrientationSign(mesh.Vertices, mesh.Triangles, pl.Point(0, 0, cbp.Z0)) <= 0 {
		return nil, nil, fmt.Errorf(`%w: this %s's assembled cells do not enclose a positive volume`, decaderr.ErrUnsupported, cbp.Noun())
	}

	faceBounds, err := tessellation.ComposeCapBlendBounds(mesh.Source, mesh.Triangles, faceExtra, store, cbp.Noun())
	if err != nil {
		return nil, nil, err
	}
	for f, bound := range faceBounds {
		topobuild.SetFaceBound(mesh, f, bound)
	}
	mesh.AreaSlack = proofbound.AbsSumUpper(mesh.AreaSlack,
		tessellation.StoreAreaAllow(mesh.Vertices, mesh.Triangles, store))
	if cbp.LoftSource != nil {
		for face, bound := range mesh.FaceBound {
			topobuild.SetFaceBound(mesh, face, proofbound.AbsSumUpper(bound, cbp.LoftSource.Proof.FacetDeparture))
		}
		mesh.AreaSlack = proofbound.AbsSumUpper(mesh.AreaSlack, cbp.LoftSource.Proof.AreaSlack)
	}
	if proofbound.IsNonFinite(mesh.AreaSlack) {
		return nil, nil, fmt.Errorf(`%w: this %s mesh states no finite area slack`, decaderr.ErrUnsupported, cbp.Noun())
	}
	if !proveVolume {
		// The level withholds every volume proof (tessellateContext's
		// withholdProofs), so none is built.
		return lms, motion, nil
	}
	if cbp.LoftSource != nil {
		return lms, motion, nil
	}
	refusal, err := OccupiedVolumeAdmission(budget, cbp)
	if err != nil {
		return nil, nil, err
	}
	if refusal != nil {
		// docs/tessellation-reach-design.md §7: this band's cells are not proven
		// to reproduce the exact offset family slice by slice, so the mesh serves
		// export alone and requireVolumeProvingPayload/meshbool.OperandSymDiff refuse it
		// with the same reason.
		mesh.SymDiffOK = false
		return lms, motion, nil //nolint:nilerr // refusal is the admission result; err was checked above
	}
	// Occupied volume (docs/tessellation-reach-design.md §7): the ideal
	// polyhedron B1 differs from the body by its chord polygons' circular
	// segments, slice by slice, and the held mesh differs from B1 by its
	// vertices' motion alone, since the two share one triangle index set.
	motionMax, err := tessellation.StoreMax(motion)
	if err != nil {
		return nil, nil, err
	}
	terms := []float64{
		ChordVolume(cbp, lms),
		proofbound.SweptVolumeAllow(motionMax, proofbound.PerturbedAreaUpper(mesh.Vertices, mesh.Triangles, motionMax)),
	}
	total, err := tessellation.SymDiff(terms)
	if err != nil {
		return nil, nil, err
	}
	mesh.VolSymDiff = total
	mesh.SymDiffOK = true
	return lms, motion, nil
}

// Vertices writes every loop's mesh vertices. Every sample of a loop
// owns one vertex per level the loop reaches: the side ring at zLo and zHi, and
// the cap contour ring at whichever cap(s) the selection named. store is
// docs/tessellation-reach-design.md §3's deltaStore, one entry per vertex.
//
// motion, built only when proveVolume, is §7's per-vertex displacement from
// the ideal polyhedron B1, a parallel array: the vertex's plane-local
// displacement (the side station's own bound, or the cap station's capMotion),
// the rounding prismrecord.Payload.PointRound measures, and the proven bound on the level
// it stands at, summed outward. The rounding is measured once per vertex and
// read by both arrays.
func Vertices(budget *proofbound.WorkBudget, cbp Payload, pl prismrecord.Payload, lms []LoopMesh, mesh *topo.Mesh, proveVolume bool) ([]float64, []float64, error) {
	var store, motion []float64
	addVertex := func(p sectionrecord.Point2, z, plane float64) (int, float64) {
		held := pl.Point(p.U, p.V, z)
		mesh.Vertices = append(mesh.Vertices, held)
		round := pl.PointRound(p.U, p.V, z, held)
		store = append(store, proofbound.AbsSumUpper(plane, round))
		return len(mesh.Vertices) - 1, round
	}
	startLevel := cbp.CapBandLevel(cbp.Z0, 1)
	endLevel := cbp.CapBandLevel(cbp.Z1, -1)
	for li := range lms {
		lm := &lms[li]
		var sideErr error
		if cbp.Draft {
			lm.SideLo, lm.SideHi, sideErr = DraftNearRing(
				budget, lm.SidePts, lm.SideBound, lm.ZLo, addVertex, &motion, proveVolume)
		} else {
			lm.SideLo, lm.SideHi, sideErr = SideRing(
				budget, lm.SidePts, lm.SideBound, lm.ZLo, lm.ZHi, addVertex, &motion, proveVolume)
		}
		if sideErr != nil {
			return nil, nil, sideErr
		}
		if len(lm.CapPts) == 0 {
			continue
		}
		lm.CapLoV = make([]int, len(lm.CapPts))
		lm.CapHiV = make([]int, len(lm.CapPts))
		for j, p := range lm.CapPts {
			if err := budget.Step(); err != nil {
				return nil, nil, err
			}
			plane := proofbound.WalkEndBoundAllow(lm.CapBound[j])
			var round float64
			if lm.OnStart {
				lm.CapLoV[j], round = addVertex(p, cbp.Z0, plane)
				if proveVolume {
					motion = append(motion, proofbound.AbsSumUpper(lm.CapMotion[j], round, startLevel.Bound))
				}
			}
			if lm.OnEnd {
				lm.CapHiV[j], round = addVertex(p, cbp.Z1, plane)
				if proveVolume {
					motion = append(motion, proofbound.AbsSumUpper(lm.CapMotion[j], round, endLevel.Bound))
				}
			}
		}
	}
	return store, motion, nil
}

// emitCap triangulates one cap face over the ring bounding it per loop:
// the offset contour ring where that loop is chamfered on this cap, and the
// original ring where it is not. The rings must clear one another by more than
// their own sagitta tubes, or a cap at this tolerance cannot prove it represents
// the region's topology and refuses rather than return a pinched mesh.
func emitCap(ctx context.Context, m *topo.Mesh, cbp Payload, lms []LoopMesh, start bool, face *topo.Face, bump func(*topo.Face, float64)) error {
	var pts []sectionrecord.Point2
	var vtx []int
	var loopIdx [][]int
	var sag []float64
	trim, delta := 0.0, 0.0
	for li := range lms {
		lm := &lms[li]
		chamfered := lm.OnStart
		if !start {
			chamfered = lm.OnEnd
		}
		ringPts, ringV := lm.SidePts, lm.SideLo
		ringSag := tessellation.CapBlendRingSagitta(lm.Proof(), false)
		if !start {
			ringV = lm.SideHi
		}
		if chamfered {
			ringPts, ringV = lm.CapPts, lm.CapLoV
			if !start {
				ringV = lm.CapHiV
			}
			ringSag = tessellation.CapBlendRingSagitta(lm.Proof(), true)
			d, ok := cbp.BandDelta[BandKey{Loop: lm.LoopIndex, Start: start}]
			if !ok {
				return fmt.Errorf(`%w: the payload states no contour displacement for the chamfer band on loop %d`, decaderr.ErrDegenerate, lm.LoopIndex)
			}
			delta = math.Max(delta, d)
			if cbp.Fillet {
				ringPts, ringV = tessellation.CompactCapBlendRing(ringPts, ringV)
			}
		}
		base := len(pts)
		pts = append(pts, ringPts...)
		vtx = append(vtx, ringV...)
		idx := make([]int, len(ringPts))
		for k := range ringPts {
			idx[k] = base + k
		}
		loopIdx = append(loopIdx, idx)
		sag = append(sag, ringSag)
		trim = math.Max(trim, ringSag)
	}
	if err := tessellation.RequireLoopClearance(ctx, pts, loopIdx, sag); err != nil {
		return err
	}
	// This cap's own chord-versus-curve deficit: the planar area between each
	// bounding ring and the curve it chords, one term per loop, the same charge
	// tessellation.WalkAreaSlack makes per cap for a prism.
	for li := range lms {
		lm := &lms[li]
		chamfered := lm.OnStart
		if !start {
			chamfered = lm.OnEnd
		}
		m.AreaSlack = proofbound.AbsSumUpper(m.AreaSlack,
			tessellation.CapBlendRingSegmentArea(lm.Proof(), chamfered, cbp.LoopOffset(lm.LoopIndex)))
	}
	tris, err := triangulation.Triangulate(ctx, pts, loopIdx)
	if err != nil {
		return err
	}
	for _, tri := range tris {
		if start {
			topobuild.AddTriangle(m, [3]int{vtx[tri[0]], vtx[tri[2]], vtx[tri[1]]}, face)
			continue
		}
		topobuild.AddTriangle(m, [3]int{vtx[tri[0]], vtx[tri[1]], vtx[tri[2]]}, face)
	}
	axial := cbp.Z0Delta
	if !start {
		axial = cbp.Z1Delta
	}
	bump(face, proofbound.AbsSumUpper(trim, delta, axial))
	return nil
}
