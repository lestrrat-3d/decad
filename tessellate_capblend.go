package decad

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/capband"
	"github.com/lestrrat-3d/decad/internal/capblend"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/decad/internal/triangulation"

	"github.com/lestrrat-3d/decad/internal/proofbound"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/r3"
)

// This file is docs/tessellation-design.md §13's increment T7
// (docs/tessellation-reach-design.md §7): the cap-loop chamfer tessellator. It
// meshes a capBlendPayload's trimmed side walls, its chamfer band patches and
// its two cap faces, and publishes §2's per-face displacement bound and area
// slack for them.
//
// At VerifyAll it also publishes §7's slice-wise occupied-volume proof for a
// payload whose every band capblend.OccupiedVolumeAdmission
// admits — whole turns, line-line miters and exactly G1 joins: the chord
// polygon's circular-segment integral over the trimmed and band ranges
// (capblend.ChordVolume) plus proofbound.SweptVolumeAllow over each vertex's displacement
// from the ideal polyhedron B1. Every other band — a circular wall at a genuine
// miter, a reflex corner — leaves the mesh export-only with symDiffOK false,
// and the mesh boolean refuses that operand (boolean.go's
// requireVolumeProvingPayload and meshbool.OperandSymDiff) with the same reason.
// Substituting bound × held area for a missing proof is forbidden outright
// (tess §11).
//
// One structural fact shapes the whole file, and it is the answer to
// docs/modify-reach-design.md §12 Table DX row DX3's "a strip whose two sides
// disagree on sample density is not watertight": each wall walk is chorded
// ONCE, at a single count, and that one count is shared three ways — by the
// trimmed side wall's own two rings, by the band patch ruled off the ring at
// the chamfered end, and by the cap contour arc the band ends on. Nothing is
// snapped, welded or dropped to make the strips meet; a count that cannot be
// chosen refuses instead.

// tessellateCapBlend meshes a cap-loop chamfer result
// (docs/tessellation-reach-design.md §7). Below VerifyAll it builds no
// occupied-volume proof at all and leaves symDiffOK false, the level's own
// contract (tessellateContext's withholdProofs).
func tessellateCapBlend(ctx context.Context, b *Body, cbp capBlendPayload, chord float64, verify Verification) (*Mesh, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	byRole := map[string]*Face{}
	for _, f := range b.Faces() {
		for _, o := range f.Origins() {
			byRole[o.Role] = f
		}
	}
	faceOfRole := func(role string) (*Face, error) {
		f, ok := byRole[role]
		if !ok {
			return nil, fmt.Errorf(`%w: the body carries no face for role %q`, ErrDegenerate, role)
		}
		return f, nil
	}
	geomOfRole := map[string]capPatchGeom{}
	for _, p := range cbp.Patches {
		geomOfRole[p.Role] = p.Geom
	}
	capStart, err := faceOfRole(provenance.RoleCapStart)
	if err != nil {
		return nil, err
	}
	capEnd, err := faceOfRole(provenance.RoleCapEnd)
	if err != nil {
		return nil, err
	}

	pl := cbp.PrismLike(0, 0)
	work := freeform.NewFreeformWork()
	budget := proofbound.NewWorkBudget(ctx)
	loops := cbp.Loops()
	lms := make([]capblend.LoopMesh, len(loops))
	for li, loop := range loops {
		lm, err := capblend.ChordLoop(ctx, budget, cbp, li, loop, chord, work)
		if err != nil {
			return nil, err
		}
		lms[li] = lm
	}

	proveVolume := verify >= VerifyAll
	if proveVolume {
		for li := range lms {
			if err := capblend.CapMotion(budget, cbp, &lms[li]); err != nil {
				return nil, err
			}
		}
	}

	mesh := bornMesh(new(Mesh))
	store, motion, err := capBlendVertices(budget, cbp, pl, lms, mesh, proveVolume)
	if err != nil {
		return nil, err
	}
	if _, err := tessellation.StoreMax(store); err != nil {
		return nil, err
	}

	// faceExtra is every displacement a face carries BESIDE its own vertices'
	// store term, which tessellation.ComposeCapBlendBounds adds per face at the end.
	faceExtra := map[*topo.Face]float64{}
	bump := func(f *Face, v float64) {
		if v > faceExtra[faceNode(f)] {
			faceExtra[faceNode(f)] = v
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
					return nil, err
				}
				face, err := faceOfRole(fmt.Sprintf("side(%d,%d)", lm.LoopIndex, w.Segs[0]))
				if err != nil {
					return nil, err
				}
				bump(face, proofbound.AbsSumUpper(lm.SideSag[i], axial))
				mesh.n.AreaSlack = proofbound.AbsSumUpper(mesh.n.AreaSlack, tessellation.WalkWallSlack(w.SegmentWalk, lm.Count[i], height))
				for k := range lm.Count[i] {
					g0 := lm.SideStart[i] + k
					g1 := lm.SideStart[i] + k + 1
					if k == lm.Count[i]-1 {
						g1 = lm.SideStart[(i+1)%n]
					}
					topobuild.AddTriangle(&mesh.n, [3]int{lm.SideLo[g0], lm.SideLo[g1], lm.SideHi[g1]}, faceNode(face))
					topobuild.AddTriangle(&mesh.n, [3]int{lm.SideLo[g0], lm.SideHi[g1], lm.SideHi[g0]}, faceNode(face))
				}
			}
		}
	}

	// Chamfer bands: the patch cells between the side ring at the band's own
	// side level and the cap contour ring at its cap level.
	for li := range lms {
		lm := &lms[li]
		if lm.OnStart {
			if err := emitCapBand(budget, mesh, cbp, lm, true, faceOfRole, geomOfRole, bump, nil); err != nil {
				return nil, err
			}
		}
		if lm.OnEnd {
			if err := emitCapBand(budget, mesh, cbp, lm, false, faceOfRole, geomOfRole, bump, nil); err != nil {
				return nil, err
			}
		}
	}

	// Cap faces. Each is bounded, per loop, by that loop's contour ring where it
	// is chamfered on this cap and by its original ring where it is not — the
	// mixed profile capblend.Build builds the same two faces from.
	if err := emitCapBlendCap(ctx, mesh, cbp, lms, true, capStart, bump); err != nil {
		return nil, err
	}
	if err := emitCapBlendCap(ctx, mesh, cbp, lms, false, capEnd, bump); err != nil {
		return nil, err
	}

	// A reflected placement flips handedness, turning every counter-clockwise
	// winding clockwise; reversing them restores outward orientation.
	if pl.Reflected() {
		for i := range mesh.n.Triangles {
			mesh.n.Triangles[i][1], mesh.n.Triangles[i][2] = mesh.n.Triangles[i][2], mesh.n.Triangles[i][1]
		}
	}

	if err := tessellation.RequireClosedMesh(mesh.n.Triangles); err != nil {
		return nil, fmt.Errorf(`%w: this %s's cells do not close into a watertight boundary`, ErrUnsupported, cbp.Noun())
	}
	if err := tessellation.RequireVertexLinks(ctx, len(mesh.n.Vertices), mesh.n.Triangles); err != nil {
		return nil, err
	}
	if err := tessellation.RequireCapBlendFacetAreas(mesh.n.Vertices, mesh.n.Triangles, cbp.Noun()); err != nil {
		return nil, err
	}
	if tessellation.OrientationSign(mesh.n.Vertices, mesh.n.Triangles, pl.Point(0, 0, cbp.Z0)) <= 0 {
		return nil, fmt.Errorf(`%w: this %s's assembled cells do not enclose a positive volume`, ErrUnsupported, cbp.Noun())
	}

	faceBounds, err := tessellation.ComposeCapBlendBounds(mesh.n.Source, mesh.n.Triangles, faceExtra, store, cbp.Noun())
	if err != nil {
		return nil, err
	}
	for f, bound := range faceBounds {
		topobuild.SetFaceBound(&mesh.n, f, bound)
	}
	mesh.n.AreaSlack = proofbound.AbsSumUpper(mesh.n.AreaSlack,
		tessellation.StoreAreaAllow(mesh.n.Vertices, mesh.n.Triangles, store))
	if cbp.LoftSource != nil {
		for face, bound := range mesh.n.FaceBound {
			topobuild.SetFaceBound(&mesh.n, face, proofbound.AbsSumUpper(bound, cbp.LoftSource.Proof.FacetDeparture))
		}
		mesh.n.AreaSlack = proofbound.AbsSumUpper(mesh.n.AreaSlack, cbp.LoftSource.Proof.AreaSlack)
	}
	if proofbound.IsNonFinite(mesh.n.AreaSlack) {
		return nil, fmt.Errorf(`%w: this %s mesh states no finite area slack`, ErrUnsupported, cbp.Noun())
	}
	if !proveVolume {
		// The level withholds every volume proof (tessellateContext's
		// withholdProofs), so none is built.
		return mesh, nil
	}
	if cbp.LoftSource != nil {
		if err := proveLoftCapBandVolume(ctx, mesh, b, cbp, lms, motion); err != nil {
			return nil, err
		}
		return mesh, nil
	}
	refusal, err := capblend.OccupiedVolumeAdmission(budget, cbp)
	if err != nil {
		return nil, err
	}
	if refusal != nil {
		// docs/tessellation-reach-design.md §7: this band's cells are not proven
		// to reproduce the exact offset family slice by slice, so the mesh serves
		// export alone and requireVolumeProvingPayload/meshbool.OperandSymDiff refuse it
		// with the same reason.
		mesh.n.SymDiffOK = false
		return mesh, nil //nolint:nilerr // refusal is the admission result; err was checked above
	}
	// Occupied volume (docs/tessellation-reach-design.md §7): the ideal
	// polyhedron B1 differs from the body by its chord polygons' circular
	// segments, slice by slice, and the held mesh differs from B1 by its
	// vertices' motion alone, since the two share one triangle index set.
	motionMax, err := tessellation.StoreMax(motion)
	if err != nil {
		return nil, err
	}
	terms := []float64{
		capblend.ChordVolume(cbp, lms),
		proofbound.SweptVolumeAllow(motionMax, proofbound.PerturbedAreaUpper(mesh.n.Vertices, mesh.n.Triangles, motionMax)),
	}
	if err := publishSymDiff(mesh, terms); err != nil {
		return nil, err
	}
	return mesh, nil
}

// capBlendVertices writes every loop's mesh vertices. Every sample of a loop
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
func capBlendVertices(budget *proofbound.WorkBudget, cbp capBlendPayload, pl prismPayload, lms []capblend.LoopMesh, mesh *Mesh, proveVolume bool) ([]float64, []float64, error) {
	var store, motion []float64
	addVertex := func(p planePoint, z, plane float64) (int, float64) {
		held := pl.Point(p.U, p.V, z)
		mesh.n.Vertices = append(mesh.n.Vertices, held)
		round := pl.PointRound(p.U, p.V, z, held)
		store = append(store, proofbound.AbsSumUpper(plane, round))
		return len(mesh.n.Vertices) - 1, round
	}
	startLevel := cbp.CapBandLevel(cbp.Z0, 1)
	endLevel := cbp.CapBandLevel(cbp.Z1, -1)
	for li := range lms {
		lm := &lms[li]
		var sideErr error
		if cbp.Draft {
			lm.SideLo, lm.SideHi, sideErr = capblend.DraftNearRing(
				budget, lm.SidePts, lm.SideBound, lm.ZLo, addVertex, &motion, proveVolume)
		} else {
			lm.SideLo, lm.SideHi, sideErr = capblend.SideRing(
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

// emitCapBand maps built patch roles to the numeric band emitter. roleOf, when
// non-nil, names patch p's role in place of chamferCap(cap,loop,p): a route L
// band's patches carry chamferLoop(f,l,p) (modify-general BG2).
func emitCapBand(budget *proofbound.WorkBudget, m *Mesh, cbp capBlendPayload, lm *capblend.LoopMesh, start bool, faceOfRole func(string) (*Face, error), geomOfRole map[string]capPatchGeom, bump func(*Face, float64), roleOf func(int) string) error {
	matSign := 1.0
	capZ := cbp.Z0
	sideV, capV := lm.SideLo, lm.CapLoV
	if !start {
		matSign, capZ = -1, cbp.Z1
		sideV, capV = lm.SideHi, lm.CapHiV
	}
	setback := cbp.SetbackAt(matSign)
	sideZ := capZ + matSign*setback.Ds
	delta, ok := cbp.BandDelta[capBandKey{Loop: lm.LoopIndex, Start: start}]
	if !ok {
		return fmt.Errorf(`%w: the payload states no contour displacement for the chamfer band on loop %d`, ErrDegenerate, lm.LoopIndex)
	}
	levelDelta := proofbound.AbsSumUpper(setback.DsDelta, proofarith.AddRoundError(capZ, matSign*setback.Ds, sideZ))
	axial := cbp.CapBandLevel(capZ, matSign).Bound
	capName := capblend.CapNameOf(matSign)
	patchFace := func(p int) (*Face, tessellation.CapBlendBandPatch, error) {
		role := fmt.Sprintf("chamferCap(%s,%d,%d)", capName, lm.LoopIndex, p)
		if cbp.Draft {
			// A draft band has no apex patch, so patch p is walk p, and its
			// face carries the prism's own wall role (draftPayload.Patches).
			role = prismWallRole(lm.LoopIndex, lm.Walks[p].Segs[0])
		}
		if roleOf != nil {
			role = roleOf(p)
		}
		f, err := faceOfRole(role)
		if err != nil {
			return nil, tessellation.CapBlendBandPatch{}, err
		}
		g, ok := geomOfRole[role]
		if !ok {
			return nil, tessellation.CapBlendBandPatch{}, fmt.Errorf(`%w: the payload states no geometry for patch role %q`, ErrDegenerate, role)
		}
		return f, tessellation.CapBlendBandPatch{
			Circular: g.Circular, SideRadius: g.SideRadius, CapRadius: g.CapRadius,
			Skew: capblend.PatchWindowSkew(g), AreaAllow: capband.DisplacementAreaAllow(g),
		}, nil
	}
	return tessellation.EmitCapBlendBand[*Face](budget, capBlendBandWriter{m: m, bump: bump},
		tessellation.CapBlendBandInput{
			Loop: lm.LoopIndex, Start: start, Walks: lm.Walks, Joins: capblend.SampleJoins(lm.Joins),
			Count: lm.Count, ArcCount: lm.ArcCount,
			SideStart: lm.SideStart, CapWallStart: lm.CapWallStart, CapArcStart: lm.CapArcStart,
			SideV: sideV, CapV: capV,
			SideSag: lm.SideSag, CapSag: lm.CapSag, CapRadius: lm.CapRadius,
			ArcSag: lm.ArcSag, LocusGap: lm.LocusGap,
			D: setback.Dc, Delta: delta, LevelDelta: levelDelta, Axial: axial,
			Amounts: cbp.WalkAmounts(lm.LoopIndex, lm.Walks, setback.Dc),
		}, patchFace)
}

type capBlendBandWriter struct {
	m    *Mesh
	bump func(*Face, float64)
}

func (w capBlendBandWriter) AddTriangle(tri [3]int, face *Face) {
	topobuild.AddTriangle(&w.m.n, tri, faceNode(face))
}
func (w capBlendBandWriter) Vertices() []r3.Vec             { return w.m.n.Vertices }
func (w capBlendBandWriter) Triangles() [][3]int            { return w.m.n.Triangles }
func (w capBlendBandWriter) Bump(face *Face, delta float64) { w.bump(face, delta) }
func (w capBlendBandWriter) AddAreaSlack(patch, facets float64) {
	w.m.n.AreaSlack = proofbound.AbsSumUpper(w.m.n.AreaSlack, patch, facets)
}

// emitCapBlendCap triangulates one cap face over the ring bounding it per loop:
// the offset contour ring where that loop is chamfered on this cap, and the
// original ring where it is not. The rings must clear one another by more than
// their own sagitta tubes, or a cap at this tolerance cannot prove it represents
// the region's topology and refuses rather than return a pinched mesh.
func emitCapBlendCap(ctx context.Context, m *Mesh, cbp capBlendPayload, lms []capblend.LoopMesh, start bool, face *Face, bump func(*Face, float64)) error {
	var pts []planePoint
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
			d, ok := cbp.BandDelta[capBandKey{Loop: lm.LoopIndex, Start: start}]
			if !ok {
				return fmt.Errorf(`%w: the payload states no contour displacement for the chamfer band on loop %d`, ErrDegenerate, lm.LoopIndex)
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
	if err := requireLoopClearance(ctx, pts, loopIdx, sag); err != nil {
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
		m.n.AreaSlack = proofbound.AbsSumUpper(m.n.AreaSlack,
			tessellation.CapBlendRingSegmentArea(lm.Proof(), chamfered, cbp.LoopOffset(lm.LoopIndex)))
	}
	tris, err := triangulation.Triangulate(ctx, point2ToRecordSlice(pts), loopIdx)
	if err != nil {
		return err
	}
	for _, tri := range tris {
		if start {
			topobuild.AddTriangle(&m.n, [3]int{vtx[tri[0]], vtx[tri[2]], vtx[tri[1]]}, faceNode(face))
			continue
		}
		topobuild.AddTriangle(&m.n, [3]int{vtx[tri[0]], vtx[tri[1]], vtx[tri[2]]}, faceNode(face))
	}
	axial := cbp.Z0Delta
	if !start {
		axial = cbp.Z1Delta
	}
	bump(face, proofbound.AbsSumUpper(trim, delta, axial))
	return nil
}
