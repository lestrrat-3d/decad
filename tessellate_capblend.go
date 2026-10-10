package decad

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/capband"
	"github.com/lestrrat-3d/decad/internal/filletband"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/triangulation"

	"github.com/lestrrat-3d/decad/internal/survey2d"

	"github.com/lestrrat-3d/decad/internal/proofbound"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/stationbound"
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
// payload whose every band capBlendOccupiedVolumeAdmission (capblend_admit.go)
// admits — whole turns, line-line miters and exactly G1 joins: the chord
// polygon's circular-segment integral over the trimmed and band ranges
// (capBlendChordVolume) plus proofbound.SweptVolumeAllow over each vertex's displacement
// from the ideal polyhedron B1. Every other band — a circular wall at a genuine
// miter, a reflex corner — leaves the mesh export-only with symDiffOK false,
// and the mesh boolean refuses that operand (boolean.go's
// requireVolumeProvingPayload and operandSymDiff) with the same reason.
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

// capBlendLoopMesh is one recorded loop's complete chording: the walks the
// build resolved, the offset joins the cap contour is trimmed at, the ONE chord
// count per walk every level of that loop reads, and the plane-local samples
// and mesh vertices that count produced.
//
// A loop chamfered on either cap carries a cap contour ring as well as its side
// ring. The contour does not depend on WHICH cap the loop is chamfered on — the
// in-plane offset is the same either way (capblend.go's mixedOffsetProfile) —
// so the samples are resolved once and lifted to whichever cap level(s) the
// selection named.
type capBlendLoopMesh struct {
	li    int
	loop  loopRecord
	walks []survey2d.SideWalk
	// joins is the per-corner offset join capOffsetJoins resolves, nil for an
	// unchamfered loop and for the one cornerless closed circle (whole).
	joins          []cornerJoin
	whole          bool
	onStart, onEnd bool
	chamfered      bool
	zLo, zHi       proofbound.BoundedScalar
	count          []int     // the shared chord count of walk i
	sideSag        []float64 // walk i's own side-directrix sagitta at that count
	capSag         []float64 // walk i's cap-directrix sagitta at that count
	capRadius      []float64
	capTh0, capTh1 []float64
	arcCount       []int // a reflex corner's connector-arc chord count, 0 elsewhere
	arcSag         []float64
	arcTh0, arcTh1 []float64
	locusGap       []float64 // corner i's ruling-versus-locus gap, both caps alike
	sidePts        []Point2
	sideBound      []proofbound.WalkEndBound
	sideStart      []int // walk i's first side sample
	capPts         []Point2
	capBound       []proofbound.WalkEndBound
	capWallStart   []int // walk i's first cap sample
	// capMotion is each cap sample's plane-local displacement from the point
	// docs/tessellation-reach-design.md §7's ideal polyhedron B1 places there,
	// filled by capBlendCapMotion only when the occupied-volume proof is built.
	// It answers a different question from capBound — a station's gap from the
	// held offset circle — and never replaces it.
	capMotion      []float64
	capArcStart    []int // corner i's first connector-arc sample, −1 when not reflex
	sideLo, sideHi []int // mesh vertices of each side sample at zLo and zHi
	capLoV, capHiV []int // mesh vertices of each cap sample at z0 and z1
}

// proof passes the numeric loop record without copying its walk or sample slices.
func (lm *capBlendLoopMesh) proof() tessellation.CapBlendLoopProof {
	return tessellation.CapBlendLoopProof{
		Walks: lm.walks, Count: lm.count,
		SideSag: lm.sideSag, CapSag: lm.capSag,
		CapRadius: lm.capRadius, CapTh0: lm.capTh0, CapTh1: lm.capTh1,
		ArcCount: lm.arcCount, ArcSag: lm.arcSag,
		ArcTh0: lm.arcTh0, ArcTh1: lm.arcTh1,
		ZLo: lm.zLo, ZHi: lm.zHi,
		Chamfered: lm.chamfered, OnStart: lm.onStart, OnEnd: lm.onEnd,
	}
}

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
	for _, p := range cbp.patches {
		geomOfRole[p.role] = p.geom
	}
	capStart, err := faceOfRole(roleCapStart)
	if err != nil {
		return nil, err
	}
	capEnd, err := faceOfRole(roleCapEnd)
	if err != nil {
		return nil, err
	}

	pl := cbp.prismLike(0, 0)
	work := freeform.NewFreeformWork()
	budget := proofbound.NewWorkBudget(ctx)
	loops := cbp.loops()
	lms := make([]capBlendLoopMesh, len(loops))
	for li, loop := range loops {
		lm, err := chordCapBlendLoop(ctx, budget, cbp, li, loop, chord, work)
		if err != nil {
			return nil, err
		}
		lms[li] = lm
	}

	proveVolume := verify >= VerifyAll
	if proveVolume {
		for li := range lms {
			if err := capBlendCapMotion(budget, cbp, &lms[li]); err != nil {
				return nil, err
			}
		}
	}

	var mesh Mesh
	store, motion, err := capBlendVertices(budget, cbp, pl, lms, &mesh, proveVolume)
	if err != nil {
		return nil, err
	}
	if _, err := tessellation.StoreMax(store); err != nil {
		return nil, err
	}

	// faceExtra is every displacement a face carries BESIDE its own vertices'
	// store term, which composeCapBlendBounds adds per face at the end.
	faceExtra := map[*Face]float64{}
	bump := func(f *Face, v float64) {
		if v > faceExtra[f] {
			faceExtra[f] = v
		}
	}

	// Trimmed side walls: the prism's own cells between the loop's two rings.
	// A draft view has none: its band runs the whole sweep, and its one side
	// ring is the near cap's rim (draftNearRing).
	if !cbp.draft {
		for li := range lms {
			lm := &lms[li]
			axial := math.Max(lm.zLo.Bound, lm.zHi.Bound)
			height := math.Abs(lm.zHi.Value - lm.zLo.Value)
			n := len(lm.walks)
			for i, w := range lm.walks {
				if err := budget.Step(); err != nil {
					return nil, err
				}
				face, err := faceOfRole(fmt.Sprintf("side(%d,%d)", lm.li, w.Segs[0]))
				if err != nil {
					return nil, err
				}
				bump(face, proofbound.AbsSumUpper(lm.sideSag[i], axial))
				mesh.areaSlack = proofbound.AbsSumUpper(mesh.areaSlack, tessellation.WalkWallSlack(w.SegmentWalk, lm.count[i], height))
				for k := range lm.count[i] {
					g0 := lm.sideStart[i] + k
					g1 := lm.sideStart[i] + k + 1
					if k == lm.count[i]-1 {
						g1 = lm.sideStart[(i+1)%n]
					}
					mesh.addTriangle([3]int{lm.sideLo[g0], lm.sideLo[g1], lm.sideHi[g1]}, face)
					mesh.addTriangle([3]int{lm.sideLo[g0], lm.sideHi[g1], lm.sideHi[g0]}, face)
				}
			}
		}
	}

	// Chamfer bands: the patch cells between the side ring at the band's own
	// side level and the cap contour ring at its cap level.
	for li := range lms {
		lm := &lms[li]
		if lm.onStart {
			if err := emitCapBand(budget, &mesh, cbp, lm, true, faceOfRole, geomOfRole, bump, nil); err != nil {
				return nil, err
			}
		}
		if lm.onEnd {
			if err := emitCapBand(budget, &mesh, cbp, lm, false, faceOfRole, geomOfRole, bump, nil); err != nil {
				return nil, err
			}
		}
	}

	// Cap faces. Each is bounded, per loop, by that loop's contour ring where it
	// is chamfered on this cap and by its original ring where it is not — the
	// mixed profile evalCapBlendContext builds the same two faces from.
	if err := emitCapBlendCap(ctx, &mesh, cbp, lms, true, capStart, bump); err != nil {
		return nil, err
	}
	if err := emitCapBlendCap(ctx, &mesh, cbp, lms, false, capEnd, bump); err != nil {
		return nil, err
	}

	// A reflected placement flips handedness, turning every counter-clockwise
	// winding clockwise; reversing them restores outward orientation.
	if pl.reflected() {
		for i := range mesh.triangles {
			mesh.triangles[i][1], mesh.triangles[i][2] = mesh.triangles[i][2], mesh.triangles[i][1]
		}
	}

	if err := tessellation.RequireClosedMesh(mesh.triangles); err != nil {
		return nil, fmt.Errorf(`%w: this %s's cells do not close into a watertight boundary`, ErrUnsupported, cbp.noun())
	}
	if err := tessellation.RequireVertexLinks(ctx, len(mesh.vertices), mesh.triangles); err != nil {
		return nil, err
	}
	if err := requireCapBlendFacetAreas(&mesh, cbp.noun()); err != nil {
		return nil, err
	}
	if tessellation.OrientationSign(mesh.vertices, mesh.triangles, pl.point(0, 0, cbp.z0)) <= 0 {
		return nil, fmt.Errorf(`%w: this %s's assembled cells do not enclose a positive volume`, ErrUnsupported, cbp.noun())
	}

	if err := composeCapBlendBounds(&mesh, faceExtra, store, cbp.noun()); err != nil {
		return nil, err
	}
	mesh.areaSlack = proofbound.AbsSumUpper(mesh.areaSlack,
		tessellation.StoreAreaAllow(mesh.vertices, mesh.triangles, store))
	if cbp.loftSource != nil {
		for face, bound := range mesh.faceBound {
			mesh.setFaceBound(face, proofbound.AbsSumUpper(bound, cbp.loftSource.proof.FacetDeparture))
		}
		mesh.areaSlack = proofbound.AbsSumUpper(mesh.areaSlack, cbp.loftSource.proof.AreaSlack)
	}
	if proofbound.IsNonFinite(mesh.areaSlack) {
		return nil, fmt.Errorf(`%w: this %s mesh states no finite area slack`, ErrUnsupported, cbp.noun())
	}
	if !proveVolume {
		// The level withholds every volume proof (tessellateContext's
		// withholdProofs), so none is built.
		return &mesh, nil
	}
	if cbp.loftSource != nil {
		if err := proveLoftCapBandVolume(ctx, &mesh, b, cbp, lms, motion); err != nil {
			return nil, err
		}
		return &mesh, nil
	}
	refusal, err := capBlendOccupiedVolumeAdmission(budget, cbp)
	if err != nil {
		return nil, err
	}
	if refusal != nil {
		// docs/tessellation-reach-design.md §7: this band's cells are not proven
		// to reproduce the exact offset family slice by slice, so the mesh serves
		// export alone and requireVolumeProvingPayload/operandSymDiff refuse it
		// with the same reason.
		mesh.symDiffOK = false
		return &mesh, nil //nolint:nilerr // refusal is the admission result; err was checked above
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
		capBlendChordVolume(cbp, lms),
		proofbound.SweptVolumeAllow(motionMax, proofbound.PerturbedAreaUpper(mesh.vertices, mesh.triangles, motionMax)),
	}
	if err := publishSymDiff(&mesh, terms); err != nil {
		return nil, err
	}
	return &mesh, nil
}

// capBlendVertices writes every loop's mesh vertices. Every sample of a loop
// owns one vertex per level the loop reaches: the side ring at zLo and zHi, and
// the cap contour ring at whichever cap(s) the selection named. store is
// docs/tessellation-reach-design.md §3's deltaStore, one entry per vertex.
//
// motion, built only when proveVolume, is §7's per-vertex displacement from
// the ideal polyhedron B1, a parallel array: the vertex's plane-local
// displacement (the side station's own bound, or the cap station's capMotion),
// the rounding exactPrismPointRound measures, and the proven bound on the level
// it stands at, summed outward. The rounding is measured once per vertex and
// read by both arrays.
func capBlendVertices(budget *proofbound.WorkBudget, cbp capBlendPayload, pl prismPayload, lms []capBlendLoopMesh, mesh *Mesh, proveVolume bool) ([]float64, []float64, error) {
	var store, motion []float64
	addVertex := func(p Point2, z, plane float64) (int, float64) {
		held := pl.point(p.U, p.V, z)
		mesh.vertices = append(mesh.vertices, held)
		round := exactPrismPointRound(pl, p.U, p.V, z, held)
		store = append(store, proofbound.AbsSumUpper(plane, round))
		return len(mesh.vertices) - 1, round
	}
	startLevel := cbp.capBandLevel(cbp.z0, 1)
	endLevel := cbp.capBandLevel(cbp.z1, -1)
	for li := range lms {
		lm := &lms[li]
		sideRing := capBlendSideRing
		if cbp.draft {
			sideRing = draftNearRing
		}
		if err := sideRing(budget, lm, addVertex, &motion, proveVolume); err != nil {
			return nil, nil, err
		}
		if len(lm.capPts) == 0 {
			continue
		}
		lm.capLoV = make([]int, len(lm.capPts))
		lm.capHiV = make([]int, len(lm.capPts))
		for j, p := range lm.capPts {
			if err := budget.Step(); err != nil {
				return nil, nil, err
			}
			plane := proofbound.WalkEndBoundAllow(lm.capBound[j])
			var round float64
			if lm.onStart {
				lm.capLoV[j], round = addVertex(p, cbp.z0, plane)
				if proveVolume {
					motion = append(motion, proofbound.AbsSumUpper(lm.capMotion[j], round, startLevel.Bound))
				}
			}
			if lm.onEnd {
				lm.capHiV[j], round = addVertex(p, cbp.z1, plane)
				if proveVolume {
					motion = append(motion, proofbound.AbsSumUpper(lm.capMotion[j], round, endLevel.Bound))
				}
			}
		}
	}
	return store, motion, nil
}

// capBlendVertexWriter places one plane-local point at level z beside its
// plane-local bound and returns the new vertex with the lift's rounding
// (capBlendVertices' addVertex).
type capBlendVertexWriter func(p Point2, z, plane float64) (int, float64)

// capBlendSideRing writes one loop's side ring twice, at zLo and at zHi, the
// two rims of its trimmed side wall, and appends each vertex's motion when the
// volume proof is built.
func capBlendSideRing(budget *proofbound.WorkBudget, lm *capBlendLoopMesh, addVertex capBlendVertexWriter, motion *[]float64, proveVolume bool) error {
	lm.sideLo = make([]int, len(lm.sidePts))
	lm.sideHi = make([]int, len(lm.sidePts))
	for j, p := range lm.sidePts {
		if err := budget.Step(); err != nil {
			return err
		}
		plane := proofbound.WalkEndBoundAllow(lm.sideBound[j])
		var round float64
		lm.sideLo[j], round = addVertex(p, lm.zLo.Value, plane)
		if proveVolume {
			*motion = append(*motion, proofbound.AbsSumUpper(plane, round, lm.zLo.Bound))
		}
		lm.sideHi[j], round = addVertex(p, lm.zHi.Value, plane)
		if proveVolume {
			*motion = append(*motion, proofbound.AbsSumUpper(plane, round, lm.zHi.Bound))
		}
	}
	return nil
}

// capBlendCapMotion reads the loop's numeric record through internal/tessellation.
func capBlendCapMotion(budget *proofbound.WorkBudget, cbp capBlendPayload, lm *capBlendLoopMesh) error {
	in := tessellation.CapBlendMotionInput{
		CapBlendLoopProof: lm.proof(),
		Loop:              lm.li, Segments: lm.loop.Segments,
		CapPts: point2ToRecordSlice(lm.capPts), CapWallStart: lm.capWallStart,
		Whole: lm.whole, D: cbp.loopOffset(lm.li), DDelta: cbp.loopSetback(lm.li).dcDelta,
		Amounts: cbp.walkAmounts(lm.li, lm.walks, cbp.loopOffset(lm.li)),
	}
	in.BandDelta[0], in.HasBandDelta[0] = cbp.bandDelta[capBandKey{loop: lm.li, start: true}]
	in.BandDelta[1], in.HasBandDelta[1] = cbp.bandDelta[capBandKey{loop: lm.li, start: false}]
	motion, err := tessellation.CapBlendCapMotion(budget, in, stationbound.CapOffsetStationBound)
	if err != nil {
		return err
	}
	lm.capMotion = motion
	return nil
}

// capBlendChordVolume adapts the resolved loops to the slice-wise proof.
func capBlendChordVolume(cbp capBlendPayload, lms []capBlendLoopMesh) float64 {
	loops := make([]tessellation.CapBlendLoopProof, len(lms))
	for i := range lms {
		loops[i] = lms[i].proof()
	}
	height := [2]float64{
		proofbound.AbsSumUpper(cbp.start.ds, cbp.start.dsDelta),
		proofbound.AbsSumUpper(cbp.end.ds, cbp.end.dsDelta),
	}
	if cbp.draft {
		h := draftBandHeightUpper(cbp)
		height = [2]float64{h, h}
	}
	return tessellation.CapBlendChordVolume(height, loops)
}

// chordCapBlendLoop resolves ONE loop's walks the way buildCapBand does and
// chords each of them ONCE, at the single count every level of that loop then
// reads (docs/tessellation-reach-design.md §7).
//
// For a circular wall of a chamfered loop the count is the larger of what the
// wall's own arc needs and what its cap-level offset arc needs, so the side
// ring, the band patch and the cap contour all carry the same number of
// stations and the band's strips meet along shared vertices rather than along
// two independently sampled polylines. A straight wall needs one sample; a
// reflex corner's connector arc is a directrix of its own and chords with its
// own count.
func chordCapBlendLoop(ctx context.Context, budget *proofbound.WorkBudget, cbp capBlendPayload, li int, loop loopRecord, chord float64, work *freeform.FreeformWork) (capBlendLoopMesh, error) {
	if err := ctx.Err(); err != nil {
		return capBlendLoopMesh{}, err
	}
	cl, err := oneLoopCornerLoop(budget, loop, work)
	if err != nil {
		return capBlendLoopMesh{}, err
	}
	walks := cl.walks
	n := len(walks)
	lm := capBlendLoopMesh{
		li:      li,
		loop:    loop,
		walks:   walks,
		whole:   n == 1 && walks[0].Closed,
		onStart: cbp.startLoops[li],
		onEnd:   cbp.endLoops[li],
	}
	lm.chamfered = lm.onStart || lm.onEnd

	lm.zLo = proofbound.MeasuredScalar(cbp.z0, cbp.z0Delta)
	lm.zHi = proofbound.MeasuredScalar(cbp.z1, cbp.z1Delta)
	if lm.onStart {
		lm.zLo = proofbound.BoundedAdd(lm.zLo, proofbound.MeasuredScalar(cbp.start.ds, cbp.start.dsDelta))
	}
	if lm.onEnd {
		lm.zHi = proofbound.BoundedSub(lm.zHi, proofbound.MeasuredScalar(cbp.end.ds, cbp.end.dsDelta))
	}
	if cbp.draft {
		// A draft band's side level is the near cap itself, and no trimmed
		// wall separates two side rings.
		lm.zLo = draftNearLevel(cbp)
		lm.zHi = lm.zLo
	}

	if lm.chamfered && !lm.whole {
		lm.joins, err = cbp.offsetJoins(budget, li, cl, cbp.loopOffset(li))
		if err != nil {
			return capBlendLoopMesh{}, err
		}
	}

	chords, err := tessellation.ChordCapBlendLoop(tessellation.CapBlendChordInput{
		Walks: walks, Segments: loop.Segments, Joins: capBlendSampleJoins(lm.joins),
		Whole: lm.whole, Chamfered: lm.chamfered, D: cbp.loopOffset(li), Chord: chord,
		Setback: capBlendProofSetback(cbp.loopSetback(li)), FootDelta: cbp.loopBandDelta(li),
	}, budget, func(w survey2d.SideWalk, d float64) (float64, error) {
		if cbp.fillet && filletband.SphereWalk(w, d) {
			return 0, nil
		}
		// A subset draft's kept wall keeps its own radius (walkAmounts).
		if cbp.draft {
			return capband.WallRadius(w, walkAmounts(cbp.draftKept, li, []survey2d.SideWalk{w}, d)[0], shellTol)
		}
		return capband.BandRadius(w, d, shellTol)
	}, capband.WallSweep)
	if err != nil {
		return capBlendLoopMesh{}, err
	}
	lm.count, lm.sideSag, lm.capSag = chords.Count, chords.SideSag, chords.CapSag
	lm.capRadius, lm.capTh0, lm.capTh1 = chords.CapRadius, chords.CapTh0, chords.CapTh1
	lm.arcCount, lm.arcSag = chords.ArcCount, chords.ArcSag
	lm.arcTh0, lm.arcTh1, lm.locusGap = chords.ArcTh0, chords.ArcTh1, chords.LocusGap

	if err := emitCapBlendSamples(budget, cbp, &lm); err != nil {
		return capBlendLoopMesh{}, err
	}
	return lm, nil
}

// emitCapBlendSamples adapts the resolved offset joins to the shared sampler.
func emitCapBlendSamples(budget *proofbound.WorkBudget, cbp capBlendPayload, lm *capBlendLoopMesh) error {
	samples, err := tessellation.SampleCapBlend(tessellation.CapBlendSampleInput{
		Walks: lm.walks, Segments: lm.loop.Segments, Joins: capBlendSampleJoins(lm.joins),
		Count: lm.count, CapRadius: lm.capRadius, CapTh0: lm.capTh0, CapTh1: lm.capTh1,
		ArcCount: lm.arcCount, ArcTh0: lm.arcTh0, ArcTh1: lm.arcTh1,
		Whole: lm.whole, Chamfered: lm.chamfered, D: cbp.loopOffset(lm.li),
	}, budget, stationbound.ChordStationBound)
	if err != nil {
		return err
	}
	lm.sidePts, lm.sideBound, lm.sideStart = point2FromRecordSlice(samples.SidePts), samples.SideBound, samples.SideStart
	lm.capPts, lm.capBound = point2FromRecordSlice(samples.CapPts), samples.CapBound
	lm.capWallStart, lm.capArcStart = samples.CapWallStart, samples.CapArcStart
	return nil
}

func capBlendSampleJoins(joins []cornerJoin) []tessellation.CapBlendJoin {
	if joins == nil {
		return nil
	}
	out := make([]tessellation.CapBlendJoin, len(joins))
	for i, j := range joins {
		out[i] = tessellation.CapBlendJoin{
			Arc: j.Arc, G1: j.G1, VU: j.VertU, VV: j.VertV, M: j.M, PA: j.PA, PB: j.PB,
		}
	}
	return out
}

func capBlendProofSetback(s capSetback) tessellation.CapBlendSetback {
	return tessellation.CapBlendSetback{
		AxialUpper: s.axialUpper(), Dc: s.dc, DcDelta: s.dcDelta,
		Ds: s.ds, DsDelta: s.dsDelta,
	}
}

// emitCapBand maps built patch roles to the numeric band emitter. roleOf, when
// non-nil, names patch p's role in place of chamferCap(cap,loop,p): a route L
// band's patches carry chamferLoop(f,l,p) (modify-general BG2).
func emitCapBand(budget *proofbound.WorkBudget, m *Mesh, cbp capBlendPayload, lm *capBlendLoopMesh, start bool, faceOfRole func(string) (*Face, error), geomOfRole map[string]capPatchGeom, bump func(*Face, float64), roleOf func(int) string) error {
	matSign := 1.0
	capZ := cbp.z0
	sideV, capV := lm.sideLo, lm.capLoV
	if !start {
		matSign, capZ = -1, cbp.z1
		sideV, capV = lm.sideHi, lm.capHiV
	}
	setback := cbp.setbackAt(matSign)
	sideZ := capZ + matSign*setback.ds
	delta, ok := cbp.bandDelta[capBandKey{loop: lm.li, start: start}]
	if !ok {
		return fmt.Errorf(`%w: the payload states no contour displacement for the chamfer band on loop %d`, ErrDegenerate, lm.li)
	}
	levelDelta := proofbound.AbsSumUpper(setback.dsDelta, proofarith.AddRoundError(capZ, matSign*setback.ds, sideZ))
	axial := cbp.capBandLevel(capZ, matSign).Bound
	capName := capNameOf(matSign)
	patchFace := func(p int) (*Face, tessellation.CapBlendBandPatch, error) {
		role := fmt.Sprintf("chamferCap(%s,%d,%d)", capName, lm.li, p)
		if cbp.draft {
			// A draft band has no apex patch, so patch p is walk p, and its
			// face carries the prism's own wall role (draftPayload.patches).
			role = prismWallRole(lm.li, lm.walks[p].Segs[0])
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
			Skew: capPatchWindowSkew(g), AreaAllow: capband.DisplacementAreaAllow(g),
		}, nil
	}
	return tessellation.EmitCapBlendBand[*Face](budget, capBlendBandWriter{m: m, bump: bump},
		tessellation.CapBlendBandInput{
			Loop: lm.li, Start: start, Walks: lm.walks, Joins: capBlendSampleJoins(lm.joins),
			Count: lm.count, ArcCount: lm.arcCount,
			SideStart: lm.sideStart, CapWallStart: lm.capWallStart, CapArcStart: lm.capArcStart,
			SideV: sideV, CapV: capV,
			SideSag: lm.sideSag, CapSag: lm.capSag, CapRadius: lm.capRadius,
			ArcSag: lm.arcSag, LocusGap: lm.locusGap,
			D: setback.dc, Delta: delta, LevelDelta: levelDelta, Axial: axial,
			Amounts: cbp.walkAmounts(lm.li, lm.walks, setback.dc),
		}, patchFace)
}

type capBlendBandWriter struct {
	m    *Mesh
	bump func(*Face, float64)
}

func (w capBlendBandWriter) AddTriangle(tri [3]int, face *Face) { w.m.addTriangle(tri, face) }
func (w capBlendBandWriter) Vertices() []r3.Vec                 { return w.m.vertices }
func (w capBlendBandWriter) Triangles() [][3]int                { return w.m.triangles }
func (w capBlendBandWriter) Bump(face *Face, delta float64)     { w.bump(face, delta) }
func (w capBlendBandWriter) AddAreaSlack(patch, facets float64) {
	w.m.areaSlack = proofbound.AbsSumUpper(w.m.areaSlack, patch, facets)
}

// emitCapBlendCap triangulates one cap face over the ring bounding it per loop:
// the offset contour ring where that loop is chamfered on this cap, and the
// original ring where it is not. The rings must clear one another by more than
// their own sagitta tubes, or a cap at this tolerance cannot prove it represents
// the region's topology and refuses rather than return a pinched mesh.
func emitCapBlendCap(ctx context.Context, m *Mesh, cbp capBlendPayload, lms []capBlendLoopMesh, start bool, face *Face, bump func(*Face, float64)) error {
	var pts []Point2
	var vtx []int
	var loopIdx [][]int
	var sag []float64
	trim, delta := 0.0, 0.0
	for li := range lms {
		lm := &lms[li]
		chamfered := lm.onStart
		if !start {
			chamfered = lm.onEnd
		}
		ringPts, ringV := lm.sidePts, lm.sideLo
		ringSag := capBlendRingSagitta(lm, false)
		if !start {
			ringV = lm.sideHi
		}
		if chamfered {
			ringPts, ringV = lm.capPts, lm.capLoV
			if !start {
				ringV = lm.capHiV
			}
			ringSag = capBlendRingSagitta(lm, true)
			d, ok := cbp.bandDelta[capBandKey{loop: lm.li, start: start}]
			if !ok {
				return fmt.Errorf(`%w: the payload states no contour displacement for the chamfer band on loop %d`, ErrDegenerate, lm.li)
			}
			delta = math.Max(delta, d)
			if cbp.fillet {
				ringPts, ringV = compactCapBlendRing(ringPts, ringV)
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
	// walkAreaSlack makes per cap for a prism.
	for li := range lms {
		lm := &lms[li]
		chamfered := lm.onStart
		if !start {
			chamfered = lm.onEnd
		}
		m.areaSlack = proofbound.AbsSumUpper(m.areaSlack, capBlendRingSegmentArea(lm, chamfered, cbp.loopOffset(lm.li)))
	}
	tris, err := triangulation.Triangulate(ctx, point2ToRecordSlice(pts), loopIdx)
	if err != nil {
		return err
	}
	for _, tri := range tris {
		if start {
			m.addTriangle([3]int{vtx[tri[0]], vtx[tri[2]], vtx[tri[1]]}, face)
			continue
		}
		m.addTriangle([3]int{vtx[tri[0]], vtx[tri[1]], vtx[tri[2]]}, face)
	}
	axial := cbp.z0Delta
	if !start {
		axial = cbp.z1Delta
	}
	bump(face, proofbound.AbsSumUpper(trim, delta, axial))
	return nil
}

// compactCapBlendRing omits zero-length contour edges where a fillet's
// inward circular offset has collapsed to its centre. The band's samples
// retain their full count so its strips still match the side wall.
func compactCapBlendRing(pts []Point2, vertices []int) ([]Point2, []int) {
	compactPts := make([]Point2, 0, len(pts))
	compactVertices := make([]int, 0, len(vertices))
	for i, p := range pts {
		if len(compactPts) != 0 && compactPts[len(compactPts)-1] == p {
			continue
		}
		compactPts = append(compactPts, p)
		compactVertices = append(compactVertices, vertices[i])
	}
	if len(compactPts) > 1 && compactPts[0] == compactPts[len(compactPts)-1] {
		compactPts = compactPts[:len(compactPts)-1]
		compactVertices = compactVertices[:len(compactVertices)-1]
	}
	return compactPts, compactVertices
}

// capBlendRingSagitta reads the side or cap ring's numeric proof.
func capBlendRingSagitta(lm *capBlendLoopMesh, contour bool) float64 {
	return tessellation.CapBlendRingSagitta(lm.proof(), contour)
}

// capBlendRingSegmentArea reads the matching circular-segment area proof.
func capBlendRingSegmentArea(lm *capBlendLoopMesh, contour bool, d float64) float64 {
	return tessellation.CapBlendRingSegmentArea(lm.proof(), contour, d)
}

// composeCapBlendBounds publishes docs/tessellation-design.md §2's sourceBound
// for every face the mesh names: the face's own accumulated trim, band and axial
// terms plus the largest store displacement its own vertices carry. Every source
// face is present by construction — the walk is over mesh.source itself — and a
// face whose composed displacement is not finite refuses rather than publishing
// an infinite bound.
func composeCapBlendBounds(m *Mesh, extra map[*Face]float64, store []float64, noun string) error {
	faceStore := map[*Face]float64{}
	for i, f := range m.source {
		for _, v := range m.triangles[i] {
			faceStore[f] = math.Max(faceStore[f], store[v])
		}
	}
	for f, s := range faceStore {
		bound := proofbound.UpRound(extra[f] + s)
		if proofbound.IsNonFinite(bound) {
			return fmt.Errorf(`%w: a %s face's composed displacement is not finite, so this mesh can state no bound for it`, ErrUnsupported, noun)
		}
		m.setFaceBound(f, bound)
	}
	return nil
}

// requireCapBlendFacetAreas refuses a mesh carrying a zero-area triangle
// (docs/tessellation-design.md §12). The test is the exact rational squared
// cross product, so a sliver is judged by what its own coordinates say rather
// than by a tolerance.
func requireCapBlendFacetAreas(m *Mesh, noun string) error {
	for i, tri := range m.triangles {
		a, b, c := m.vertices[tri[0]], m.vertices[tri[1]], m.vertices[tri[2]]
		if tessellation.CapBlendTwiceAreaSq(a, b, c).Sign() <= 0 {
			return fmt.Errorf(`%w: facet %d of this %s mesh has zero area`, ErrUnsupported, i, noun)
		}
	}
	return nil
}
