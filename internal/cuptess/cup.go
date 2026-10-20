package cuptess

import (
	"context"
	"fmt"
	"math"
	"slices"

	"github.com/lestrrat-3d/decad/internal/cupwall"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/offset2d"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/stationbound"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/decad/internal/triangulation"
	"github.com/lestrrat-3d/r3"
)

// Tessellate meshes a cup (agent-docs/modify-design.md §9, D4): the outer region O
// and the cavity region C, each with k ≥ 0 holes, sharing one rim per region
// loop at the open end. Every region loop is chorded ONCE — the same chording
// feeds its walls, its floor cap and its half of the rim band — so every ring
// vertex is shared and the mesh is watertight and consistently oriented by
// construction, exactly as the prism's is. A hole of O is a tunnel through the
// whole body; a hole of C is a solid POST rising from the floor. The planar
// faces (the kept cap over O, the pocket floor over C, and one rim band per
// loop) triangulate through the shipped cap triangulator.
// verifyAll decides whether the area-slack and occupied-volume proofs at the end
// of the build are composed at all; every audit and every face bound above
// them runs at each level (agent-docs/tessellation-design.md §1).
func Tessellate(ctx context.Context, body *topo.Body, cp cupwall.View, chord float64, verifyAll bool) (*topo.Mesh, error) {
	byRole := map[string]*topo.Face{}
	for _, f := range topobuild.Faces(body) {
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
	capStart, err := faceOfRole(provenance.RoleCapStart)
	if err != nil {
		return nil, err
	}
	shellCap, err := faceOfRole(cupwall.RoleShellCap)
	if err != nil {
		return nil, err
	}

	// One free-form counter for the whole chorded record — the cup's outer region
	// and its cavity are the two halves of one section (see tessellation.ChordLoop).
	work := freeform.NewFreeformWork()
	oLoops := append([]sectionrecord.LoopRecord{cp.Outer.Outer}, cp.Outer.Holes...)
	cLoops := append([]sectionrecord.LoopRecord{cp.Cavity.Outer}, cp.Cavity.Holes...)
	if len(oLoops) != len(cLoops) {
		return nil, fmt.Errorf(`%w: the cup's outer and cavity regions have different loop counts`, decaderr.ErrDegenerate)
	}

	openIsMax := cp.ZOpen > cp.ZOuter
	oLo, oHi := math.Min(cp.ZOuter, cp.ZOpen), math.Max(cp.ZOuter, cp.ZOpen)
	cLo, cHi := math.Min(cp.ZCav, cp.ZOpen), math.Max(cp.ZCav, cp.ZOpen)

	mesh := new(topo.Mesh)
	base := cp.BasePrism()
	var verts []r3.Vec
	// vertexStore is agent-docs/tessellation-reach-design.md §3's deltaStore, parallel
	// to verts: the sample's plane-local enclosure gap carried through the frame
	// plus the rounding the frame/placement write commits.
	var vertexStore []float64
	add := func(v r3.Vec, store float64) int {
		verts = append(verts, v)
		vertexStore = append(vertexStore, store)
		return len(verts) - 1
	}
	// Each face's own trim and axial displacement, composed into its published
	// sourceBound once the whole cup is built.
	faceTrim := map[*topo.Face]float64{}
	faceAxial := map[*topo.Face]float64{}
	// The summed circular-segment area of each region's own loops, over the
	// region's own sweep height — the cup's occupied-volume analytic term.
	var oSegmentArea, cSegmentArea float64

	// One chorded ring per region loop: the walls hang off it and the caps and
	// rims share it, so every vertex is shared and the mesh closes by
	// construction. A ring holds its samples, the lo/hi mesh vertices of each,
	// its wall faces and the sagitta bound the chording proved.
	type ring struct {
		samples []sectionrecord.Point2
		loV     []int
		hiV     []int
		faces   []*topo.Face
		sag     float64
		walks   int
		perim   float64
	}
	chordRing := func(loop sectionrecord.LoopRecord, h, lo, hi, loDelta, hiDelta float64, role string, area *float64) (ring, error) {
		// A cup chords the DERIVED region loops — an offset cavity, an outer
		// contour — which no payload holds a resolution of, so each segment
		// resolves through walkOf here as it always has.
		cl, err := tessellation.ChordLoop(ctx, loop, chord, h, work, nil, 0, func(w survey2d.SideWalk) (*topo.Face, error) {
			return faceOfRole(fmt.Sprintf(role, w.Segs[0]))
		}, stationbound.ChordStationBound)
		if err != nil {
			return ring{}, err
		}
		samples := cl.Samples
		// A cup always triangulates two planar patches off this ring (its
		// kept cap or pocket floor, plus a rim band), so it keeps both cap
		// halves of the loop's slack, the same total chording's own
		// combined areaSlack used to carry before it split.
		mesh.AreaSlack = proofbound.AbsSumUpper(mesh.AreaSlack, cl.WallSlack, cl.CapSlack, cl.CapSlack)
		*area = proofbound.AbsSumUpper(*area, cl.SegmentArea)
		r := ring{samples: slices.Clone(samples), faces: cl.FaceOf,
			sag: cl.MaxSag, walks: cl.Walks, perim: cl.PerimeterUpper}
		r.loV = make([]int, len(samples))
		r.hiV = make([]int, len(samples))
		// A wall spans both of its region's levels, so it cannot attribute its
		// axial displacement to one of them and takes the larger.
		wallAxial := math.Max(loDelta, hiDelta)
		for i, p := range samples {
			plane := proofbound.WalkEndBoundAllow(cl.BoundOf[i])
			loV := base.Point(p.U, p.V, lo)
			hiV := base.Point(p.U, p.V, hi)
			r.loV[i] = add(loV, proofbound.AbsSumUpper(plane, base.PointRound(p.U, p.V, lo, loV)))
			r.hiV[i] = add(hiV, proofbound.AbsSumUpper(plane, base.PointRound(p.U, p.V, hi, hiV)))
			f := cl.FaceOf[i]
			faceTrim[f] = math.Max(faceTrim[f], cl.SagOf[i])
			faceAxial[f] = wallAxial
		}
		return r, nil
	}
	// Each region's own lo/hi level displacement, in the order chordRing takes
	// them: the open end carries zOpenDelta, each floor its own level's.
	oLoDelta, oHiDelta := cp.ZOuterDelta, cp.ZOpenDelta
	cLoDelta, cHiDelta := cp.ZCavDelta, cp.ZOpenDelta
	if !openIsMax {
		oLoDelta, oHiDelta = cp.ZOpenDelta, cp.ZOuterDelta
		cLoDelta, cHiDelta = cp.ZOpenDelta, cp.ZCavDelta
	}

	// Outer region O: every loop in its natural sense (outer counter-clockwise,
	// holes clockwise), role side(i,j).
	oRings := make([]ring, len(oLoops))
	for i, loop := range oLoops {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		oRings[i], err = chordRing(loop, oHi-oLo, oLo, oHi, oLoDelta, oHiDelta, fmt.Sprintf("side(%d,%%d)", i), &oSegmentArea)
		if err != nil {
			return nil, err
		}
	}

	// Cavity region C: every loop REVERSED (its material lies outside it), so
	// the reversed outer walks clockwise and each reversed hole counter-
	// clockwise (a post), role shellSide(i,j).
	cRings := make([]ring, len(cLoops))
	for i, loop := range cLoops {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		rev, err := offset2d.ReverseLoopRecordContext(ctx, loop)
		if err != nil {
			return nil, err
		}
		cRings[i], err = chordRing(rev, cHi-cLo, cLo, cHi, cLoDelta, cHiDelta, fmt.Sprintf("shellSide(%d,%%d)", i), &cSegmentArea)
		if err != nil {
			return nil, err
		}
	}
	mesh.Vertices = verts
	storeMax, err := tessellation.StoreMax(vertexStore)
	if err != nil {
		return nil, err
	}

	// The cup topology shares every chorded ring across its wall, floor and rim.
	toTopologyRing := func(r ring) tessellation.CupRing[*topo.Face] {
		return tessellation.CupRing[*topo.Face]{Samples: slices.Clone(r.samples),
			LoV: r.loV, HiV: r.hiV, Faces: r.faces, Sag: r.sag}
	}
	oTopology := make([]tessellation.CupRing[*topo.Face], len(oRings))
	cTopology := make([]tessellation.CupRing[*topo.Face], len(cRings))
	for i := range oRings {
		oTopology[i] = toTopologyRing(oRings[i])
	}
	for i := range cRings {
		cTopology[i] = toTopologyRing(cRings[i])
	}
	rimFace := func(i int) (*topo.Face, error) { return faceOfRole(fmt.Sprintf("rim(%d)", i)) }
	assembled, err := tessellation.AssembleCup(ctx, oTopology, cTopology, openIsMax, capStart, shellCap,
		rimFace, tessellation.RequireLoopClearance, triangulation.Triangulate, faceTrim, faceAxial,
		cp.ZOuterDelta, cp.ZCavDelta, cp.ZOpenDelta)
	if err != nil {
		return nil, err
	}
	mesh.Triangles, mesh.Source = assembled.Triangles, assembled.Sources

	// A reflected placement flips handedness, turning every counter-clockwise
	// winding clockwise; reversing the windings restores outward orientation.
	if base.Reflected() {
		for i := range mesh.Triangles {
			mesh.Triangles[i][1], mesh.Triangles[i][2] = mesh.Triangles[i][2], mesh.Triangles[i][1]
		}
	}

	// The cup's planar faces triangulate through the shipped cap triangulator,
	// which resolves every rim band — including the bridge-collinear one an
	// outward cup or a rectangular post produces, where a rounded outer loop's
	// corner tangent points land on the sharp inner loop's edge lines. The
	// walls and floors close by construction; this proves the assembled mesh is
	// watertight and refuses a cracked one rather than return it, a safety net
	// against any residual chording pathology (core §11, never a wrong mesh).
	if err := topobuild.LiftAuditError(tessellation.RequireClosedMesh(mesh.Triangles)); err != nil {
		return nil, err
	}
	// The offset region — the cavity inward, the outer region outward — is
	// recorded within offsetDelta of the region the cup denotes, so every face
	// it bounds carries that displacement beside its own trim, store and level
	// terms: its walls, its own planar cap and every rim. The receiver's own
	// region carries none (agent-docs/tessellation-design.md §6).
	displacedRings, displacedCap, displacedHeight := cRings, shellCap, cHi-cLo
	if cp.Sense == cupwall.Outward {
		displacedRings, displacedCap, displacedHeight = oRings, capStart, oHi-oLo
	}
	var displacedWalks int
	var displacedPerim float64
	for _, r := range displacedRings {
		displacedWalks += r.walks
		displacedPerim = proofbound.AbsSumUpper(displacedPerim, r.perim)
	}
	if cp.OffsetDelta > 0 {
		displacedFaces := map[*topo.Face]struct{}{displacedCap: {}}
		for _, r := range displacedRings {
			for _, f := range r.faces {
				displacedFaces[f] = struct{}{}
			}
		}
		for i := range oLoops {
			rim, err := faceOfRole(fmt.Sprintf("rim(%d)", i))
			if err != nil {
				return nil, err
			}
			displacedFaces[rim] = struct{}{}
		}
		for f := range displacedFaces {
			faceTrim[f] = proofbound.AbsSumUpper(faceTrim[f], cp.OffsetDelta)
		}
	}
	bounds, err := tessellation.FaceBounds(mesh.Triangles, mesh.Source, faceTrim, faceAxial, vertexStore, 0)
	if err != nil {
		return nil, err
	}
	for face, bound := range bounds {
		topobuild.SetFaceBound(mesh, face, bound)
	}
	if !verifyAll {
		// The same reading the prism arm states: the slack accumulated above
		// is unfinished without the per-facet term below, and the
		// occupied-volume proof has not started, so neither is published.
		return mesh, nil
	}
	mesh.AreaSlack = proofbound.AbsSumUpper(mesh.AreaSlack,
		tessellation.StoreAreaAllow(mesh.Vertices, mesh.Triangles, vertexStore))
	// The displaced region's own area moves by its displacement area once in
	// its cap and once in the rims it bounds, and its walls' length by the
	// displacement length over their height — evalPrism's composition for a
	// section displacement, one region at a time.
	displacedArea := proofbound.SectionDisplacementArea(cp.OffsetDelta, displacedWalks, displacedPerim)
	if cp.OffsetDelta > 0 {
		wallMove := proofbound.ProductUpper(proofbound.SectionDisplacementLength(cp.OffsetDelta, displacedWalks), displacedHeight)
		mesh.AreaSlack = proofbound.AbsSumUpper(mesh.AreaSlack, displacedArea, displacedArea, wallMove)
	}

	// Occupied volume (agent-docs/tessellation-reach-design.md §3): each region's own
	// chorded-section deficit over its own sweep height, the displaced region's
	// section displacement over its height, each planar level's displacement
	// over the patch it caps, and the computed coordinates' swept volume.
	areaUpper := tessellation.FaceAreaUpper(mesh.Vertices, mesh.Triangles, mesh.Source, vertexStore)
	rimArea := 0.0
	for i := range oLoops {
		rim, err := faceOfRole(fmt.Sprintf("rim(%d)", i))
		if err != nil {
			return nil, err
		}
		rimArea = proofbound.AbsSumUpper(rimArea, areaUpper[rim])
	}
	terms := []float64{
		proofbound.ProductUpper(oHi-oLo, oSegmentArea),
		proofbound.ProductUpper(cHi-cLo, cSegmentArea),
		proofbound.ProductUpper(displacedHeight, displacedArea),
		proofbound.ProductUpper(cp.ZOuterDelta, areaUpper[capStart]),
		proofbound.ProductUpper(cp.ZCavDelta, areaUpper[shellCap]),
		proofbound.ProductUpper(cp.ZOpenDelta, rimArea),
		proofbound.SweptVolumeAllow(storeMax, proofbound.PerturbedAreaUpper(mesh.Vertices, mesh.Triangles, storeMax)),
	}
	total, err := tessellation.SymDiff(terms)
	if err != nil {
		return nil, err
	}
	mesh.VolSymDiff = total
	mesh.SymDiffOK = true
	return mesh, nil
}
