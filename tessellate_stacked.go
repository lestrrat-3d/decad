package decad

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/units"
)

type stackedRing struct {
	samples        []Point2
	bottom, top    []int
	faces          []*Face
	sag            float64
	segmentArea    float64
	walks          int
	perimeterUpper float64
}

func tessellateStacked(ctx context.Context, b *Body, sp stackedPrismPayload, chord float64, verify Verification) (*Mesh, error) {
	if err := falsifyStackedPayload(ctx, sp); err != nil {
		return nil, err
	}
	columns, bySlab, err := stackedColumns(sp)
	if err != nil {
		return nil, err
	}
	budget := chord
	if sp.sectionDelta > 0 {
		budget = downRound(downRound(chord - sp.sectionDelta))
		if budget <= 0 {
			return nil, fmt.Errorf(`%w: requested tolerance %s leaves no chord budget above the body's own section displacement %s`,
				ErrUnsupported, units.Millimeters(chord), units.Millimeters(sp.sectionDelta))
		}
	}
	byRole := map[string]*Face{}
	for _, face := range b.Faces() {
		for _, origin := range face.Origins() {
			byRole[origin.Role] = face
		}
	}
	faceOfRole := func(role string) (*Face, error) {
		face := byRole[role]
		if face == nil {
			return nil, fmt.Errorf(`%w: a stacked prism has no face for role %q`, ErrDegenerate, role)
		}
		return face, nil
	}
	var mesh Mesh
	base := sp.outerPrism()
	work := newFreeformWork()
	var vertexStore []float64
	faceTrim := map[*Face]float64{}
	faceAxial := map[*Face]float64{}
	addVertex := func(p Point2, z float64, source walkEndBound) int {
		v := base.point(p.U, p.V, z)
		mesh.vertices = append(mesh.vertices, v)
		vertexStore = append(vertexStore, absSumUpper(walkEndBoundAllow(source),
			exactPrismPointRound(base, p.U, p.V, z, v)))
		return len(mesh.vertices) - 1
	}
	rings := make([]stackedRing, len(columns))
	for ci, col := range columns {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		first, last := sp.slabs[col.start], sp.slabs[col.end]
		cl, err := chordLoop(ctx, col.loop, budget, last.z1-first.z0, work, nil, col.loopIndex,
			func(w sideWalk) (*Face, error) {
				return faceOfRole(fmt.Sprintf("slab(%d).region(0).side(%d,%d)",
					col.start, col.loopIndex, w.segs[0]))
			})
		if err != nil {
			return nil, err
		}
		r := &rings[ci]
		r.samples, r.faces, r.sag = cl.samples, cl.faceOf, cl.maxSag
		r.segmentArea, r.walks, r.perimeterUpper = cl.segmentArea, cl.walks, cl.perimeterUpper
		r.bottom = make([]int, len(cl.samples))
		r.top = make([]int, len(cl.samples))
		mesh.areaSlack = absSumUpper(mesh.areaSlack, cl.wallSlack, cl.capSlack, cl.capSlack)
		for j, p := range cl.samples {
			r.bottom[j] = addVertex(p, first.z0, cl.boundOf[j])
			r.top[j] = addVertex(p, last.z1, cl.boundOf[j])
		}
		for j, face := range cl.faceOf {
			next := (j + 1) % len(cl.samples)
			mesh.addTriangle([3]int{r.bottom[j], r.bottom[next], r.top[next]}, face)
			mesh.addTriangle([3]int{r.bottom[j], r.top[next], r.top[j]}, face)
			faceTrim[face] = math.Max(faceTrim[face], cl.sagOf[j])
			faceAxial[face] = math.Max(first.z0Delta, last.z1Delta)
		}
	}
	storeMax, err := requireDerivableStore(vertexStore)
	if err != nil {
		return nil, err
	}
	for _, slabColumns := range bySlab {
		var points []Point2
		var indices [][]int
		var sags []float64
		for _, ci := range slabColumns {
			r := rings[ci]
			start := len(points)
			points = append(points, r.samples...)
			idx := make([]int, len(r.samples))
			for i := range idx {
				idx[i] = start + i
			}
			indices = append(indices, idx)
			sags = append(sags, r.sag)
		}
		if err := requireLoopClearance(ctx, points, indices, sags); err != nil {
			return nil, err
		}
	}
	// All region loops already have their material winding. An exposed patch
	// uses a hole loop as its outer boundary and reverses its index order.
	emitRegion := func(role string, slabColumns []int, top, reverseFace, reverseLoop bool, axial float64) error {
		face, err := faceOfRole(role)
		if err != nil {
			return err
		}
		var points []Point2
		var vertices []int
		var loops [][]int
		for _, ci := range slabColumns {
			r := rings[ci]
			start := len(points)
			points = append(points, r.samples...)
			if top {
				vertices = append(vertices, r.top...)
			} else {
				vertices = append(vertices, r.bottom...)
			}
			idx := make([]int, len(r.samples))
			for i := range idx {
				idx[i] = start + i
			}
			if reverseLoop {
				for a, z := 0, len(idx)-1; a < z; a, z = a+1, z-1 {
					idx[a], idx[z] = idx[z], idx[a]
				}
			}
			loops = append(loops, idx)
			faceTrim[face] = math.Max(faceTrim[face], r.sag)
		}
		tris, err := triangulate2DContext(ctx, points, loops)
		if err != nil {
			return err
		}
		for _, tri := range tris {
			a, bb, c := vertices[tri[0]], vertices[tri[1]], vertices[tri[2]]
			if reverseFace {
				bb, c = c, bb
			}
			mesh.addTriangle([3]int{a, bb, c}, face)
		}
		faceAxial[face] = axial
		return nil
	}
	first, last := sp.slabs[0], sp.slabs[len(sp.slabs)-1]
	if err := emitRegion(roleCapStart, bySlab[0], false, true, false, first.z0Delta); err != nil {
		return nil, err
	}
	if err := emitRegion(roleCapEnd, bySlab[len(bySlab)-1], true, false, false, last.z1Delta); err != nil {
		return nil, err
	}
	for k := range sp.interfaces {
		lower, upper := sp.slabs[k].regions[0], sp.slabs[k+1].regions[0]
		lowerOnly, upperOnly, err := stackedExclusiveHoles(lower, upper)
		if err != nil {
			return nil, err
		}
		for e, hole := range upperOnly {
			ci, err := stackedHoleColumn(columns, bySlab[k+1][1:], hole, k+1, true)
			if err != nil {
				return nil, err
			}
			if err := emitRegion(fmt.Sprintf("floor(%d,%d)", k, e), []int{ci}, false, false, true,
				sp.slabs[k].z1Delta); err != nil {
				return nil, err
			}
		}
		for e, hole := range lowerOnly {
			ci, err := stackedHoleColumn(columns, bySlab[k][1:], hole, k, false)
			if err != nil {
				return nil, err
			}
			if err := emitRegion(fmt.Sprintf("ceiling(%d,%d)", k, e), []int{ci}, true, true, true,
				sp.slabs[k].z1Delta); err != nil {
				return nil, err
			}
		}
	}
	if base.reflected() {
		for i := range mesh.triangles {
			mesh.triangles[i][1], mesh.triangles[i][2] = mesh.triangles[i][2], mesh.triangles[i][1]
		}
	}
	if err := liftTessellationError(tessellation.RequireClosedMesh(mesh.triangles)); err != nil {
		return nil, err
	}
	if err := composeFaceBounds(&mesh, faceTrim, faceAxial, vertexStore, sp.sectionDelta); err != nil {
		return nil, err
	}
	if verify < VerifyAll {
		return &mesh, nil
	}
	var allWalks int
	var allPerimeter float64
	for ci, col := range columns {
		r := rings[ci]
		allWalks += r.walks
		allPerimeter = absSumUpper(allPerimeter, r.perimeterUpper)
		if sp.sectionDelta > 0 {
			height := sp.slabs[col.end].z1 - sp.slabs[col.start].z0
			mesh.areaSlack = absSumUpper(mesh.areaSlack,
				productUpper(sectionDisplacementLength(sp.sectionDelta, r.walks), height))
		}
	}
	for _, face := range b.Faces() {
		if _, ok := face.surface.(Plane); ok && sp.sectionDelta > 0 {
			mesh.areaSlack = absSumUpper(mesh.areaSlack,
				sectionDisplacementArea(sp.sectionDelta, allWalks, allPerimeter))
		}
	}
	mesh.areaSlack = absSumUpper(mesh.areaSlack, meshStoreAreaAllow(&mesh, vertexStore))
	areaUpper := meshFaceAreaUpper(&mesh, vertexStore)
	terms := make([]float64, 0, len(columns)*len(sp.slabs)+len(faceAxial)+2)
	for k, slab := range sp.slabs {
		var segments, perimeter float64
		walks := 0
		for _, ci := range bySlab[k] {
			r := rings[ci]
			segments = absSumUpper(segments, r.segmentArea)
			perimeter = absSumUpper(perimeter, r.perimeterUpper)
			walks += r.walks
		}
		height := slab.z1 - slab.z0
		terms = append(terms, productUpper(height, segments),
			productUpper(height, sectionDisplacementArea(sp.sectionDelta, walks, perimeter)))
	}
	for face, axial := range faceAxial {
		if _, ok := face.surface.(Plane); ok {
			terms = append(terms, productUpper(axial, areaUpper[face]))
		}
	}
	terms = append(terms, sweptVolumeAllow(storeMax,
		perturbedAreaUpper(mesh.vertices, mesh.triangles, storeMax)))
	if err := publishSymDiff(&mesh, terms); err != nil {
		return nil, err
	}
	return &mesh, nil
}

func stackedHoleColumn(columns []stackedColumn, candidates []int, hole LoopRecord, slab int, starts bool) (int, error) {
	for _, ci := range candidates {
		col := columns[ci]
		equal, err := loopRecordsEqual(nil, col.loop, hole)
		if err != nil {
			return 0, err
		}
		if equal && ((starts && col.start == slab) || (!starts && col.end == slab)) {
			return ci, nil
		}
	}
	return 0, fmt.Errorf(`%w: an exposed patch has no wall column`, ErrDegenerate)
}
