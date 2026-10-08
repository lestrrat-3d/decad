package decad

import (
	"context"
	"fmt"
	"math"
	"strings"

	"github.com/lestrrat-3d/decad/internal/freeform"

	"github.com/lestrrat-3d/decad/internal/survey2d"

	"github.com/lestrrat-3d/decad/internal/proofbound"

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
		budget = freeform.DownRound(freeform.DownRound(chord - sp.sectionDelta))
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
	work := freeform.NewFreeformWork()
	var vertexStore []float64
	faceTrim := map[*Face]float64{}
	faceAxial := map[*Face]float64{}
	addVertex := func(p Point2, z float64, source proofbound.WalkEndBound) int {
		v := base.point(p.U, p.V, z)
		mesh.vertices = append(mesh.vertices, v)
		vertexStore = append(vertexStore, proofbound.AbsSumUpper(proofbound.WalkEndBoundAllow(source),
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
			func(w survey2d.SideWalk) (*Face, error) {
				return faceOfRole(fmt.Sprintf("slab(%d).region(%d).side(%d,%d)",
					col.start, col.region, col.loopIndex, w.Segs[0]))
			})
		if err != nil {
			return nil, err
		}
		r := &rings[ci]
		r.samples, r.faces, r.sag = cl.samples, cl.faceOf, cl.maxSag
		r.segmentArea, r.walks, r.perimeterUpper = cl.segmentArea, cl.walks, cl.perimeterUpper
		r.bottom = make([]int, len(cl.samples))
		r.top = make([]int, len(cl.samples))
		mesh.areaSlack = proofbound.AbsSumUpper(mesh.areaSlack, cl.wallSlack, cl.capSlack, cl.capSlack)
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
		for _, e := range slabColumns {
			r := rings[e.column]
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
	// Every column ring carries its loop's own material winding: an outer
	// column runs CCW, a hole column CW. A patch triangulates its outer CCW and
	// its holes CW, so a ring whose column winding differs from the role it
	// plays in the patch is read reversed; a -N patch flips its triangles.
	emitPatch := func(face *Face, loops []stackedPatchLoop, reverseFace bool, axial float64) error {
		var points []Point2
		var vertices []int
		var indexLoops [][]int
		var sags []float64
		for _, pl := range loops {
			r := rings[pl.column]
			start := len(points)
			points = append(points, r.samples...)
			if pl.top {
				vertices = append(vertices, r.top...)
			} else {
				vertices = append(vertices, r.bottom...)
			}
			idx := make([]int, len(r.samples))
			for i := range idx {
				idx[i] = start + i
			}
			if (columns[pl.column].loopIndex == 0) != pl.outer {
				for a, z := 0, len(idx)-1; a < z; a, z = a+1, z-1 {
					idx[a], idx[z] = idx[z], idx[a]
				}
			}
			indexLoops = append(indexLoops, idx)
			sags = append(sags, r.sag)
			faceTrim[face] = math.Max(faceTrim[face], r.sag)
		}
		if len(loops) > 1 {
			// A union patch joins rings from two slabs, which no per-slab
			// clearance proof has compared.
			if err := requireLoopClearance(ctx, points, indexLoops, sags); err != nil {
				return err
			}
		}
		tris, err := triangulate2DContext(ctx, points, indexLoops)
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
	capLoops := func(slabColumns []stackedSlabLoop, region int, top bool) []stackedPatchLoop {
		var loops []stackedPatchLoop
		for _, e := range slabColumns {
			if e.region != region {
				continue
			}
			loops = append(loops, stackedPatchLoop{column: e.column, top: top, outer: e.loop == 0})
		}
		return loops
	}
	first, last := sp.slabs[0], sp.slabs[len(sp.slabs)-1]
	for _, end := range []struct {
		slab    int
		role    string
		top     bool
		reverse bool
		axial   float64
	}{{0, roleCapStart, false, true, first.z0Delta}, {len(sp.slabs) - 1, roleCapEnd, true, false, last.z1Delta}} {
		for r := range sp.slabs[end.slab].regions {
			face, err := stackedCapFace(b, sp, faceOfRole, end.role, end.slab, r)
			if err != nil {
				return nil, err
			}
			if err := emitPatch(face, capLoops(bySlab[end.slab], r, end.top), end.reverse, end.axial); err != nil {
				return nil, err
			}
		}
	}
	for k := range sp.interfaces {
		patches, err := stackedInterfacePatches(sp, columns, bySlab, k)
		if err != nil {
			return nil, err
		}
		for _, patch := range patches {
			face, err := faceOfRole(patch.role)
			if err != nil {
				return nil, err
			}
			if err := emitPatch(face, patch.loops, !patch.floor, sp.slabs[k].z1Delta); err != nil {
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
		allPerimeter = proofbound.AbsSumUpper(allPerimeter, r.perimeterUpper)
		if sp.sectionDelta > 0 {
			height := sp.slabs[col.end].z1 - sp.slabs[col.start].z0
			mesh.areaSlack = proofbound.AbsSumUpper(mesh.areaSlack,
				proofbound.ProductUpper(proofbound.SectionDisplacementLength(sp.sectionDelta, r.walks), height))
		}
	}
	for _, face := range b.Faces() {
		if _, ok := face.surface.(Plane); ok && sp.sectionDelta > 0 {
			mesh.areaSlack = proofbound.AbsSumUpper(mesh.areaSlack,
				proofbound.SectionDisplacementArea(sp.sectionDelta, allWalks, allPerimeter))
		}
	}
	mesh.areaSlack = proofbound.AbsSumUpper(mesh.areaSlack, meshStoreAreaAllow(&mesh, vertexStore))
	areaUpper := meshFaceAreaUpper(&mesh, vertexStore)
	terms := make([]float64, 0, len(columns)*len(sp.slabs)+len(faceAxial)+2)
	for k, slab := range sp.slabs {
		var segments, perimeter float64
		walks := 0
		for _, e := range bySlab[k] {
			r := rings[e.column]
			segments = proofbound.AbsSumUpper(segments, r.segmentArea)
			perimeter = proofbound.AbsSumUpper(perimeter, r.perimeterUpper)
			walks += r.walks
		}
		height := slab.z1 - slab.z0
		terms = append(terms, proofbound.ProductUpper(height, segments),
			proofbound.ProductUpper(height, proofbound.SectionDisplacementArea(sp.sectionDelta, walks, perimeter)))
	}
	for face, axial := range faceAxial {
		if _, ok := face.surface.(Plane); ok {
			terms = append(terms, proofbound.ProductUpper(axial, areaUpper[face]))
		}
	}
	terms = append(terms, proofbound.SweptVolumeAllow(storeMax,
		proofbound.PerturbedAreaUpper(mesh.vertices, mesh.triangles, storeMax)))
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

// stackedCapFace is the cap face with the given role over one region. A
// stack has one cap of each role. A prism group has one per region under the
// same role, so its cap is the one sharing an edge with a wall the build
// named for that region (slab(k).region(r).side(i,j)).
func stackedCapFace(b *Body, sp stackedPrismPayload, faceOfRole func(string) (*Face, error), role string, slab, region int) (*Face, error) {
	if !sp.isGroup() {
		return faceOfRole(role)
	}
	prefix := fmt.Sprintf("slab(%d).region(%d).", slab, region)
	for _, face := range b.Faces() {
		if !faceHasRole(face, role) {
			continue
		}
		for _, l := range face.loops {
			for _, ce := range l.coedges {
				for _, nb := range ce.edge.faces {
					if nb == face {
						continue
					}
					for _, origin := range nb.origins {
						if strings.HasPrefix(origin.Role, prefix) {
							return face, nil
						}
					}
				}
			}
		}
	}
	return nil, fmt.Errorf(`%w: a prism group has no %s face for region %d`, ErrDegenerate, role, region)
}

func faceHasRole(face *Face, role string) bool {
	for _, origin := range face.origins {
		if origin.Role == role {
			return true
		}
	}
	return false
}
