package decad

import (
	"context"
	"fmt"
	"math"
	"strings"

	"github.com/lestrrat-3d/decad/internal/stackedrecord"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/triangulation"

	"github.com/lestrrat-3d/decad/internal/survey2d"

	"github.com/lestrrat-3d/decad/internal/proofbound"

	"github.com/lestrrat-3d/decad/internal/stationbound"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/units"
)

func tessellateStacked(ctx context.Context, b *Body, sp stackedPrismPayload, chord float64, verify Verification) (*Mesh, error) {
	if err := stackedrecord.Falsify(ctx, stackedrecord.Record{Slabs: sp.slabs, Interfaces: sp.interfaces}); err != nil {
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
	rings := make([]tessellation.StackedRing[*Face], len(columns))
	for ci, col := range columns {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		first, last := sp.slabs[col.Start], sp.slabs[col.End]
		cl, err := tessellation.ChordLoop(ctx, col.Loop, budget, last.Z1-first.Z0, work, nil, col.LoopIndex,
			func(w survey2d.SideWalk) (*Face, error) {
				return faceOfRole(fmt.Sprintf("slab(%d).region(%d).side(%d,%d)",
					col.Start, col.Region, col.LoopIndex, w.Segs[0]))
			}, stationbound.ChordStationBound)
		if err != nil {
			return nil, err
		}
		r := &rings[ci]
		r.Samples, r.Faces, r.Sag = cl.Samples, cl.FaceOf, cl.MaxSag
		r.SegmentArea, r.Walks, r.PerimeterUpper = cl.SegmentArea, cl.Walks, cl.PerimeterUpper
		r.Bottom = make([]int, len(cl.Samples))
		r.Top = make([]int, len(cl.Samples))
		mesh.areaSlack = proofbound.AbsSumUpper(mesh.areaSlack, cl.WallSlack, cl.CapSlack, cl.CapSlack)
		for j, p := range cl.Samples {
			r.Bottom[j] = addVertex(p, first.Z0, cl.BoundOf[j])
			r.Top[j] = addVertex(p, last.Z1, cl.BoundOf[j])
		}
		for j, face := range cl.FaceOf {
			next := (j + 1) % len(cl.Samples)
			mesh.addTriangle([3]int{r.Bottom[j], r.Bottom[next], r.Top[next]}, face)
			mesh.addTriangle([3]int{r.Bottom[j], r.Top[next], r.Top[j]}, face)
			faceTrim[face] = math.Max(faceTrim[face], cl.SagOf[j])
			faceAxial[face] = math.Max(first.Z0Delta, last.Z1Delta)
		}
	}
	storeMax, err := tessellation.StoreMax(vertexStore)
	if err != nil {
		return nil, err
	}
	for _, slabColumns := range bySlab {
		columnsInSlab := make([]int, len(slabColumns))
		for i, entry := range slabColumns {
			columnsInSlab[i] = entry.Column
		}
		if err := tessellation.StackedSlabClearance(ctx, rings, columnsInSlab, requireRecordLoopClearance); err != nil {
			return nil, err
		}
	}
	// Every column ring carries its loop's own material winding: an outer
	// column runs CCW, a hole column CW. A patch triangulates its outer CCW and
	// its holes CW, so a ring whose column winding differs from the role it
	// plays in the patch is read reversed; a -N patch flips its triangles.
	emitPatch := func(face *Face, loops []stackedrecord.PatchLoop, reverseFace bool, axial float64) error {
		patchLoops := make([]tessellation.StackedPatchLoop, len(loops))
		for i, loop := range loops {
			patchLoops[i] = tessellation.StackedPatchLoop{
				Column: loop.Column, Top: loop.Top, Outer: loop.Outer,
				ColumnOuter: columns[loop.Column].LoopIndex == 0,
			}
		}
		patch, err := tessellation.StackedPatchTriangles(ctx, rings, patchLoops, reverseFace,
			requireRecordLoopClearance, triangulation.Triangulate)
		if err != nil {
			return err
		}
		for _, tri := range patch.Triangles {
			mesh.addTriangle(tri, face)
		}
		faceTrim[face] = math.Max(faceTrim[face], patch.Sag)
		faceAxial[face] = axial
		return nil
	}
	capLoops := func(slabColumns []stackedrecord.SlabLoop, region int, top bool) []stackedrecord.PatchLoop {
		var loops []stackedrecord.PatchLoop
		for _, e := range slabColumns {
			if e.Region != region {
				continue
			}
			loops = append(loops, stackedrecord.PatchLoop{Column: e.Column, Top: top, Outer: e.Loop == 0})
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
	}{{0, roleCapStart, false, true, first.Z0Delta}, {len(sp.slabs) - 1, roleCapEnd, true, false, last.Z1Delta}} {
		for r := range sp.slabs[end.slab].Regions {
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
			face, err := faceOfRole(patch.Role)
			if err != nil {
				return nil, err
			}
			if err := emitPatch(face, patch.Loops, !patch.Floor, sp.slabs[k].Z1Delta); err != nil {
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
	columnHeights := make([]float64, len(columns))
	for i, column := range columns {
		columnHeights[i] = sp.slabs[column.End].Z1 - sp.slabs[column.Start].Z0
	}
	slabHeights := make([]float64, len(sp.slabs))
	slabColumns := make([][]int, len(bySlab))
	for k, slab := range sp.slabs {
		slabHeights[k] = slab.Z1 - slab.Z0
		slabColumns[k] = make([]int, len(bySlab[k]))
		for i, entry := range bySlab[k] {
			slabColumns[k][i] = entry.Column
		}
	}
	proof := tessellation.ProveStacked(tessellation.StackedProofInput[*Face]{
		Rings: rings, ColumnHeights: columnHeights, SlabHeights: slabHeights,
		SlabColumns: slabColumns, SectionDelta: sp.sectionDelta,
		Faces: b.Faces(), Source: mesh.source, FaceAxial: faceAxial,
		IsPlanar: func(face *Face) bool { _, ok := face.surface.(Plane); return ok },
		Vertices: mesh.vertices, Triangles: mesh.triangles,
		Store: vertexStore, StoreMax: storeMax, AreaSlack: mesh.areaSlack,
	})
	mesh.areaSlack = proof.AreaSlack
	if err := publishSymDiff(&mesh, proof.Terms); err != nil {
		return nil, err
	}
	return &mesh, nil
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
