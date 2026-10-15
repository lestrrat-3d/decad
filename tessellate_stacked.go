package decad

import (
	"context"
	"fmt"
	"math"
	"strings"

	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/stackedrecord"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/triangulation"

	"github.com/lestrrat-3d/decad/internal/survey2d"

	"github.com/lestrrat-3d/decad/internal/proofbound"

	"github.com/lestrrat-3d/decad/internal/stationbound"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/units"
)

func tessellateStacked(ctx context.Context, b *Body, sp stackedPrismPayload, chord float64, verify Verification) (*Mesh, error) {
	if !sp.OpposedSectionExact() {
		return nil, fmt.Errorf(`%w: opposed holes at a stacked interface carry section displacement`, ErrUnsupported)
	}
	if err := stackedrecord.Falsify(ctx, stackedrecord.Record{Slabs: sp.Slabs, Interfaces: sp.Interfaces}); err != nil {
		return nil, err
	}
	columns, bySlab, err := sp.Columns()
	if err != nil {
		return nil, err
	}
	budget := chord
	if sp.SectionDelta > 0 {
		budget = freeform.DownRound(freeform.DownRound(chord - sp.SectionDelta))
		if budget <= 0 {
			return nil, fmt.Errorf(`%w: requested tolerance %s leaves no chord budget above the body's own section displacement %s`,
				ErrUnsupported, units.Millimeters(chord), units.Millimeters(sp.SectionDelta))
		}
	}
	byRole := map[string]*topo.Face{}
	for _, face := range b.Faces() {
		for _, origin := range face.Origins() {
			byRole[origin.Role] = faceNode(face)
		}
	}
	faceOfRole := func(role string) (*topo.Face, error) {
		face := byRole[role]
		if face == nil {
			return nil, fmt.Errorf(`%w: a stacked prism has no face for role %q`, ErrDegenerate, role)
		}
		return face, nil
	}
	mesh := bornMesh(new(Mesh))
	base := sp.OuterPrism()
	work := freeform.NewFreeformWork()
	var vertexStore []float64
	faceTrim := map[*topo.Face]float64{}
	faceAxial := map[*topo.Face]float64{}
	addVertex := func(p planePoint, z float64, source proofbound.WalkEndBound) int {
		v := base.Point(p.U, p.V, z)
		mesh.n.Vertices = append(mesh.n.Vertices, v)
		vertexStore = append(vertexStore, proofbound.AbsSumUpper(proofbound.WalkEndBoundAllow(source),
			base.PointRound(p.U, p.V, z, v)))
		return len(mesh.n.Vertices) - 1
	}
	rings := make([]tessellation.StackedRing[*topo.Face], len(columns))
	for ci, col := range columns {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		first, last := sp.Slabs[col.Start], sp.Slabs[col.End]
		cl, err := tessellation.ChordLoop(ctx, col.Loop, budget, last.Z1-first.Z0, work, nil, col.LoopIndex,
			func(w survey2d.SideWalk) (*topo.Face, error) {
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
		mesh.n.AreaSlack = proofbound.AbsSumUpper(mesh.n.AreaSlack, cl.WallSlack, cl.CapSlack, cl.CapSlack)
		for j, p := range cl.Samples {
			r.Bottom[j] = addVertex(p, first.Z0, cl.BoundOf[j])
			r.Top[j] = addVertex(p, last.Z1, cl.BoundOf[j])
		}
		for j, face := range cl.FaceOf {
			next := (j + 1) % len(cl.Samples)
			topobuild.AddTriangle(&mesh.n, [3]int{r.Bottom[j], r.Bottom[next], r.Top[next]}, face)
			topobuild.AddTriangle(&mesh.n, [3]int{r.Bottom[j], r.Top[next], r.Top[j]}, face)
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
		if err := tessellation.StackedSlabClearance(ctx, rings, columnsInSlab, tessellation.RequireLoopClearance); err != nil {
			return nil, err
		}
	}
	// Every column ring carries its loop's own material winding: an outer
	// column runs CCW, a hole column CW. A patch triangulates its outer CCW and
	// its holes CW, so a ring whose column winding differs from the role it
	// plays in the patch is read reversed; a -N patch flips its triangles.
	emitPatch := func(face *topo.Face, loops []stackedrecord.PatchLoop, reverseFace bool, axial float64) error {
		patchLoops := make([]tessellation.StackedPatchLoop, len(loops))
		for i, loop := range loops {
			patchLoops[i] = tessellation.StackedPatchLoop{
				Column: loop.Column, Top: loop.Top, Outer: loop.Outer,
				ColumnOuter: columns[loop.Column].LoopIndex == 0,
			}
		}
		patch, err := tessellation.StackedPatchTriangles(ctx, rings, patchLoops, reverseFace,
			tessellation.RequireLoopClearance, triangulation.Triangulate)
		if err != nil {
			return err
		}
		for _, tri := range patch.Triangles {
			topobuild.AddTriangle(&mesh.n, tri, face)
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
	first, last := sp.Slabs[0], sp.Slabs[len(sp.Slabs)-1]
	for _, end := range []struct {
		slab    int
		role    string
		top     bool
		reverse bool
		axial   float64
	}{{0, provenance.RoleCapStart, false, true, first.Z0Delta}, {len(sp.Slabs) - 1, provenance.RoleCapEnd, true, false, last.Z1Delta}} {
		for r := range sp.Slabs[end.slab].Regions {
			face, err := stackedCapFace(b, sp, faceOfRole, end.role, end.slab, r)
			if err != nil {
				return nil, err
			}
			if err := emitPatch(face, capLoops(bySlab[end.slab], r, end.top), end.reverse, end.axial); err != nil {
				return nil, err
			}
		}
	}
	for k := range sp.Interfaces {
		patches, err := sp.InterfacePatches(columns, bySlab, k)
		if err != nil {
			return nil, err
		}
		for _, patch := range patches {
			face, err := faceOfRole(patch.Role)
			if err != nil {
				return nil, err
			}
			if err := emitPatch(face, patch.Loops, !patch.Floor, sp.Slabs[k].Z1Delta); err != nil {
				return nil, err
			}
		}
	}
	if base.Reflected() {
		for i := range mesh.n.Triangles {
			mesh.n.Triangles[i][1], mesh.n.Triangles[i][2] = mesh.n.Triangles[i][2], mesh.n.Triangles[i][1]
		}
	}
	if err := topobuild.LiftAuditError(tessellation.RequireClosedMesh(mesh.n.Triangles)); err != nil {
		return nil, err
	}
	if err := composeFaceBounds(mesh, faceTrim, faceAxial, vertexStore, sp.SectionDelta); err != nil {
		return nil, err
	}
	if verify < VerifyAll {
		return mesh, nil
	}
	columnHeights := make([]float64, len(columns))
	for i, column := range columns {
		columnHeights[i] = sp.Slabs[column.End].Z1 - sp.Slabs[column.Start].Z0
	}
	slabHeights := make([]float64, len(sp.Slabs))
	slabColumns := make([][]int, len(bySlab))
	for k, slab := range sp.Slabs {
		slabHeights[k] = slab.Z1 - slab.Z0
		slabColumns[k] = make([]int, len(bySlab[k]))
		for i, entry := range bySlab[k] {
			slabColumns[k][i] = entry.Column
		}
	}
	proof := tessellation.ProveStacked(tessellation.StackedProofInput[*topo.Face]{
		Rings: rings, ColumnHeights: columnHeights, SlabHeights: slabHeights,
		SlabColumns: slabColumns, SectionDelta: sp.SectionDelta,
		Faces: topobuild.Faces(&b.n), Source: mesh.n.Source, FaceAxial: faceAxial,
		IsPlanar: func(face *topo.Face) bool { _, ok := face.Surface.(surfacegeom.Plane); return ok },
		Vertices: mesh.n.Vertices, Triangles: mesh.n.Triangles,
		Store: vertexStore, StoreMax: storeMax, AreaSlack: mesh.n.AreaSlack,
	})
	mesh.n.AreaSlack = proof.AreaSlack
	if err := publishSymDiff(mesh, proof.Terms); err != nil {
		return nil, err
	}
	return mesh, nil
}

// stackedCapFace is the cap face with the given role over one region. A
// stack has one cap of each role. A prism group has one per region under the
// same role, so its cap is the one sharing an edge with a wall the build
// named for that region (slab(k).region(r).side(i,j)).
func stackedCapFace(b *Body, sp stackedPrismPayload, faceOfRole func(string) (*topo.Face, error), role string, slab, region int) (*topo.Face, error) {
	if !sp.IsGroup() {
		return faceOfRole(role)
	}
	prefix := fmt.Sprintf("slab(%d).region(%d).", slab, region)
	for _, face := range b.Faces() {
		if !faceHasRole(face, role) {
			continue
		}
		for _, l := range face.n.Loops {
			for _, ce := range l.CoEdges {
				for _, nb := range ce.Edge.Faces {
					if nb == faceNode(face) {
						continue
					}
					for _, origin := range nb.Origins {
						if strings.HasPrefix(origin.Role, prefix) {
							return faceNode(face), nil
						}
					}
				}
			}
		}
	}
	return nil, fmt.Errorf(`%w: a prism group has no %s face for region %d`, ErrDegenerate, role, region)
}

func faceHasRole(face *Face, role string) bool {
	for _, origin := range face.n.Origins {
		if origin.Role == role {
			return true
		}
	}
	return false
}
