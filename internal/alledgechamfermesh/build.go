// Package alledgechamfermesh builds the all-edge Chamfer mesh over topology nodes.
package alledgechamfermesh

import (
	"context"
	"fmt"
	"math"
	"slices"

	"github.com/lestrrat-3d/decad/internal/breprecord"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/stationbound"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/decad/internal/triangulation"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// Build tiles every planar face from its own exact corner vertices and chords
// the unchanged circular bore once for its two cap holes and wall. The only
// denoted-to-held volume difference is that bore's chord sliver plus the
// certified station-position displacement. A record without a bore has
// neither, and its mesh is the polyhedron of its exact corners. ctx and body
// must be non-nil.
func Build(ctx context.Context, body *topo.Body, p breprecord.AllEdgeChamferPayload,
	chord float64, verifyBoundary, verifyAll bool) (*topo.Mesh, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	geometry, _, err := p.PlacedGeometry()
	if err != nil {
		return nil, err
	}
	p = geometry
	faces := topobuild.Faces(body)
	planes := 18
	want := planes
	if p.Bored() {
		want++
	}
	if len(faces) != want {
		return nil, fmt.Errorf(`%w: the all-edge chamfer has no 18 planes and its bore`, decaderr.ErrUnsupported)
	}
	var samples tessellation.ChordSamples[*topo.Face]
	height := 0.0
	if p.Bored() {
		loop := sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{sectionrecord.CircleSeg{
			Center: sectionrecord.Point2{U: p.Center[0], V: p.Center[1]},
			Radius: units.Millimeters(p.Radius), CCW: true, TStart: 0, TEnd: 1,
		}}}
		heightMeasure := proofbound.BoundedSub(proofbound.ExactScalar(p.Box[1][1]),
			proofbound.ExactScalar(p.Box[1][0]))
		height = proofbound.AbsSumUpper(heightMeasure.Value, heightMeasure.Bound)
		var err error
		samples, err = tessellation.ChordLoop(ctx, loop, chord, height, freeform.NewFreeformWork(), nil, 0,
			func(_ survey2d.SideWalk) (*topo.Face, error) { return faces[18], nil }, stationbound.ChordStationBound)
		if err != nil {
			return nil, err
		}
		if len(samples.Samples) < 3 {
			return nil, fmt.Errorf(`%w: the bore has too few stations`, decaderr.ErrUnsupported)
		}
	}
	mesh := new(topo.Mesh)
	index := map[r3.Vec]int{}
	var store []float64
	add := func(v r3.Vec, bound float64) int {
		if i, ok := index[v]; ok {
			store[i] = math.Max(store[i], bound)
			return i
		}
		index[v] = len(mesh.Vertices)
		mesh.Vertices = append(mesh.Vertices, v)
		store = append(store, bound)
		return len(mesh.Vertices) - 1
	}
	circle := [2][]int{make([]int, len(samples.Samples)), make([]int, len(samples.Samples))}
	for side := range 2 {
		for i, sample := range samples.Samples {
			if err := ctx.Err(); err != nil {
				return nil, err
			}
			v := r3.NewVec(sample.U, p.Box[1][side], sample.V)
			circle[side][i] = add(v, proofbound.WalkEndBoundAllow(samples.BoundOf[i]))
		}
	}
	for fi, face := range faces[:planes] {
		plane, ok := face.Surface.(surfacegeom.Plane)
		if !ok || len(face.Loops) == 0 {
			return nil, fmt.Errorf(`%w: all-edge chamfer face %d is not planar`, decaderr.ErrUnsupported, fi)
		}
		var points []sectionrecord.Point2
		var meshIndices []int
		outer := make([]int, 0, len(face.Loops[0].CoEdges))
		for _, ce := range face.Loops[0].CoEdges {
			v := topobuild.CoEdgeStart(ce).Position
			local := plane.Frame.ToLocal(v)
			points = append(points, sectionrecord.Point2{U: local.X, V: local.Y})
			meshIndices = append(meshIndices, add(v, 0))
			outer = append(outer, len(points)-1)
		}
		loops := [][]int{outer}
		if p.Bored() && (fi == 2 || fi == 3) {
			side := fi - 2
			hole := make([]int, 0, len(circle[side]))
			for _, meshIndex := range circle[side] {
				v := mesh.Vertices[meshIndex]
				local := plane.Frame.ToLocal(v)
				points = append(points, sectionrecord.Point2{U: local.X, V: local.Y})
				meshIndices = append(meshIndices, meshIndex)
				hole = append(hole, len(points)-1)
			}
			if triangulation.SignedLoopArea(points, hole) > 0 {
				slices.Reverse(hole)
			}
			loops = append(loops, hole)
		}
		tris, err := triangulation.Triangulate(ctx, points, loops)
		if err != nil {
			return nil, err
		}
		for _, tri := range tris {
			topobuild.AddTriangle(mesh, [3]int{meshIndices[tri[0]], meshIndices[tri[1]], meshIndices[tri[2]]}, face)
		}
	}
	for i := range circle[0] {
		j := (i + 1) % len(circle[0])
		lo0, lo1 := circle[0][i], circle[0][j]
		hi0, hi1 := circle[1][i], circle[1][j]
		topobuild.AddTriangle(mesh, [3]int{lo0, lo1, hi0}, faces[18])
		topobuild.AddTriangle(mesh, [3]int{lo1, hi1, hi0}, faces[18])
	}
	if err := topobuild.RequireClosedMesh(mesh); err != nil {
		return nil, err
	}
	if err := tessellation.RequireVertexLinks(ctx, len(mesh.Vertices), mesh.Triangles); err != nil {
		return nil, err
	}
	anchor := r3.NewVec(p.Box[0][0], p.Box[1][0], p.Box[2][0])
	if tessellation.OrientationSign(mesh.Vertices, mesh.Triangles, anchor) <= 0 {
		return nil, fmt.Errorf(`%w: the all-edge chamfer mesh has no positive enclosed volume`, decaderr.ErrUnsupported)
	}
	if verifyBoundary {
		if err := loftmesh.LoftCrossingAudit(proofbound.NewWorkBudget(ctx), mesh.Vertices, mesh.Triangles); err != nil {
			return nil, err
		}
	}
	trim := map[*topo.Face]float64{}
	if p.Bored() {
		trim = map[*topo.Face]float64{faces[2]: samples.MaxSag, faces[3]: samples.MaxSag,
			faces[18]: samples.MaxSag}
	}
	bounds, err := tessellation.FaceBounds(mesh.Triangles, mesh.Source, trim, nil, store, 0)
	if err != nil {
		return nil, err
	}
	for face, bound := range bounds {
		topobuild.SetFaceBound(mesh, face, bound)
	}
	mesh.VertexBound = append([]float64(nil), store...)
	for i, tri := range mesh.Triangles {
		topobuild.SetFaceBound(mesh, mesh.Source[i], max(store[tri[0]], store[tri[1]], store[tri[2]]))
	}
	mesh.AreaSlack = proofbound.AbsSumUpper(samples.WallSlack,
		proofbound.ProductUpper(2, samples.CapSlack),
		tessellation.StoreAreaAllow(mesh.Vertices, mesh.Triangles, store))
	if verifyAll {
		storeMax, err := tessellation.StoreMax(store)
		if err != nil {
			return nil, err
		}
		terms := []float64{proofbound.SweptVolumeAllow(storeMax,
			proofbound.PerturbedAreaUpper(mesh.Vertices, mesh.Triangles, storeMax))}
		if p.Bored() {
			terms = append([]float64{proofbound.ProductUpper(height, samples.SegmentArea)}, terms...)
		}
		total, err := tessellation.SymDiff(terms)
		if err != nil {
			return nil, err
		}
		mesh.VolSymDiff = total
		mesh.SymDiffOK = true
	}
	return mesh, nil
}
