package decad

import (
	"context"
	"fmt"
	"math"
	"slices"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/stationbound"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/decad/internal/triangulation"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// tessellateAllEdgeChamfer tiles every planar face from its own exact corner
// vertices and chords the unchanged circular bore once for its two cap holes
// and wall. The only denoted-to-held volume difference is that bore's chord
// sliver plus the certified station-position displacement.
func tessellateAllEdgeChamfer(ctx context.Context, body *Body, p allEdgeChamferPayload,
	chord float64, verify Verification) (*Mesh, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	geometry, _, err := p.placedGeometry()
	if err != nil {
		return nil, err
	}
	p = geometry
	faces := body.Faces()
	if len(faces) != 19 {
		return nil, fmt.Errorf(`%w: the all-edge chamfer has no 18 planes and one bore`, ErrUnsupported)
	}
	loop := sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{sectionrecord.CircleSeg{
		Center: sectionrecord.Point2{U: p.center[0], V: p.center[1]},
		Radius: units.Millimeters(p.radius), CCW: true, TStart: 0, TEnd: 1,
	}}}
	heightMeasure := proofbound.BoundedSub(proofbound.ExactScalar(p.box[1][1]),
		proofbound.ExactScalar(p.box[1][0]))
	height := proofbound.AbsSumUpper(heightMeasure.Value, heightMeasure.Bound)
	samples, err := tessellation.ChordLoop(ctx, loop, chord, height, freeform.NewFreeformWork(), nil, 0,
		func(_ survey2d.SideWalk) (*Face, error) { return faces[18], nil }, stationbound.ChordStationBound)
	if err != nil {
		return nil, err
	}
	if len(samples.Samples) < 3 {
		return nil, fmt.Errorf(`%w: the bore has too few stations`, ErrUnsupported)
	}
	var mesh Mesh
	index := map[r3.Vec]int{}
	var store []float64
	add := func(v r3.Vec, bound float64) int {
		if i, ok := index[v]; ok {
			store[i] = math.Max(store[i], bound)
			return i
		}
		index[v] = len(mesh.vertices)
		mesh.vertices = append(mesh.vertices, v)
		store = append(store, bound)
		return len(mesh.vertices) - 1
	}
	circle := [2][]int{make([]int, len(samples.Samples)), make([]int, len(samples.Samples))}
	for side := range 2 {
		for i, sample := range samples.Samples {
			if err := ctx.Err(); err != nil {
				return nil, err
			}
			v := r3.NewVec(sample.U, p.box[1][side], sample.V)
			circle[side][i] = add(v, proofbound.WalkEndBoundAllow(samples.BoundOf[i]))
		}
	}
	for fi, face := range faces[:18] {
		plane, ok := face.surface.(Plane)
		if !ok || len(face.loops) == 0 {
			return nil, fmt.Errorf(`%w: all-edge chamfer face %d is not planar`, ErrUnsupported, fi)
		}
		var points []sectionrecord.Point2
		var meshIndices []int
		outer := make([]int, 0, len(face.loops[0].coedges))
		for _, ce := range face.loops[0].coedges {
			v := ce.Start().position
			local := plane.Frame.ToLocal(v)
			points = append(points, sectionrecord.Point2{U: local.X, V: local.Y})
			meshIndices = append(meshIndices, add(v, 0))
			outer = append(outer, len(points)-1)
		}
		loops := [][]int{outer}
		if fi == 2 || fi == 3 {
			side := fi - 2
			hole := make([]int, 0, len(circle[side]))
			for _, meshIndex := range circle[side] {
				v := mesh.vertices[meshIndex]
				local := plane.Frame.ToLocal(v)
				points = append(points, sectionrecord.Point2{U: local.X, V: local.Y})
				meshIndices = append(meshIndices, meshIndex)
				hole = append(hole, len(points)-1)
			}
			if signedLoopArea(points, hole) > 0 {
				slices.Reverse(hole)
			}
			loops = append(loops, hole)
		}
		tris, err := triangulation.Triangulate(ctx, points, loops)
		if err != nil {
			return nil, err
		}
		for _, tri := range tris {
			mesh.addTriangle([3]int{meshIndices[tri[0]], meshIndices[tri[1]], meshIndices[tri[2]]}, face)
		}
	}
	for i := range circle[0] {
		j := (i + 1) % len(circle[0])
		lo0, lo1 := circle[0][i], circle[0][j]
		hi0, hi1 := circle[1][i], circle[1][j]
		mesh.addTriangle([3]int{lo0, lo1, hi0}, faces[18])
		mesh.addTriangle([3]int{lo1, hi1, hi0}, faces[18])
	}
	if err := liftTessellationError(tessellation.RequireClosedMesh(mesh.triangles)); err != nil {
		return nil, err
	}
	if err := tessellation.RequireVertexLinks(ctx, len(mesh.vertices), mesh.triangles); err != nil {
		return nil, err
	}
	anchor := r3.NewVec(p.box[0][0], p.box[1][0], p.box[2][0])
	if tessellation.OrientationSign(mesh.vertices, mesh.triangles, anchor) <= 0 {
		return nil, fmt.Errorf(`%w: the all-edge chamfer mesh has no positive enclosed volume`, ErrUnsupported)
	}
	if verify >= VerifyBoundary {
		if err := loftmesh.LoftCrossingAudit(proofbound.NewWorkBudget(ctx), mesh.vertices, mesh.triangles); err != nil {
			return nil, err
		}
	}
	trim := map[*Face]float64{faces[2]: samples.MaxSag, faces[3]: samples.MaxSag,
		faces[18]: samples.MaxSag}
	if err := composeFaceBounds(&mesh, trim, nil, store, 0); err != nil {
		return nil, err
	}
	mesh.setVertexBounds(store)
	mesh.areaSlack = proofbound.AbsSumUpper(samples.WallSlack,
		proofbound.ProductUpper(2, samples.CapSlack),
		tessellation.StoreAreaAllow(mesh.vertices, mesh.triangles, store))
	if verify >= VerifyAll {
		storeMax, err := tessellation.StoreMax(store)
		if err != nil {
			return nil, err
		}
		terms := []float64{proofbound.ProductUpper(height, samples.SegmentArea),
			proofbound.SweptVolumeAllow(storeMax,
				proofbound.PerturbedAreaUpper(mesh.vertices, mesh.triangles, storeMax))}
		if err := publishSymDiff(&mesh, terms); err != nil {
			return nil, err
		}
	}
	return &mesh, nil
}

func signedLoopArea(points []sectionrecord.Point2, loop []int) float64 {
	var sum float64
	for i, id := range loop {
		a, b := points[id], points[loop[(i+1)%len(loop)]]
		sum += a.U*b.V - a.V*b.U
	}
	return sum
}
