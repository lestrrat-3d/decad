package planarsnapshot

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/pair/planar"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
)

// PrismInput holds the recorded section, frame and sweep levels used to
// construct an exact planar snapshot.
type PrismInput struct {
	Outer         sectionrecord.LoopRecord
	Holes         []sectionrecord.LoopRecord
	Frame         r3.Frame
	Transform     r3.Transform
	Z0, Z1        float64
	SectionDelta  float64
	Z0Delta       float64
	Z1Delta       float64
	SurfaceResult bool
	CapStartRole  string
	CapEndRole    string
}

// PrismSolid lifts an all-line prism and checks each cap triangle's exact
// orientation. triangulate must use the same section triangulator as the mesh.
func PrismSolid(ctx context.Context, budget *proofbound.WorkBudget, in PrismInput, faceIndex map[string]int,
	triangulate func(context.Context, []sectionrecord.Point2, [][]int) ([][3]int, error)) (planar.PlanarSolid, bool, error) {
	if in.SurfaceResult || in.SectionDelta != 0 || in.Z0Delta != 0 || in.Z1Delta != 0 ||
		!finite(in.Z0, in.Z1) || in.Z0 >= in.Z1 || !PositiveAffine(in.Transform) ||
		!proofbound.FiniteVec(in.Frame.Origin()) || !proofbound.FiniteVec(in.Frame.U()) ||
		!proofbound.FiniteVec(in.Frame.V()) || !proofbound.FiniteVec(in.Frame.N()) {
		return planar.PlanarSolid{}, false, nil
	}
	fu, fv, fn := proofarith.DyVec(in.Frame.U()), proofarith.DyVec(in.Frame.V()), proofarith.DyVec(in.Frame.N())
	if proofarith.DvDot(fu, proofarith.DvCross(fv, fn)).Sign() <= 0 {
		return planar.PlanarSolid{}, false, nil
	}
	var pts []sectionrecord.Point2
	loops := make([][]int, 0, 1+len(in.Holes))
	for role, loop := range append([]sectionrecord.LoopRecord{in.Outer}, in.Holes...) {
		indices, ok := loopPoints(loop, &pts)
		if !ok {
			return planar.PlanarSolid{}, false, nil
		}
		area := loopArea(pts, indices)
		if (role == 0 && area.Sign() <= 0) || (role > 0 && area.Sign() >= 0) {
			return planar.PlanarSolid{}, false, nil
		}
		loops = append(loops, indices)
	}
	caps, err := triangulate(ctx, pts, loops)
	if err != nil {
		if ctxErr := ctx.Err(); ctxErr != nil {
			return planar.PlanarSolid{}, false, ctxErr
		}
		return planar.PlanarSolid{}, false, nil
	}
	n := len(pts)
	solid := planar.PlanarSolid{Verts: make([]proofarith.DyV3, 2*n)}
	origin := proofarith.DyVec(in.Frame.Origin())
	z := [2]proofarith.Dyadic{proofarith.MustDyOf(in.Z0), proofarith.MustDyOf(in.Z1)}
	for i, p := range pts {
		if err := budget.Step(); err != nil {
			return planar.PlanarSolid{}, false, err
		}
		local := proofarith.DvAdd(origin, proofarith.DvAdd(scaleVec(fu, proofarith.MustDyOf(p.U)),
			scaleVec(fv, proofarith.MustDyOf(p.V))))
		for level := range z {
			solid.Verts[level*n+i] = transformPoint(in.Transform,
				proofarith.DvAdd(local, scaleVec(fn, z[level])))
		}
	}
	for _, tri := range caps {
		if err := budget.Step(); err != nil {
			return planar.PlanarSolid{}, false, err
		}
		if cross2(pts[tri[0]], pts[tri[1]], pts[tri[2]]).Sign() <= 0 {
			return planar.PlanarSolid{}, false, nil
		}
		solid.Tris = append(solid.Tris, [3]int{tri[0], tri[2], tri[1]},
			[3]int{n + tri[0], n + tri[1], n + tri[2]})
	}
	start, okStart := faceIndex[in.CapStartRole]
	end, okEnd := faceIndex[in.CapEndRole]
	mapped := okStart && okEnd
	for range caps {
		solid.Faces = append(solid.Faces, start, end)
	}
	for li, loop := range loops {
		for i, from := range loop {
			to := loop[(i+1)%len(loop)]
			solid.Tris = append(solid.Tris, [3]int{from, to, n + to}, [3]int{from, n + to, n + from})
			side, ok := faceIndex[fmt.Sprintf("side(%d,%d)", li, i)]
			mapped = mapped && ok
			solid.Faces = append(solid.Faces, side, side)
		}
	}
	if !mapped {
		solid.Faces = nil
	}
	return solid, true, nil
}

func loopPoints(loop sectionrecord.LoopRecord, pts *[]sectionrecord.Point2) ([]int, bool) {
	if len(loop.Segments) < 3 {
		return nil, false
	}
	indices := make([]int, 0, len(loop.Segments))
	var first, last sectionrecord.Point2
	for i, segment := range loop.Segments {
		line, ok := segment.(sectionrecord.LineSeg)
		if !ok {
			return nil, false
		}
		start, end := line.Start, line.End
		switch {
		case line.TStart == 0 && line.TEnd == 1:
		case line.TStart == 1 && line.TEnd == 0:
			start, end = end, start
		default:
			return nil, false
		}
		if !finite(start.U, start.V, end.U, end.V) || start == end ||
			(i > 0 && start != last) {
			return nil, false
		}
		if i == 0 {
			first = start
		}
		last = end
		indices = append(indices, len(*pts))
		*pts = append(*pts, start)
	}
	if last != first {
		return nil, false
	}
	return indices, true
}

func loopArea(pts []sectionrecord.Point2, loop []int) proofarith.Dyadic {
	area := proofarith.DyZero()
	for i, from := range loop {
		a, b := pts[from], pts[loop[(i+1)%len(loop)]]
		area = proofarith.DyAdd(area, proofarith.DySubScalar(
			proofarith.DyMul(proofarith.MustDyOf(a.U), proofarith.MustDyOf(b.V)),
			proofarith.DyMul(proofarith.MustDyOf(b.U), proofarith.MustDyOf(a.V))))
	}
	return area
}

func cross2(a, b, c sectionrecord.Point2) proofarith.Dyadic {
	au, av := proofarith.MustDyOf(a.U), proofarith.MustDyOf(a.V)
	bu := proofarith.DySubScalar(proofarith.MustDyOf(b.U), au)
	bv := proofarith.DySubScalar(proofarith.MustDyOf(b.V), av)
	cu := proofarith.DySubScalar(proofarith.MustDyOf(c.U), au)
	cv := proofarith.DySubScalar(proofarith.MustDyOf(c.V), av)
	return proofarith.DySubScalar(proofarith.DyMul(bu, cv), proofarith.DyMul(bv, cu))
}

func finite(values ...float64) bool {
	for _, value := range values {
		if math.IsNaN(value) || math.IsInf(value, 0) {
			return false
		}
	}
	return true
}
