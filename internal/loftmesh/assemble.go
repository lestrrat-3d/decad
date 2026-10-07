package loftmesh

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/r3"
)

// Assembly is the held vertex and triangle set plus the indices consumed by
// the root topology builder. Tris begins with wall triangles, then the first
// cap and the second cap in that order.
type Assembly struct {
	Verts              []r3.Vec
	Tris               [][3]int
	Walls              int
	CapStartCount      int
	Reversed           bool
	Cell               [][2]int
	Side               []uint8
	VIdx, WIdx         [][]int
	Pts0, Pts1         []sectionrecord.Point2
	LoopIdx0, LoopIdx1 [][]int
	Delta              float64
}

// Assemble lifts paired stations once, emits two triangles per wall cell,
// triangulates both caps, and orients the complete shell from its signed
// tetrahedron sum. The callbacks retain the caller's triangulation and error
// boundaries. Their order follows the first and second cap walks.
func Assemble(ctx context.Context, pairs []LoopPair, f0, f1 r3.Frame, plane0 sectionrecord.PlaneRecord,
	xform r3.Transform, stationRound float64,
	triangulate func(context.Context, []sectionrecord.Point2, [][]int) ([][3]int, error),
	pointError func(string) error) (Assembly, error) {
	anchor := xform.Apply(plane0.Origin)
	if !proofbound.FiniteVec(anchor) {
		return Assembly{}, pointError("placed plane origin")
	}

	vIdx := make([][]int, len(pairs))
	wIdx := make([][]int, len(pairs))
	var verts []r3.Vec
	maxInputAbs := 0.0
	for i, p := range pairs {
		if err := ctx.Err(); err != nil {
			return Assembly{}, err
		}
		vIdx[i] = make([]int, len(p.V))
		for j, pt := range p.V {
			vIdx[i][j] = len(verts)
			lifted := f0.ToWorldUV(pt.U, pt.V)
			maxInputAbs = max(maxInputAbs, proofbound.VecMaxAbs(lifted))
			placed := xform.Apply(lifted)
			if !proofbound.FiniteVec(placed) {
				return Assembly{}, pointError(fmt.Sprintf("placed vertex %d of loop %d on the first profile", j, i))
			}
			verts = append(verts, placed)
		}
		wIdx[i] = make([]int, len(p.W))
		for j, pt := range p.W {
			wIdx[i][j] = len(verts)
			lifted := f1.ToWorldUV(pt.U, pt.V)
			maxInputAbs = max(maxInputAbs, proofbound.VecMaxAbs(lifted))
			placed := xform.Apply(lifted)
			if !proofbound.FiniteVec(placed) {
				return Assembly{}, pointError(fmt.Sprintf("placed vertex %d of loop %d on the second profile", j, i))
			}
			verts = append(verts, placed)
		}
	}

	var tris [][3]int
	var cell [][2]int
	var side []uint8
	for i, p := range pairs {
		if err := ctx.Err(); err != nil {
			return Assembly{}, err
		}
		n := len(p.V)
		for j := range n {
			jn := (j + 1) % n
			vj, vjn := vIdx[i][j], vIdx[i][jn]
			wj, wjn := wIdx[i][j], wIdx[i][jn]
			tris = append(tris, [3]int{vj, vjn, wjn})
			cell = append(cell, [2]int{i, j})
			side = append(side, 0)
			tris = append(tris, [3]int{vj, wjn, wj})
			cell = append(cell, [2]int{i, j})
			side = append(side, 1)
		}
	}
	walls := len(tris)

	var pts0, pts1 []sectionrecord.Point2
	var loopIdx0, loopIdx1 [][]int
	var pts0ToV, pts1ToV []int
	for i, p := range pairs {
		idx0 := make([]int, len(p.V))
		for j, pt := range p.V {
			idx0[j] = len(pts0)
			pts0 = append(pts0, pt)
			pts0ToV = append(pts0ToV, vIdx[i][j])
		}
		loopIdx0 = append(loopIdx0, idx0)

		idx1 := make([]int, len(p.W))
		for j, pt := range p.W {
			idx1[j] = len(pts1)
			pts1 = append(pts1, pt)
			pts1ToV = append(pts1ToV, wIdx[i][j])
		}
		loopIdx1 = append(loopIdx1, idx1)
	}

	tris0, err := triangulate(ctx, pts0, loopIdx0)
	if err != nil {
		return Assembly{}, err
	}
	tris1, err := triangulate(ctx, pts1, loopIdx1)
	if err != nil {
		return Assembly{}, err
	}

	for _, t := range tris0 {
		tris = append(tris, [3]int{pts0ToV[t[0]], pts0ToV[t[2]], pts0ToV[t[1]]})
	}
	capStartCount := len(tris0)
	for _, t := range tris1 {
		tris = append(tris, [3]int{pts1ToV[t[0]], pts1ToV[t[1]], pts1ToV[t[2]]})
	}

	reversed := tessellation.OrientationSign(verts, tris, anchor) < 0
	if reversed {
		for i, t := range tris {
			tris[i] = [3]int{t[0], t[2], t[1]}
		}
	}

	placeAllow := 0.0
	if xform != r3.Identity() {
		placeAllow = proofbound.RigidRoundAllow(maxInputAbs, proofbound.VecMaxAbs(xform.Translation()))
	}
	delta := proofbound.AbsSumUpper(stationRound, placeAllow)

	return Assembly{
		Verts: verts, Tris: tris, Walls: walls, CapStartCount: capStartCount,
		Reversed: reversed, Cell: cell, Side: side, VIdx: vIdx, WIdx: wIdx,
		Pts0: pts0, Pts1: pts1, LoopIdx0: loopIdx0, LoopIdx1: loopIdx1,
		Delta: delta,
	}, nil
}
