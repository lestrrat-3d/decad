package revolvemesh

import (
	"context"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetproof"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/r3"
)

// RimVertices constructs the certified stations and shared held vertices.
// The source must have passed the root's quarter-turn cap-rim admission.
func RimVertices(source RimSource) ([]RimStation, float64, float64, []r3.Vec, error) {
	stations, err := RimStations(source)
	if err != nil {
		return nil, 0, 0, nil, err
	}
	bound, missing, err := RimBounds(source, stations)
	if err != nil {
		return nil, 0, 0, nil, err
	}
	n := len(stations)
	verts := make([]r3.Vec, 0, 4*n-2)
	for i, s := range stations {
		verts = append(verts, r3.NewVec(source.XLo, 0, s.Z), r3.NewVec(s.X, 0, s.Z))
		if i != n-1 {
			verts = append(verts, r3.NewVec(s.X, s.Y, s.Z), r3.NewVec(source.XLo, s.Y, s.Z))
		}
	}
	return stations, bound, missing, verts, nil
}

// RimFacetsAndBounds constructs the ordered facets and their mesh bounds.
// Stations, vertices, and bound must come from RimVertices for the same source.
func RimFacetsAndBounds(ctx context.Context, source RimSource, stations []RimStation,
	verts []r3.Vec, bound float64) ([][3]int, []int, []float64, float64, float64, error) {
	n := len(stations)
	tris := make([][3]int, 0, 8*n)
	src := make([]int, 0, 8*n)
	add := func(group, a, b, c int) {
		tris = append(tris, [3]int{a, b, c})
		src = append(src, group)
	}
	quad := func(group, a, b, c, d int) {
		add(group, a, b, c)
		add(group, a, c, d)
	}
	quad(3, 0, 3, 2, 1)
	for i := range n - 1 {
		if i%256 == 0 {
			if err := ctx.Err(); err != nil {
				return nil, nil, nil, 0, 0, err
			}
		}
		a, b, c, d := 4*i, 4*i+1, 4*i+2, 4*i+3
		if i == n-2 {
			at, bt := 4*(i+1), 4*(i+1)+1
			quad(4, a, b, bt, at)
			quad(1, d, at, bt, c)
			add(2, a, at, d)
			add(0, b, c, bt)
			continue
		}
		an, bn, cn, dn := 4*(i+1), 4*(i+1)+1, 4*(i+1)+2, 4*(i+1)+3
		quad(4, a, b, bn, an)
		quad(1, d, dn, cn, c)
		quad(2, a, an, dn, d)
		group := 0
		if i < RimBlendStations {
			group = 5
		}
		quad(group, b, c, cn, bn)
	}
	if err := tessellation.RequireVertexLinks(ctx, len(verts), tris); err != nil {
		return nil, nil, nil, 0, 0,
			fmt.Errorf("%w: cap rim facets do not close at a vertex: %s", decaderr.ErrUnsupported, err)
	}
	vertexBound := make([]float64, len(verts))
	for i := range vertexBound {
		vertexBound[i] = bound
	}
	length := new(big.Rat).Sub(proofarith.FloatRat(source.XHi), proofarith.FloatRat(source.XLo))
	radius := proofarith.FloatRat(source.Radius)
	setback := proofarith.FloatRat(source.Setback)
	r2 := new(big.Rat).Mul(radius, radius)
	trueArea := new(big.Rat).Add(new(big.Rat).Mul(big.NewRat(2, 1), r2),
		new(big.Rat).Add(new(big.Rat).Mul(big.NewRat(4, 1), new(big.Rat).Mul(length, radius)),
			new(big.Rat).Mul(big.NewRat(2, 1), new(big.Rat).Mul(setback, radius))))
	areaSlack := math.Max(proofbound.RatFloatUp(trueArea), facetproof.MeshAreaUpper(verts, tris))
	diameter2 := new(big.Rat).Add(new(big.Rat).Mul(length, length), new(big.Rat).Mul(big.NewRat(2, 1), r2))
	dPair := proofbound.RatSqrtUp(diameter2)
	return tris, src, vertexBound, areaSlack, dPair, nil
}
