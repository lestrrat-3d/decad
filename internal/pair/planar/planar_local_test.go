package planar

import (
	"context"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/stretchr/testify/require"
)

func TestFanAtEdgeSignsMatchSegmentChecks(t *testing.T) {
	t.Parallel()
	half := proof.DyShift(proof.DyInt(1), -1)
	point := func(x, y proof.Dyadic) proof.DyV3 {
		return proof.DyV3{x, y, proof.DyZero()}
	}
	solid := &PlanarSolid{
		Verts: []proof.DyV3{
			point(proof.DyZero(), proof.DyZero()), point(proof.DyInt(1), proof.DyZero()),
			point(proof.DyZero(), proof.DyInt(1)), point(proof.DyInt(1), proof.DyInt(1)),
			point(proof.DyInt(4), proof.DyZero()), point(proof.DyInt(5), proof.DyZero()), point(proof.DyInt(6), proof.DyZero()),
		},
		Tris: [][3]int{{0, 1, 2}, {1, 3, 2}, {4, 5, 6}},
	}
	prep := preparePlanar(solid)
	queries := []hpoint{
		dyPoint(point(proof.DyInt(1), proof.DyZero())),
		dyPoint(point(half, half)),
		dyPoint(point(proof.DyShift(proof.DyInt(1), -2), proof.DyShift(proof.DyInt(1), -2))),
		dyPoint(point(proof.DyShift(proof.DyInt(9), -1), proof.DyZero())),
		dyPoint(point(proof.DyInt(3), proof.DyInt(3))),
	}
	for _, query := range queries {
		want := fanAtWithSegmentChecks(prep, query)
		got, err := (&planarKernel{poll: func() error { return nil }}).fanAt(prep, query)
		require.NoError(t, err)
		require.Equal(t, want, got)
	}
}

// fanAtWithSegmentChecks preserves the original edge-membership path as a
// reference for small fixtures, including the zero-normal facet fallback.
func fanAtWithSegmentChecks(p *planarPrep, x hpoint) fan {
	var f fan
	var tris []int
	seenVert := make(map[int]struct{})
	seenEdge := make(map[[2]int]struct{})
	for t, tri := range p.s.Tris {
		if !pointInFacet(p, t, x) {
			continue
		}
		tris = append(tris, t)
		f.normals = append(f.normals, p.normal[t])
		for i, v := range tri {
			if _, ok := seenVert[v]; !ok {
				seenVert[v] = struct{}{}
				f.verts = append(f.verts, p.s.Verts[v])
			}
			u, w := tri[i], tri[(i+1)%3]
			key := [2]int{min(u, w), max(u, w)}
			if _, ok := seenEdge[key]; ok {
				continue
			}
			seenEdge[key] = struct{}{}
			if onSegment(p.s.Verts[key[0]], p.s.Verts[key[1]], x) {
				f.edgeDirs = append(f.edgeDirs, proof.DvSub(p.s.Verts[key[1]], p.s.Verts[key[0]]))
			}
		}
	}
	for i, t := range tris {
		normal := f.normals[i]
		tri := p.s.Tris[t]
		a := p.s.Verts[tri[0]]
		support := true
		for _, v := range f.verts {
			if proof.DvDot(normal, proof.DvSub(v, a)).Sign() > 0 {
				support = false
				break
			}
		}
		if support {
			f.supports = append(f.supports, normal)
		}
		for _, other := range f.normals[:i] {
			if proof.DvIsZero(proof.DvCross(other, normal)) && proof.DvDot(other, normal).Sign() < 0 {
				f.folded = true
			}
		}
	}
	return f
}

// A tiny positive gap can underflow in the distance lower bound. The old
// candidate must remain in the fan scan so its poll error is observed.
func TestFanAtTinyGapPreservesPollError(t *testing.T) {
	tiny := proof.DyShift(proof.DyInt(1), -600)
	zero, one := proof.DyZero(), proof.DyInt(1)
	prep := preparePlanar(&PlanarSolid{
		Verts: []proof.DyV3{{tiny, zero, zero}, {tiny, one, zero}, {tiny, zero, one}},
		Tris:  [][3]int{{0, 1, 2}},
	})
	query := dyPoint(proof.DyV3{zero, zero, zero})
	prep.floatBoxes()
	xb, boxed := query.floatBox()
	require.True(t, boxed)
	require.Equal(t, []int{0}, prep.facetsAt(xb, boxed))
	_, err := (&planarKernel{poll: func() error { return ErrWorkLimit }}).fanAt(prep, query)
	require.ErrorIs(t, err, ErrWorkLimit)
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	_, err = (&planarKernel{poll: ctx.Err}).fanAt(prep, query)
	require.ErrorIs(t, err, context.Canceled)
}

func TestFacetsAtKeepsFlatAndUnboxedTriangles(t *testing.T) {
	zero, one, far := proof.DyZero(), proof.DyInt(1), proof.DyInt(10)
	prep := preparePlanar(&PlanarSolid{
		Verts: []proof.DyV3{
			{far, zero, zero}, {far, one, zero}, {far, zero, one},
			{far, zero, zero}, {far, one, zero}, {far, one, zero},
			{zero, zero, zero}, {zero, one, zero}, {zero, zero, one},
		},
		Tris: [][3]int{{0, 1, 2}, {3, 4, 5}, {6, 7, 8}},
	})
	prep.floatBoxes()
	xb, boxed := dyPoint(proof.DyV3{zero, zero, zero}).floatBox()
	require.True(t, boxed)
	require.Equal(t, []int{1, 2}, prep.facetsAt(xb, true))
	require.Equal(t, []int{0, 1, 2}, prep.facetsAt(floatBox{}, false))
}
