package planar

import (
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
