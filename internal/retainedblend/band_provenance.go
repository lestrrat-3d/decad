package retainedblend

import (
	"math/big"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// Exact on-plane provenance of a band's end plane (agent-docs/modify-design.md Part E
// §4.3). A held vertex lies on end plane G by construction when it bounds a face
// whose retained plane tag is G's tag exactly. Retention Claim R puts every true
// point of such a face's patch on its tag, and a vertex's true point lies in the
// closure of every face it bounds, so that point lies on G, and the vertex's own
// bound already covers its distance from it. The reading states a vertex's
// construction; it never admits a residual.

// tagPoint is a plane tag read as exact rationals: a point on the plane and the
// normal's three components.
type tagPoint struct {
	o, n [3]*big.Rat
}

// planeRats reads f's plane tag as rationals, or false for a face that is not a
// plane or holds a value that is no rational.
func planeRats(f *topo.Face) (tagPoint, bool) {
	pl, ok := f.Surface.(surfacegeom.Plane)
	if !ok {
		return tagPoint{}, false
	}
	n, o := pl.Frame.N(), pl.Frame.Origin()
	var t tagPoint
	for i, x := range [3]float64{n.X, n.Y, n.Z} {
		if t.n[i] = proof.FloatRat(x); t.n[i] == nil {
			return tagPoint{}, false
		}
	}
	for i, x := range [3]float64{o.X, o.Y, o.Z} {
		if t.o[i] = proof.FloatRat(x); t.o[i] == nil {
			return tagPoint{}, false
		}
	}
	return t, true
}

// exactTag reads f's plane tag as rationals, or false when f is not a retained
// plane whose tag states no departure: the tag equals its record
// (tagIsRecord), departs from its denoted patch by nothing (TagDeparture and
// NormalBound zero) and lies off the true plane by nothing (AxialDelta zero).
func exactTag(f *topo.Face) (tagPoint, bool) {
	if f == nil || !f.RetainedTag || f.TagDeparture != 0 || f.NormalBound != 0 || !tagIsRecord(f) {
		return tagPoint{}, false
	}
	if f.HasAxialDelta && f.AxialDelta != 0 {
		return tagPoint{}, false
	}
	return planeRats(f)
}

// sameTagPlane reports whether the plane tag of f is the plane of g's tag
// exactly: the normals are parallel and f's origin lies on g's plane, both read
// on the tags' floats as rationals. Only g's tag is read, not its record, so g
// may state a displacement: the band charges g's own displacement into its slack,
// as it does for the corner it already places on g.
func sameTagPlane(f, g *topo.Face) bool {
	tf, ok := exactTag(f)
	if !ok || g == nil {
		return false
	}
	tg, ok := planeRats(g)
	if !ok {
		return false
	}
	mul := func(a, b *big.Rat) *big.Rat { return new(big.Rat).Mul(a, b) }
	sub := func(a, b *big.Rat) *big.Rat { return new(big.Rat).Sub(a, b) }
	zeroF, zeroG := true, true
	for i := range 3 {
		zeroF = zeroF && tf.n[i].Sign() == 0
		zeroG = zeroG && tg.n[i].Sign() == 0
	}
	if zeroF || zeroG {
		return false
	}
	// n_f × n_g = 0.
	for i := range 3 {
		j, k := (i+1)%3, (i+2)%3
		if sub(mul(tf.n[j], tg.n[k]), mul(tf.n[k], tg.n[j])).Sign() != 0 {
			return false
		}
	}
	// n_g · (o_f − o_g) = 0.
	sum := new(big.Rat)
	for i := range 3 {
		sum.Add(sum, mul(tg.n[i], sub(tf.o[i], tg.o[i])))
	}
	return sum.Sign() == 0
}

// onTagPlane holds the held vertices that lie on the plane tag of face g by
// construction: every vertex of the boundary of a face whose retained tag is g's
// exactly (sameTagPlane). A vertex inside a face's fan is not admitted. It is
// empty when the work holds no face map.
func (w *work) onTagPlane(g *topo.Face) map[int]bool {
	on := map[int]bool{}
	for f, fi := range w.faceIx {
		if !sameTagPlane(f, g) {
			continue
		}
		type edge struct{ a, b int }
		have := map[edge]struct{}{}
		facets := facetsOfFace(w.fp.FaceOf, fi)
		for _, t := range facets {
			tri := w.fp.Tris[t]
			for k := range 3 {
				have[edge{tri[k], tri[(k+1)%3]}] = struct{}{}
			}
		}
		for _, t := range facets {
			tri := w.fp.Tris[t]
			for k := range 3 {
				a, b := tri[k], tri[(k+1)%3]
				if _, twin := have[edge{b, a}]; !twin {
					on[a], on[b] = true, true
				}
			}
		}
	}
	return on
}

// faceAt is the face of the held body with index fi, or nil.
func (w *work) faceAt(fi int) *topo.Face {
	for f, i := range w.faceIx {
		if i == fi {
			return f
		}
	}
	return nil
}
