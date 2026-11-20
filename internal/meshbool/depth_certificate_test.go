package meshbool

import (
	"math"
	"math/big"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// exactVec is an exact 3D vector.
type exactVec [3]*big.Rat

func exactVecOf(v r3.Vec) exactVec {
	return exactVec{new(big.Rat).SetFloat64(v.X), new(big.Rat).SetFloat64(v.Y), new(big.Rat).SetFloat64(v.Z)}
}

func (a exactVec) sub(b exactVec) exactVec {
	return exactVec{new(big.Rat).Sub(a[0], b[0]), new(big.Rat).Sub(a[1], b[1]), new(big.Rat).Sub(a[2], b[2])}
}

func (a exactVec) add(b exactVec) exactVec {
	return exactVec{new(big.Rat).Add(a[0], b[0]), new(big.Rat).Add(a[1], b[1]), new(big.Rat).Add(a[2], b[2])}
}

func (a exactVec) scale(s *big.Rat) exactVec {
	return exactVec{new(big.Rat).Mul(a[0], s), new(big.Rat).Mul(a[1], s), new(big.Rat).Mul(a[2], s)}
}

func (a exactVec) dot(b exactVec) *big.Rat {
	r := new(big.Rat).Mul(a[0], b[0])
	r.Add(r, new(big.Rat).Mul(a[1], b[1]))
	return r.Add(r, new(big.Rat).Mul(a[2], b[2]))
}

func (a exactVec) cross(b exactVec) exactVec {
	m := func(x, y, z, w *big.Rat) *big.Rat {
		return new(big.Rat).Sub(new(big.Rat).Mul(x, y), new(big.Rat).Mul(z, w))
	}
	return exactVec{m(a[1], b[2], a[2], b[1]), m(a[2], b[0], a[0], b[2]), m(a[0], b[1], a[1], b[0])}
}

// exactTriDist2 is the exact squared distance from p to the closed triangle
// t: the squared plane distance when the exact foot lies in t, otherwise the
// least squared distance to its three closed edges.
func exactTriDist2(p exactVec, t [3]exactVec) *big.Rat {
	n := t[1].sub(t[0]).cross(t[2].sub(t[0]))
	nn := n.dot(n)
	if nn.Sign() > 0 {
		nw := n.dot(p.sub(t[0]))
		foot := p.sub(n.scale(new(big.Rat).Quo(nw, nn)))
		inside := true
		for i := range 3 {
			if t[(i+1)%3].sub(t[i]).cross(foot.sub(t[i])).dot(n).Sign() < 0 {
				inside = false
				break
			}
		}
		if inside {
			return new(big.Rat).Quo(new(big.Rat).Mul(nw, nw), nn)
		}
	}
	var best *big.Rat
	for i := range 3 {
		q0, v := t[i], t[(i+1)%3].sub(t[i])
		s := new(big.Rat)
		if c := v.dot(v); c.Sign() > 0 {
			s.Quo(v.dot(p.sub(q0)), c)
			if s.Sign() < 0 {
				s.SetInt64(0)
			} else if s.Cmp(big.NewRat(1, 1)) > 0 {
				s.SetInt64(1)
			}
		}
		d := p.sub(q0.add(v.scale(s)))
		if d2 := d.dot(d); best == nil || d2.Cmp(best) < 0 {
			best = d2
		}
	}
	return best
}

// exactMeshDist2 is the exact squared distance from p to m's closed facets.
func exactMeshDist2(p r3.Vec, m *BoolMesh) *big.Rat {
	pr := exactVecOf(p)
	var best *big.Rat
	for i := range m.Tris {
		c := TriCorners(m, i)
		if d2 := exactTriDist2(pr, [3]exactVec{exactVecOf(c[0]), exactVecOf(c[1]), exactVecOf(c[2])}); best == nil || d2.Cmp(best) < 0 {
			best = d2
		}
	}
	return best
}

// floatAtLeastSqrt is the least float64 whose square is at least d2: an
// upper bound on the exact distance sqrt(d2).
func floatAtLeastSqrt(d2 *big.Rat) float64 {
	f := new(big.Float).SetPrec(256).SetRat(d2)
	s, _ := f.Sqrt(f).Float64()
	for new(big.Rat).Mul(ratOf(s), ratOf(s)).Cmp(d2) < 0 {
		s = math.Nextafter(s, math.Inf(1))
	}
	for s > 0 {
		lower := math.Nextafter(s, 0)
		if new(big.Rat).Mul(ratOf(lower), ratOf(lower)).Cmp(d2) < 0 {
			break
		}
		s = lower
	}
	return s
}

func ratOf(x float64) *big.Rat { return new(big.Rat).SetFloat64(x) }

// sliverTetrahedron is the closed tetrahedron A, B, C, E with coordinates near
// 100 mm and its face ABC split at D, the point a fraction s of the way from
// AB's midpoint toward C, so facet ABD is a sliver of relative width about s.
// s = 0 keeps ABC whole. Every facet faces outward. shift moves every vertex.
// It returns the mesh, the corners of the face the samples lie under (ABD, or
// ABC when s = 0) and the apex E across the solid from that face.
func sliverTetrahedron(t *testing.T, s float64, shift r3.Vec) (*BoolMesh, [3]r3.Vec, r3.Vec) {
	t.Helper()
	a := r3.Vec{X: 101.37, Y: 47.913, Z: -63.21}.Add(shift)
	b := r3.Vec{X: 52.11, Y: 88.402, Z: -17.77}.Add(shift)
	c := r3.Vec{X: 79.03, Y: 12.6, Z: -21.004}.Add(shift)
	e := r3.Vec{X: 61.5, Y: 41.25, Z: -78.8}.Add(shift)
	verts := []r3.Vec{a, b, c, e}
	faces := [][3]int{{0, 1, 2}, {0, 1, 3}, {1, 2, 3}, {2, 0, 3}}
	face := [3]r3.Vec{a, b, c}
	if s != 0 {
		mid := a.Add(b).Scale(0.5)
		d := mid.Add(c.Sub(mid).Scale(s))
		verts = append(verts, d)
		faces = [][3]int{{0, 1, 4}, {1, 2, 4}, {2, 0, 4}, {0, 1, 3}, {1, 2, 3}, {2, 0, 3}}
		face = [3]r3.Vec{a, b, d}
	}
	centre := a.Add(b).Add(c).Add(e).Scale(0.25)
	for i, f := range faces {
		p, q, r := verts[f[0]], verts[f[1]], verts[f[2]]
		if q.Sub(p).Cross(r.Sub(p)).Dot(p.Add(q).Add(r).Scale(1.0/3).Sub(centre)) < 0 {
			faces[i] = [3]int{f[0], f[2], f[1]}
		}
	}
	bm, err := PrepContext(t.Context(), verts, faces, make([]int, len(faces)))
	require.NoError(t, err)
	bm.SetVertexBounds(make([]float64, len(verts)))
	return bm, face, e
}

// TestCertifiedInteriorDepthBoundsExactDepth places float points just inside
// one face of a closed tetrahedron, at depths from 1e-9 to 1e-3, and checks the
// certified depth against the exact distance to the facets. The certified
// depth never exceeds the exact distance, and DeepWitnessAt never admits a
// point whose exact depth is at most the slack, with b padded by 1e-9 of the
// slack as the tangency gate pads it. Thin facets make the float facet normal
// point well away from the exact one, and every float distance errs by up to a
// few units in the last place of the coordinates whatever the depth. The
// "centred" case puts the sampled face's centroid at the origin, so the points
// lie far nearer the origin than the facets' corners do. The tetrahedron's
// centroid, admitted at half its exact depth, and the samples deeper than
// 1e-6, certified within 1e-5 of their exact depth, check that the
// certificate still admits.
func TestCertifiedInteriorDepthBoundsExactDepth(t *testing.T) {
	t.Parallel()
	cases := []struct {
		name    string
		s       float64
		centred bool
	}{
		{name: "plain"},
		{name: "sliver1e-4", s: 1e-4},
		{name: "sliver1e-7", s: 1e-7},
		{name: "centred", centred: true},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			var shift r3.Vec
			if tc.centred {
				_, face, _ := sliverTetrahedron(t, tc.s, r3.Vec{})
				shift = face[0].Add(face[1]).Add(face[2]).Scale(-1.0 / 3)
			}
			bm, face, apex := sliverTetrahedron(t, tc.s, shift)
			n := face[1].Sub(face[0]).Cross(face[2].Sub(face[0]))
			n = n.Scale(1 / n.Len())
			if n.Dot(apex.Sub(face[0])) < 0 {
				n = n.Scale(-1) // point n into the solid
			}
			// The tetrahedron's centroid is far inside it, so it is a witness
			// at half its exact depth.
			centroid := face[0].Add(face[1]).Add(face[2]).Add(apex).Scale(0.25)
			inner := floatAtLeastSqrt(exactMeshDist2(centroid, bm))
			deep, err := DeepWitnessAt(t.Context(), proof.XptOf(centroid), bm, inner/2)
			require.NoError(t, err)
			require.True(t, deep, "centroid at exact depth %.17g", inner)
			rng := rand.New(rand.NewPCG(7, math.Float64bits(tc.s)))
			const samples = 400
			tight := 0
			for k := range samples {
				u, v := rng.Float64(), rng.Float64()
				if tc.centred {
					// Near the centroid, within about 0.1 mm of the origin.
					u, v = 1.0/3+1e-3*(u-0.5), 1.0/3+1e-3*(v-0.5)
				} else if u+v > 1 {
					u, v = 1-u, 1-v
				}
				q := face[0].Add(face[1].Sub(face[0]).Scale(u)).Add(face[2].Sub(face[0]).Scale(v))
				p := q.Add(n.Scale(math.Pow(10, -9+6*rng.Float64())))
				xp := proof.XptOf(p)
				d2 := exactMeshDist2(p, bm)
				cert := CertifiedInteriorDepth(xp, bm)
				if cert > 0 {
					require.LessOrEqual(t, new(big.Rat).Mul(ratOf(cert), ratOf(cert)).Cmp(d2), 0,
						"sample %d: certified depth %.17g exceeds the exact depth %.17g", k, cert, floatAtLeastSqrt(d2))
				}
				slack := floatAtLeastSqrt(d2)
				deep, err := DeepWitnessAt(t.Context(), xp, bm, slack*(1+1e-9))
				require.NoError(t, err)
				require.False(t, deep, "sample %d: admitted at exact depth %.17g, no deeper than the slack", k, slack)
				if slack >= 1e-6 {
					// The certificate gives up at most a few ulps of the
					// coordinates, far below 1e-5 of a depth this large.
					require.Greater(t, cert, slack*(1-1e-5), "sample %d: certified depth far below the exact depth", k)
					tight++
				}
			}
			require.Positive(t, tight)
		})
	}
}

// depthSearchCase builds a facet of one shape at scale s and a float point
// near one of its edges: inside or outside the edge by a few ulps of s, and
// off the plane by up to about s·2^-51 or, one time in six, by up to s.
// Shapes: 0 a generic facet spanning the box, 1 a sliver with its third
// corner near the midpoint of the first edge, 2 a needle with its third corner
// near the first, 3 a small facet far from the origin, 4 a facet on a plane
// x + y = k whose normal has two tied components.
func depthSearchCase(rng *rand.Rand, s float64, shape int) (r3.Vec, [3]r3.Vec) {
	const u = 0x1p-53
	full := func(scale float64) r3.Vec {
		return r3.Vec{X: scale * (2*rng.Float64() - 1), Y: scale * (2*rng.Float64() - 1), Z: scale * (2*rng.Float64() - 1)}
	}
	var a, b, c r3.Vec
	switch shape {
	case 0:
		// Corners near the box's corners make the longest edges and the
		// largest coordinates, where an edge reading errs most.
		corner := func() r3.Vec {
			sign := func() float64 { return float64(1 - 2*rng.IntN(2)) }
			return r3.Vec{X: sign() * s * (1 - rng.Float64()/8), Y: sign() * s * (1 - rng.Float64()/8), Z: sign() * s * (1 - rng.Float64()/8)}
		}
		a, b, c = corner(), corner(), corner()
	case 1:
		a, b = full(s), full(s)
		mid := a.Add(b).Scale(0.5)
		c = mid.Add(full(s).Sub(mid).Scale(math.Pow(10, -4-12*rng.Float64())))
	case 2:
		a, b = full(s), full(s)
		c = a.Add(full(s).Scale(math.Pow(10, -3-12*rng.Float64())))
	case 3:
		o, sc := full(s), s*math.Pow(10, -1-8*rng.Float64())
		a, b, c = o.Add(full(sc)), o.Add(full(sc)), o.Add(full(sc))
	default:
		k := s / 4 * (2*rng.Float64() - 1)
		on := func() r3.Vec {
			x := s / 2 * (2*rng.Float64() - 1)
			return r3.Vec{X: x, Y: k - x, Z: s / 2 * (2*rng.Float64() - 1)}
		}
		a, b, c = on(), on(), on()
	}
	tri := [3]r3.Vec{a, b, c}
	rot := rng.IntN(3)
	tri = [3]r3.Vec{tri[rot], tri[(rot+1)%3], tri[(rot+2)%3]}
	nf := tri[1].Sub(tri[0]).Cross(tri[2].Sub(tri[0]))
	if l := nf.Len(); l > 0 {
		nf = nf.Scale(1 / l)
	}
	j := rng.IntN(3)
	e0, e1 := tri[j], tri[(j+1)%3]
	lam := rng.Float64()
	if rng.IntN(4) == 0 {
		lam = []float64{1e-9, 1 - 1e-9, 0.5, 1e-15}[rng.IntN(4)]
	}
	in := nf.Cross(e1.Sub(e0))
	if l := in.Len(); l > 0 {
		in = in.Scale(1 / l)
	}
	sigma := s * u * math.Pow(10, -1+3.5*rng.Float64())
	if rng.IntN(5) == 0 {
		sigma = -sigma
	}
	delta := s * u * math.Pow(10, -3+4*rng.Float64())
	if rng.IntN(6) == 0 {
		delta = s * math.Pow(10, -12+12*rng.Float64())
	}
	if rng.IntN(2) == 0 {
		delta = -delta
	}
	return e0.Add(e1.Sub(e0).Scale(lam)).Add(in.Scale(sigma)).Add(nf.Scale(delta)), tri
}

// nudgeULPs moves each coordinate of v by up to k float steps either way.
func nudgeULPs(rng *rand.Rand, v r3.Vec, k int) r3.Vec {
	step := func(x float64) float64 {
		for range rng.IntN(k + 1) {
			x = math.Nextafter(x, math.Inf(1-2*rng.IntN(2)))
		}
		return x
	}
	return r3.Vec{X: step(v.X), Y: step(v.Y), Z: step(v.Z)}
}

// TestDepthFacetDistanceWithinBound searches for float inputs on which
// depthFacetDistance over-reads the exact point-to-facet distance d: generic
// facets, slivers, needles, small facets far from the origin and facets with
// tied normal components, at scales from 2^-300 to 2^200, with the point a few
// ulps inside or outside an edge. From the worst of each shape and scale it
// climbs by moving the point and a corner by a few ulps. Every reading D stays
// within depthCertificate's proven error, D <= (1+12u)·d + 132u·M (u = 2^-53,
// M the largest coordinate magnitude), and the certificate of a one-facet mesh
// never exceeds d. Under the race detector it searches the 100 mm scale only.
func TestDepthFacetDistanceWithinBound(t *testing.T) {
	t.Parallel()
	const u = 0x1p-53
	// Generic facets reach the largest errors, so they get most of the cases.
	const samples, climbs, genericFactor = 300, 100, 7
	scales := []float64{1, 100, 1e6, 0x1p200, 0x1p-300}
	if raceDetector {
		// Each scale draws its own sequence, so this is the plain build's
		// 100 mm search unchanged.
		scales = []float64{100}
	}
	finite := func(v r3.Vec) bool {
		return !proofbound.IsNonFinite(v.X) && !proofbound.IsNonFinite(v.Y) && !proofbound.IsNonFinite(v.Z)
	}
	worstSeen := math.Inf(-1)
	for _, s := range scales {
		for shape := range 5 {
			rng := rand.New(rand.NewPCG(uint64(shape)+11, math.Float64bits(s)))
			// excess checks one case and returns (D − (1+12u)·d)/(u·M).
			excess := func(p r3.Vec, tri [3]r3.Vec) float64 {
				xt := [3]proof.Xpt{proof.XptOf(tri[0]), proof.XptOf(tri[1]), proof.XptOf(tri[2])}
				d := depthFacetDistance(p, tri, depthNormalOf(proof.XtriangleNormal(xt[0], xt[1], xt[2])))
				d2 := exactTriDist2(exactVecOf(p), [3]exactVec{exactVecOf(tri[0]), exactVecOf(tri[1]), exactVecOf(tri[2])})
				m := vertexReach([]r3.Vec{p, tri[0], tri[1], tri[2]})
				exact := new(big.Float).SetPrec(400).SetRat(d2)
				exact.Sqrt(exact)
				over := new(big.Float).SetPrec(400).Mul(exact, new(big.Float).SetPrec(400).SetFloat64(1+12*u))
				over.Sub(new(big.Float).SetPrec(400).SetFloat64(d), over)
				ratio, _ := over.Quo(over, new(big.Float).SetPrec(400).SetFloat64(m*u)).Float64()
				require.LessOrEqual(t, ratio, 132.0, "scale %g shape %d: reading %.17g over-reads the exact distance %.17g by %.3g u·M",
					s, shape, d, floatAtLeastSqrt(d2), ratio)
				one := &BoolMesh{Verts: tri[:], Tris: [][3]int{{0, 1, 2}}}
				if cert := depthCertificate(d, proof.XptOf(p), p, one); cert > 0 {
					require.LessOrEqual(t, new(big.Rat).Mul(ratOf(cert), ratOf(cert)).Cmp(d2), 0,
						"scale %g shape %d: certificate %.17g exceeds the exact distance %.17g", s, shape, cert, floatAtLeastSqrt(d2))
				}
				return ratio
			}
			worst := math.Inf(-1)
			var wp r3.Vec
			var wt [3]r3.Vec
			n := samples
			if shape == 0 {
				n *= genericFactor
			}
			for range n {
				p, tri := depthSearchCase(rng, s, shape)
				if !finite(p) || !finite(tri[0]) || !finite(tri[1]) || !finite(tri[2]) {
					continue
				}
				if r := excess(p, tri); r > worst {
					worst, wp, wt = r, p, tri
				}
			}
			for it := range climbs {
				k := 1 + it%8
				p, tri := nudgeULPs(rng, wp, k), wt
				if rng.IntN(3) == 0 {
					i := rng.IntN(3)
					tri[i] = nudgeULPs(rng, tri[i], k)
				}
				if r := excess(p, tri); r > worst {
					worst, wp, wt = r, p, tri
				}
			}
			worstSeen = max(worstSeen, worst)
		}
	}
	t.Logf("largest over-read: %.3f u·M of the 132 u·M proven", worstSeen)
}
