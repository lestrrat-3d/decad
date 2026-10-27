package meshbool

import (
	"fmt"
	"math/big"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func contactMembershipFixture(b [3]r3.Vec) ([3]r3.Vec, [3]r3.Vec, [3]proof.Xpt, [3]proof.Xpt, proof.Xpt, proof.Xpt) {
	a := [3]r3.Vec{{}, {X: 2}, {Y: 2}}
	xa := [3]proof.Xpt{proof.XptOf(a[0]), proof.XptOf(a[1]), proof.XptOf(a[2])}
	xb := [3]proof.Xpt{proof.XptOf(b[0]), proof.XptOf(b[1]), proof.XptOf(b[2])}
	na := proof.Xcross(proof.Xsub(xa[1], xa[0]), proof.Xsub(xa[2], xa[0]))
	nb := proof.Xcross(proof.Xsub(xb[1], xb[0]), proof.Xsub(xb[2], xb[0]))
	return a, b, xa, xb, na, nb
}

// This keeps the pre-cache metadata sequence as the differential reference.
func classifyContactReference(ta, tb [3]r3.Vec, xta, xtb [3]proof.Xpt, na, nb proof.Xpt,
	pa, pb *[3]Xp2, sa, sb *[3]int, useFilter bool, fna, fnb r3.Vec, normalsPrepared bool,
) (TriContact, error) {
	span, crossing, err := triTriSpan(ta, tb, xta, xtb, na, nb, pa, pb, sa, sb, useFilter, fna, fnb, normalsPrepared)
	out := TriContact{EdgeA: -1, EdgeB: -1}
	if err != nil {
		return out, err
	}
	out.Kind, out.P0, out.P1 = span.Kind, span.P0, span.P1
	if !crossing {
		return out, nil
	}
	out.Sin2 = SinSquared(na, nb)
	out.P0OnA = OnTriBoundary(out.P0, xta, na)
	out.P0OnB = OnTriBoundary(out.P0, xtb, nb)
	if out.Kind == ContactSegment {
		out.P1OnA = OnTriBoundary(out.P1, xta, na)
		out.P1OnB = OnTriBoundary(out.P1, xtb, nb)
		out.EdgeA = SegAlongEdge(out.P0, out.P1, xta, na)
		out.EdgeB = SegAlongEdge(out.P0, out.P1, xtb, nb)
	} else {
		out.P1OnA, out.P1OnB = out.P0OnA, out.P0OnB
	}
	return out, nil
}

func TestTriTriClassifyBoundaryMembershipMatchesReference(t *testing.T) {
	t.Parallel()
	cases := map[string]struct {
		b    [3]r3.Vec
		kind ContactKind
	}{
		"interior segment": {
			b: [3]r3.Vec{{X: 0.5, Y: -0.5, Z: -1}, {X: 0.5, Y: 1.5, Z: 1},
				{X: 0.5, Y: 1.5, Z: -1}},
			kind: ContactSegment,
		},
		"edge segment": {
			b:    [3]r3.Vec{{Y: 0, Z: -1}, {X: 2, Y: 0, Z: -1}, {X: 1, Y: 0, Z: 1}},
			kind: ContactSegment,
		},
		"vertex edge segment": {
			b:    [3]r3.Vec{{}, {X: 1, Z: 1}, {X: 1, Z: -1}},
			kind: ContactSegment,
		},
		"vertex point": {
			b:    [3]r3.Vec{{}, {X: -1, Z: 1}, {Y: -1, Z: -1}},
			kind: ContactPoint,
		},
		"point inside face": {
			b: [3]r3.Vec{{X: 0.5, Y: 0.5}, {X: 0.5, Y: 0.75, Z: 1},
				{X: 0.5, Y: 0.25, Z: 1}},
			kind: ContactPoint,
		},
		"point at opposed interval ends": {
			b:    [3]r3.Vec{{X: 1}, {X: 1, Y: -1, Z: 1}, {X: 1, Y: -1, Z: -1}},
			kind: ContactPoint,
		},
		"equal interval ends": {
			b:    [3]r3.Vec{{X: 0.5}, {X: 0.5, Y: 1.5}, {X: 0.5, Y: 0.75, Z: 1}},
			kind: ContactSegment,
		},
		"both in-plane edges": {
			b:    [3]r3.Vec{{}, {X: 2}, {Z: 1}},
			kind: ContactSegment,
		},
		"coplanar point": {
			b:    [3]r3.Vec{{X: 2}, {X: 3}, {X: 2, Y: -1}},
			kind: ContactPoint,
		},
		"coplanar region": {
			b:    [3]r3.Vec{{X: 0.5, Y: 0.5}, {X: 1.5, Y: 0.5}, {X: 0.5, Y: 1.5}},
			kind: ContactRegion,
		},
		"disjoint": {
			b:    [3]r3.Vec{{X: 4, Z: -1}, {X: 4, Z: 1}, {X: 4, Y: 2, Z: 1}},
			kind: ContactNone,
		},
		"coplanar disjoint": {
			b:    [3]r3.Vec{{X: 4}, {X: 5}, {X: 4, Y: 1}},
			kind: ContactNone,
		},
	}
	for name, tc := range cases {
		t.Run(name, func(t *testing.T) {
			for _, scaled := range []bool{false, true} {
				for _, swapped := range []bool{false, true} {
					for permutation := range 6 {
						a, b, xa, xb, _, _ := contactMembershipFixture(tc.b)
						if scaled {
							for i := range 3 {
								xa[i] = scaleContactPoint(xa[i], 1+80*i)
								xb[i] = scaleContactPoint(xb[i], 1+72*i)
							}
						}
						if swapped {
							a, b = b, a
							xa, xb = xb, xa
						}
						originalA, originalXA := a, xa
						for i := range 3 {
							j := (i + permutation%3) % 3
							if permutation >= 3 {
								j = (permutation%3 - i + 3) % 3
							}
							a[i], xa[i] = originalA[j], originalXA[j]
						}
						na := proof.Xcross(proof.Xsub(xa[1], xa[0]), proof.Xsub(xa[2], xa[0]))
						nb := proof.Xcross(proof.Xsub(xb[1], xb[0]), proof.Xsub(xb[2], xb[0]))
						var sa, sb [3]int
						for i := range 3 {
							sa[i] = proof.OrientSignExact(xb[0], xb[1], xb[2], xa[i])
							sb[i] = proof.OrientSignExact(xa[0], xa[1], xa[2], xb[i])
						}
						u, v := ProjAxes(na)
						var pa, pb [3]Xp2
						for i := range 3 {
							pa[i] = NewXP2FromXpt(xa[i], u, v)
							pb[i] = NewXP2FromXpt(xb[i], u, v)
						}
						for mode := range 4 {
							var ppa, ppb *[3]Xp2
							var psa, psb *[3]int
							if mode >= 1 {
								psa, psb = &sa, &sb
							}
							if mode >= 2 {
								ppa, ppb = &pa, &pb
							}
							prepared := mode == 3
							args := []any{name, scaled, swapped, permutation, mode}
							want, werr := classifyContactReference(a, b, xa, xb, na, nb, ppa, ppb,
								psa, psb, !scaled, na.Vec(), nb.Vec(), prepared)
							got, gerr := TriTriClassifyCore(a, b, xa, xb, na, nb, ppa, ppb,
								psa, psb, !scaled, na.Vec(), nb.Vec(), prepared)
							require.Equal(t, werr, gerr, "%v", args)
							require.Equal(t, tc.kind, got.Kind, "%v", args)
							require.Equal(t, want, got, "%v", args)
						}
					}
				}
			}
		})
	}
}

func scaleContactPoint(p proof.Xpt, shift int) proof.Xpt {
	scale := new(big.Int).Lsh(big.NewInt(1), uint(shift))
	return proof.Xpt{
		X: new(big.Int).Mul(p.X, scale), Y: new(big.Int).Mul(p.Y, scale),
		Z: new(big.Int).Mul(p.Z, scale), W: new(big.Int).Mul(p.W, scale),
	}
}

// TestContactBoundaryRationalCoordinatesMatchEdgePredicates uses exact points
// that their rounded float coordinates cannot represent. Plane-side signs
// come from the exact points, and the independent edge predicates check every
// boundary flag and edge index after the interval overlap.
func TestContactBoundaryRationalCoordinatesMatchEdgePredicates(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(311, 79))
	var stats contactCacheStats
	checked := 0
	for sample := range 400 {
		var a, b [3]r3.Vec
		var xa, xb [3]proof.Xpt
		for _, vertices := range []*[3]proof.Xpt{&xa, &xb} {
			for k := range 3 {
				// Different odd weights also prevent a shared-denominator
				// assumption from deciding a boundary comparison.
				vertices[k] = proof.Xpt{
					X: big.NewInt(int64(rng.IntN(17) - 8)),
					Y: big.NewInt(int64(rng.IntN(17) - 8)),
					Z: big.NewInt(int64(rng.IntN(17) - 8)),
					W: big.NewInt(int64(3 + 2*rng.IntN(4))),
				}
			}
		}
		na := proof.Xcross(proof.Xsub(xa[1], xa[0]), proof.Xsub(xa[2], xa[0]))
		nb := proof.Xcross(proof.Xsub(xb[1], xb[0]), proof.Xsub(xb[2], xb[0]))
		if zeroXpt(na) || zeroXpt(nb) {
			continue
		}
		var sa, sb [3]int
		for k := range 3 {
			a[k], b[k] = xa[k].Vec(), xb[k].Vec()
			sa[k] = proof.OrientSignExact(xb[0], xb[1], xb[2], xa[k])
			sb[k] = proof.OrientSignExact(xa[0], xa[1], xa[2], xb[k])
		}
		want, sin2, wantErr := legacyClassify(a, b, xa, xb, na, nb, nil, nil, &sa, &sb,
			false, r3.Vec{}, r3.Vec{}, false)
		got, gotErr := TriTriClassifyCore(a, b, xa, xb, na, nb, nil, nil, &sa, &sb,
			false, r3.Vec{}, r3.Vec{}, false)
		requireSameContact(t, &stats, want, sin2, wantErr, got, gotErr, fmt.Sprintf("sample %d", sample))
		checked++
	}
	require.Greater(t, checked, 390)
	require.Positive(t, stats.kinds[ContactNone])
	require.Positive(t, stats.kinds[ContactSegment])
}

func BenchmarkTriTriClassifyMembership(b *testing.B) {
	cases := map[string][3]r3.Vec{
		"interior": {{X: 0.5, Y: -0.5, Z: -1}, {X: 0.5, Y: 1.5, Z: 1}, {X: 0.5, Y: 1.5, Z: -1}},
		"edge":     {{Y: 0, Z: -1}, {X: 2, Y: 0, Z: -1}, {X: 1, Y: 0, Z: 1}},
	}
	for name, tri := range cases {
		b.Run(name, func(b *testing.B) {
			a, c, xa, xc, na, nc := contactMembershipFixture(tri)
			b.ReportAllocs()
			for b.Loop() {
				_, err := TriTriClassify(a, c, xa, xc, na, nc)
				if err != nil {
					b.Fatal(err)
				}
			}
		})
	}
}
