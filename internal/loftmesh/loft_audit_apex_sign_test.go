package loftmesh

import (
	"math"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// apexFixture is a mesh of independent shared-edge pairs. Pair p owns vertices
// 4p..4p+3 and triangles 2p and 2p+1: triangle 2p holds the edge {4p, 4p+1}
// and the apex 4p+2, triangle 2p+1 holds the same edge and the apex 4p+3, each
// in a random corner order so the shared edge lands on every corner pair.
type apexFixture struct {
	verts []r3.Vec
	tris  [][3]int
}

func (f *apexFixture) add(rng *rand.Rand, e0, e1, a, b r3.Vec) {
	base := len(f.verts)
	f.verts = append(f.verts, e0, e1, a, b)
	pi, pj := planePairPerms[rng.IntN(6)], planePairPerms[rng.IntN(6)]
	ti := [3]int{base, base + 1, base + 2}
	tj := [3]int{base, base + 1, base + 3}
	if rng.IntN(2) == 0 {
		tj[0], tj[1] = tj[1], tj[0]
	}
	f.tris = append(f.tris,
		[3]int{ti[pi[0]], ti[pi[1]], ti[pi[2]]},
		[3]int{tj[pj[0]], tj[pj[1]], tj[pj[2]]})
}

// apexPoints draws one pair's four points. kind picks the family.
func apexPoints(rng *rand.Rand, kind int) (e0, e1, a, b r3.Vec) {
	cube := func() r3.Vec {
		return r3.Vec{X: rng.Float64()*2 - 1, Y: rng.Float64()*2 - 1, Z: rng.Float64()*2 - 1}
	}
	integer := func() r3.Vec {
		return r3.Vec{X: float64(rng.IntN(9) - 4), Y: float64(rng.IntN(9) - 4), Z: float64(rng.IntN(9) - 4)}
	}
	// coplanarApex is t0 + s(t1-t0) + u(t2-t0) over integers, exact in floats.
	coplanarApex := func(t0, t1, t2 r3.Vec) r3.Vec {
		s, u := float64(rng.IntN(7)-3), float64(rng.IntN(7)-3)
		return t0.Add(t1.Sub(t0).Scale(s)).Add(t2.Sub(t0).Scale(u))
	}
	switch kind {
	case 0: // generic fold
		return cube(), cube(), cube(), cube()
	case 1: // exactly coplanar, integer coordinates
		e0, e1, a = integer(), integer(), integer()
		return e0, e1, a, coplanarApex(e0, e1, a)
	case 2: // one ulp off the plane at a large translation, so the filter cannot decide
		e0, e1, a = integer(), integer(), integer()
		b = coplanarApex(e0, e1, a)
		shift := r3.Vec{X: float64(rng.IntN(1e7)), Y: float64(rng.IntN(1e7)), Z: float64(rng.IntN(1e7))}
		e0, e1, a, b = e0.Add(shift), e1.Add(shift), a.Add(shift), b.Add(shift)
		switch rng.IntN(3) {
		case 0:
			b.X = math.Nextafter(b.X, math.Inf(1-2*rng.IntN(2)))
		case 1:
			b.Y = math.Nextafter(b.Y, math.Inf(1-2*rng.IntN(2)))
		default:
			b.Z = math.Nextafter(b.Z, math.Inf(1-2*rng.IntN(2)))
		}
		return e0, e1, a, b
	case 3: // fold around the edge at a random angle
		e0, e1 = cube(), cube()
		axis := e1.Sub(e0)
		perp := r3.Vec{X: -axis.Y, Y: axis.X, Z: 0}
		other := axis.Cross(perp)
		phi := rng.Float64() * math.Pi
		r := 0.2 + rng.Float64()
		a = e0.Add(perp.Scale(r))
		b = e0.Add(perp.Scale(r * math.Cos(phi))).Add(other.Scale(r * math.Sin(phi)))
		return e0, e1, a, b
	default: // an axis-aligned plane: zero columns decide the sign exactly
		e0, e1 = r3.Vec{X: rng.Float64(), Y: rng.Float64()}, r3.Vec{X: rng.Float64(), Y: rng.Float64()}
		a = r3.Vec{X: rng.Float64(), Y: rng.Float64()}
		return e0, e1, a, r3.Vec{X: rng.Float64(), Y: rng.Float64(), Z: float64(rng.IntN(2))}
	}
}

// apexFixtureMesh draws the whole fixture: scales 2^k for k in {-40, 0, 40} and
// 200 pairs scaled so every coordinate difference is subnormal.
func apexFixtureMesh(pairs int) *apexFixture {
	rng := rand.New(rand.NewPCG(53, 59))
	f := &apexFixture{}
	for p := range pairs {
		scale := math.Ldexp(1, [3]int{-40, 0, 40}[rng.IntN(3)])
		if p < 200 {
			scale = math.Ldexp(1, -1070)
		}
		for {
			e0, e1, a, b := apexPoints(rng, p%5)
			e0, e1, a, b = e0.Scale(scale), e1.Scale(scale), a.Scale(scale), b.Scale(scale)
			// The audit never reaches a collapsed triangle (S6 drops it first).
			probe := []r3.Vec{e0, e1, a, b}
			if TriangleCollapsed(probe, [3]int{0, 1, 2}) || TriangleCollapsed(probe, [3]int{0, 1, 3}) {
				continue
			}
			f.add(rng, e0, e1, a, b)
			break
		}
	}
	return f
}

// apexReferenceSign is the exact sign the pair function used before the float
// filter: na . (apex - edgeA) over the exact lifts.
func apexReferenceSign(d *LoftAuditData, tris [][3]int, i, j int) int {
	shared, _ := tessellation.SharedVertexIndices(tris[i], tris[j])
	apexIndex := tessellation.TriangleApexIndex(tris[j], shared[0], shared[1])
	return proof.XdotSign(d.Norms[i], proof.Xsub(d.Xverts[apexIndex], d.Xverts[shared[0]]))
}

func TestApexPlaneSignMatchesExact(t *testing.T) {
	t.Parallel()
	const pairs = 8000
	f := apexFixtureMesh(pairs)
	plain := NewLoftAuditData(f.verts, f.tris)
	exact := NewLoftAuditDataWithExact(f.verts, perturbedExact(f.verts), f.tris)
	decided, declined, perturbedMoved := 0, 0, 0
	signs := map[int]int{}
	for p := range pairs {
		i, j := 2*p, 2*p+1
		shared, count := tessellation.SharedVertexIndices(f.tris[i], f.tris[j])
		require.Equal(t, 2, count)
		apexIndex := tessellation.TriangleApexIndex(f.tris[j], shared[0], shared[1])
		want := apexReferenceSign(plain, f.tris, i, j)
		got := plain.apexPlaneSign(plain.corners(i), plain.corners(j), plain.xtri(i), plain.xtri(j),
			plain.Norms[i], plain.Xverts[apexIndex], plain.Xverts[shared[0]])
		require.Equal(t, want, got, "pair %d", p)
		signs[want]++

		ti := plain.corners(i)
		var apexCorner r3.Vec
		for k, v := range f.tris[j] {
			if v == apexIndex {
				apexCorner = plain.corners(j)[k]
			}
		}
		if _, ok := proof.OrientSignFloat(ti[0], ti[1], ti[2], apexCorner); ok {
			decided++
		} else {
			declined++
		}

		// Exact lifts that differ from the floats: the float answer is off.
		wantX := apexReferenceSign(exact, f.tris, i, j)
		gotX := exact.apexPlaneSign(exact.corners(i), exact.corners(j), exact.xtri(i), exact.xtri(j),
			exact.Norms[i], exact.Xverts[apexIndex], exact.Xverts[shared[0]])
		require.Equal(t, wantX, gotX, "exact-lift pair %d", p)
		if wantX != want {
			perturbedMoved++
		}
	}
	require.GreaterOrEqual(t, 2*decided, pairs, "the float filter decided %d of %d pairs", decided, pairs)
	require.GreaterOrEqual(t, 20*declined, pairs, "the float filter declined %d of %d pairs", declined, pairs)
	require.Positive(t, signs[0], "no coplanar pair")
	require.Positive(t, signs[1], "no positive pair")
	require.Positive(t, signs[-1], "no negative pair")
	require.Positive(t, perturbedMoved, "the perturbed lifts never changed a sign, so they prove nothing")
}

// TestApexPlaneSignKeepsPairOutcomes runs the shipped pair function against the
// exact-only reference on every shared-edge pair of the fixture, including the
// coplanar pairs that reach TriTriCoplanarSharedEdge and refuse.
func TestApexPlaneSignKeepsPairOutcomes(t *testing.T) {
	t.Parallel()
	f := apexFixtureMesh(3000)
	data := NewLoftAuditData(f.verts, f.tris)
	refused, edge := 0, 0
	for p := range len(f.tris) / 2 {
		i, j := 2*p, 2*p+1
		for _, sc := range []LoftAuditShortcuts{loftGenericShortcuts, {Sweep: true}} {
			got, gotErr := auditLoftPairDataSigned(data, f.tris, i, j, sc, nil, nil)
			want, wantErr := auditLoftPairDataSignedReference(data, f.tris, i, j, sc)
			require.Equal(t, want, got, "pair %d", p)
			if wantErr != nil {
				require.EqualError(t, gotErr, wantErr.Error(), "pair %d", p)
				var gotC, wantC *LoftContactError
				require.ErrorAs(t, gotErr, &gotC)
				require.ErrorAs(t, wantErr, &wantC)
				require.Equal(t, [2]int{wantC.I, wantC.J}, [2]int{gotC.I, gotC.J}, "pair %d", p)
				refused++
				continue
			}
			require.NoError(t, gotErr, "pair %d", p)
			if got == LoftPairEdgeCertificate {
				edge++
			}
		}
	}
	require.Positive(t, refused, "no coplanar refusal reached the pair function")
	require.Positive(t, edge, "no edge certificate")
}

// TestApexPlaneSignKeepsAuditWork compares the generic audit's error and work
// counts on helical sweeps with a replay of every candidate through the
// exact-only reference.
func TestApexPlaneSignKeepsAuditWork(t *testing.T) {
	t.Parallel()
	for name, profile := range planeProfiles() {
		t.Run(name, func(t *testing.T) {
			t.Parallel()
			perTurn, stations := 8, 12
			if len(profile) > 4 {
				perTurn, stations = 4, 6
			}
			for _, squeeze := range []float64{0.2, 0.6, 1.0} {
				verts, tris := planeHelixMesh(profile, 1, 0.85*squeeze, perTurn, stations)
				work, err := LoftCrossingAuditWork(proofbound.NewWorkBudget(t.Context()), verts, tris, loftGenericShortcuts)
				data := NewLoftAuditData(verts, tris)
				cands, cerr := LoftSweepCandidates(proofbound.NewWorkBudget(t.Context()), verts, tris)
				require.NoError(t, cerr)
				edgeCerts := 0
				for _, c := range cands {
					_, count := tessellation.SharedVertexIndices(tris[c[0]], tris[c[1]])
					if count != 2 {
						continue
					}
					got, gotErr := auditLoftPairDataSigned(data, tris, c[0], c[1], loftGenericShortcuts, nil, nil)
					want, wantErr := auditLoftPairDataSignedReference(data, tris, c[0], c[1], loftGenericShortcuts)
					require.Equal(t, want, got, "%s pair %v", name, c)
					require.Equal(t, wantErr == nil, gotErr == nil, "%s pair %v", name, c)
					if got == LoftPairEdgeCertificate {
						edgeCerts++
					}
				}
				if err == nil {
					require.Equal(t, edgeCerts, work.EdgeCerts, "%s squeeze %v", name, squeeze)
				}
				require.Positive(t, edgeCerts, "%s squeeze %v", name, squeeze)
			}
		})
	}
}

// TestApexPlaneSignCallerOrderDiffersFromTable passes AuditLoftPairData a
// caller triple that lists triangle j's vertices in another order than the
// table was built with. The table owns the order, so every path refuses it.
func TestApexPlaneSignCallerOrderDiffersFromTable(t *testing.T) {
	t.Parallel()
	verts := []r3.Vec{{}, {X: 1}, {Y: 1}, {Z: 1}}
	table := [][3]int{{0, 1, 2}, {0, 1, 3}}
	data := NewLoftAuditData(verts, table)
	caller := [][3]int{{0, 1, 2}, {3, 0, 1}}
	for _, sc := range []LoftAuditShortcuts{{Certificates: true}, {Certificates: true, Sweep: true}} {
		_, err := AuditLoftPairData(data, caller, 0, 1, sc)
		require.ErrorIs(t, err, ErrLoftAuditOrder)
		got, err := AuditLoftPairData(data, table, 0, 1, sc)
		require.NoError(t, err)
		require.Equal(t, LoftPairEdgeCertificate, got)
	}
}

// permuteTriples returns tris with every triple's corners in a random order.
func permuteTriples(rng *rand.Rand, tris [][3]int) [][3]int {
	out := make([][3]int, len(tris))
	for n, tri := range tris {
		p := planePairPerms[rng.IntN(6)]
		out[n] = [3]int{tri[p[0]], tri[p[1]], tri[p[2]]}
	}
	return out
}

// TestApexPlaneSignKeepsOutcomesUnderCallerPermutation audits the fixture's
// shared-edge pairs with every triple's corner order permuted, the table built
// from the permuted order, and compares each outcome and error with the
// exact-only reference.
func TestApexPlaneSignKeepsOutcomesUnderCallerPermutation(t *testing.T) {
	t.Parallel()
	f := apexFixtureMesh(3000)
	rng := rand.New(rand.NewPCG(61, 67))
	refused, edge := 0, 0
	for round := range 2 {
		caller := permuteTriples(rng, f.tris)
		data := NewLoftAuditData(f.verts, caller)
		for p := range len(f.tris) / 2 {
			i, j := 2*p, 2*p+1
			got, gotErr := auditLoftPairDataSigned(data, caller, i, j, loftGenericShortcuts, nil, nil)
			want, wantErr := auditLoftPairDataSignedReference(data, caller, i, j, loftGenericShortcuts)
			require.Equal(t, want, got, "round %d pair %d", round, p)
			if wantErr != nil {
				require.EqualError(t, gotErr, wantErr.Error(), "round %d pair %d", round, p)
				refused++
				continue
			}
			require.NoError(t, gotErr, "round %d pair %d", round, p)
			if got == LoftPairEdgeCertificate {
				edge++
			}
		}
	}
	require.Positive(t, refused)
	require.Positive(t, edge)
}

// TestApexPlaneSignDeclinesForeignPoints calls apexPlaneSign with an edge
// endpoint that is no corner of triangle i's table entry, and with points that
// carry no lift. Neither may be decided in floats: the first must equal the
// exact dot product, the second must reach it (and panic on the missing lift).
func TestApexPlaneSignDeclinesForeignPoints(t *testing.T) {
	t.Parallel()
	verts := []r3.Vec{{}, {X: 1}, {Y: 1}, {Z: 1}}
	data := NewLoftAuditData(verts, [][3]int{{0, 1, 2}, {0, 1, 3}})
	ti, tj, xti, xtj := data.corners(0), data.corners(1), data.xtri(0), data.xtri(1)
	// Vertex 3 is off triangle 0's plane, but as edgeA it makes apex - edgeA
	// zero, so the exact sign is 0 while the float orientation of vertex 3 is not.
	foreign := data.Xverts[3]
	want := proof.XdotSign(data.Norms[0], proof.Xsub(data.Xverts[3], foreign))
	require.Zero(t, want)
	require.Equal(t, want, data.apexPlaneSign(ti, tj, xti, xtj, data.Norms[0], data.Xverts[3], foreign))
	s, ok := proof.OrientSignFloat(ti[0], ti[1], ti[2], verts[3])
	require.True(t, ok)
	require.NotZero(t, s, "the control float sign is zero, so the check proves nothing")

	var none [3]proof.Xpt
	require.Panics(t, func() {
		data.apexPlaneSign(ti, tj, none, none, data.Norms[0], proof.Xpt{}, proof.Xpt{})
	})
}
