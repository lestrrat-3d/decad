package loftmesh

import (
	"errors"
	"math"
	"math/rand/v2"
	"sync/atomic"
	"testing"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// planePairPerms are the six orders a triangle's three vertex indices can take,
// so the shared vertex lands on every corner position.
var planePairPerms = [6][3]int{{0, 1, 2}, {0, 2, 1}, {1, 0, 2}, {1, 2, 0}, {2, 0, 1}, {2, 1, 0}}

// planePairTris is the triangle list of a two-triangle fixture: triangle 0 has
// the vertices {v, a1, a2} and triangle 1 {v, b1, b2} (v is vertex 0), each in
// the corner order perm0 and perm1 pick.
func planePairTris(perm0, perm1 int) [][3]int {
	a, b := [3]int{0, 1, 2}, [3]int{0, 3, 4}
	p, q := planePairPerms[perm0], planePairPerms[perm1]
	return [][3]int{{a[p[0]], a[p[1]], a[p[2]]}, {b[q[0]], b[q[1]], b[q[2]]}}
}

// ulp moves x one ulp up or down.
func ulp(x float64, up bool) float64 {
	if up {
		return math.Nextafter(x, math.Inf(1))
	}
	return math.Nextafter(x, math.Inf(-1))
}

// planePairPoints draws the five points (v, a1, a2, b1, b2) of one shared-vertex
// pair in the unit frame around v = 0. kind selects the family.
func planePairPoints(rng *rand.Rand, kind int) [5]r3.Vec {
	cube := func() r3.Vec {
		return r3.Vec{X: rng.Float64()*2 - 1, Y: rng.Float64()*2 - 1, Z: rng.Float64()*2 - 1}
	}
	polar := func(lo, hi float64) r3.Vec {
		a := lo + rng.Float64()*(hi-lo)
		r := 0.2 + rng.Float64()
		return r3.Vec{X: r * math.Cos(a), Y: r * math.Sin(a)}
	}
	switch kind {
	case 0: // generic corners: the pair separates or crosses at random
		return [5]r3.Vec{{}, cube(), cube(), cube(), cube()}
	case 1: // flat fan; the b sector overlaps the a sector half of the time
		lo := math.Pi
		if rng.IntN(2) == 0 {
			lo = math.Pi / 2
		}
		return [5]r3.Vec{{}, polar(0, math.Pi), polar(0, math.Pi), polar(lo, lo+math.Pi), polar(lo, lo+math.Pi)}
	case 2: // fold: b lies in a half plane at a random angle to the a half plane
		phi := rng.Float64() * math.Pi
		a := func() r3.Vec {
			return r3.Vec{X: rng.Float64()*2 - 1, Y: -(0.1 + rng.Float64())}
		}
		b := func() r3.Vec {
			r := 0.1 + rng.Float64()
			return r3.Vec{X: rng.Float64()*2 - 1, Y: r * math.Cos(phi), Z: r * math.Sin(phi)}
		}
		return [5]r3.Vec{{}, a(), a(), b(), b()}
	case 3: // saddle: bent fan z = c x y
		c := 0.5 + rng.Float64()*2
		sad := func() r3.Vec {
			x, y := rng.Float64()*2-1, rng.Float64()*2-1
			return r3.Vec{X: x, Y: y, Z: c * x * y}
		}
		return [5]r3.Vec{{}, sad(), sad(), sad(), sad()}
	case 4: // integer coplanar pair, often overlapping
		in := func() r3.Vec { return r3.Vec{X: float64(rng.IntN(9) - 4), Y: float64(rng.IntN(9) - 4)} }
		return [5]r3.Vec{{}, in(), in(), in(), in()}
	case 5: // b crosses triangle a through an interior point away from v
		a1, a2 := cube(), cube()
		m := a1.Scale(0.3).Add(a2.Scale(0.3))
		w := cube()
		if w.Z == 0 {
			w.Z = 0.5
		}
		// The crossing is real only when a is not edge-on to w; leave that to the draw.
		return [5]r3.Vec{{}, a1, a2, m.Add(w), m.Sub(w)}
	case 6: // b2 within one ulp of the plane of triangle a
		a1, a2 := cube(), cube()
		t, u := 0.2+rng.Float64(), 0.2+rng.Float64()
		b2 := a1.Scale(t).Add(a2.Scale(u))
		axis := rng.IntN(3)
		up := rng.IntN(2) == 0
		switch axis {
		case 0:
			b2.X = ulp(b2.X, up)
		case 1:
			b2.Y = ulp(b2.Y, up)
		default:
			b2.Z = ulp(b2.Z, up)
		}
		return [5]r3.Vec{{}, a1, a2, cube(), b2}
	default: // kind 7: b1 repeats a1's coordinates at another index
		a1 := cube()
		return [5]r3.Vec{{}, a1, cube(), a1, cube()}
	}
}

// subnormalPairPoints draws a pair whose coordinate differences are zero,
// subnormal or ordinary, the territory where a float filter without a range
// guard returns a wrong certain sign.
func subnormalPairPoints(rng *rand.Rand) [5]r3.Vec {
	coord := func() float64 {
		switch rng.IntN(5) {
		case 0:
			return 0
		case 1:
			return float64(rng.IntN(60)-30) * math.SmallestNonzeroFloat64
		case 2:
			return 0.25 * float64(rng.IntN(9)-4)
		case 3:
			return float64(rng.IntN(9)-4) * 0x1p-1000
		default:
			return rng.Float64()*2 - 1
		}
	}
	pt := func() r3.Vec { return r3.Vec{X: coord(), Y: coord(), Z: coord()} }
	return [5]r3.Vec{pt(), pt(), pt(), pt(), pt()}
}

// planePairResult is what the reference path says about one pair and whether
// certificate D fired on it.
type planePairResult struct {
	fired, admitted, usable bool
}

// checkPlanePair builds the two-triangle mesh and, when the separating plane
// fires, requires the zero-shortcut reference to admit the pair and the exact
// classification to report a single contact point at the shared vertex.
func checkPlanePair(t *testing.T, pts [5]r3.Vec, perm0, perm1 int, label string) planePairResult {
	t.Helper()
	verts := pts[:]
	tris := planePairTris(perm0, perm1)
	for _, tri := range tris {
		if TriangleCollapsed(verts, tri) {
			return planePairResult{}
		}
	}
	data := NewLoftAuditData(verts, tris)
	_, err := AuditLoftPairData(data, tris, 0, 1, LoftAuditShortcuts{})
	fired := data.separatingPlaneCertificate(tris[0], tris[1], 0, 1, 0)
	if fired {
		require.NoError(t, err, "%s: the separating plane fired on a pair the reference refuses: %v", label, pts)
		contact, cerr := meshbool.TriTriClassify(data.corners(0), data.corners(1), data.xtri(0), data.xtri(1),
			data.Norms[0], data.Norms[1])
		require.NoError(t, cerr, label)
		require.Equal(t, meshbool.ContactPoint, contact.Kind, "%s: %v", label, pts)
		require.Equal(t, data.Xverts[0].Key(), contact.P0.Key(), "%s: contact point is not the shared vertex", label)
		// The certificate must also agree with the audit as shipped.
		out, gerr := auditLoftPairDataSigned(data, tris, 0, 1, loftGenericShortcuts, nil, nil)
		require.NoError(t, gerr, label)
		require.Equal(t, LoftPairVertexCertificate, out, label)
	}
	return planePairResult{fired: fired, admitted: err == nil, usable: true}
}

func TestSeparatingPlaneCertificateImpliesPointContact(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(41, 43))
	exponents := []int{-40, -20, 0, 20, 40}
	admitted, fired, total := 0, 0, 0
	for n := range 6000 {
		pts := planePairPoints(rng, n%8)
		scale := math.Ldexp(1, exponents[rng.IntN(len(exponents))])
		var shift r3.Vec
		if rng.IntN(2) == 0 {
			shift = r3.Vec{X: (rng.Float64()*2 - 1) * 1e7, Y: (rng.Float64()*2 - 1) * 1e7,
				Z: (rng.Float64()*2 - 1) * 1e7}
		}
		for k := range pts {
			pts[k] = pts[k].Scale(scale).Add(shift)
		}
		res := checkPlanePair(t, pts, rng.IntN(6), rng.IntN(6), "family")
		if !res.usable {
			continue
		}
		total++
		if res.admitted {
			admitted++
			if res.fired {
				fired++
			}
		}
	}
	for range 200 {
		res := checkPlanePair(t, subnormalPairPoints(rng), rng.IntN(6), rng.IntN(6), "subnormal")
		if res.usable && res.admitted {
			admitted++
			if res.fired {
				fired++
			}
		}
	}
	t.Logf("%d usable pairs, %d admitted, %d decided by the separating plane", total, admitted, fired)
	require.Greater(t, total, 4500)
	require.Positive(t, fired, "the separating plane never fired")
	require.GreaterOrEqual(t, 10*fired, 3*admitted,
		"the separating plane fired on %d of %d admitted pairs", fired, admitted)
}

// planeHelixMesh is a float helical sweep of profile (offsets in the meridian
// plane) around the y axis: the cells split [b0,b1,t1], [b0,t1,t0] as
// coilshell.Build splits them, and each end gets a fan cap. coilshell imports
// loftmesh, so the test builds its own.
func planeHelixMesh(profile [][2]float64, radius, pitch float64, perTurn, stations int) ([]r3.Vec, [][3]int) {
	m := len(profile)
	var verts []r3.Vec
	for k := range stations + 1 {
		theta := 2 * math.Pi * float64(k) / float64(perTurn)
		lift := pitch * float64(k) / float64(perTurn)
		for _, p := range profile {
			r := radius + p[0]
			verts = append(verts, r3.Vec{X: r * math.Cos(theta), Y: p[1] + lift, Z: r * math.Sin(theta)})
		}
	}
	at := func(k, e int) int { return k*m + e%m }
	var tris [][3]int
	for k := range stations {
		for e := range m {
			b0, b1, t0, t1 := at(k, e), at(k, e+1), at(k+1, e), at(k+1, e+1)
			tris = append(tris, [3]int{b0, b1, t1}, [3]int{b0, t1, t0})
		}
	}
	for _, k := range []int{0, stations} {
		var c r3.Vec
		for e := range m {
			c = c.Add(verts[at(k, e)])
		}
		verts = append(verts, c.Scale(1/float64(m)))
		center := len(verts) - 1
		for e := range m {
			if k == 0 {
				tris = append(tris, [3]int{center, at(k, e+1), at(k, e)})
				continue
			}
			tris = append(tris, [3]int{center, at(k, e), at(k, e+1)})
		}
	}
	return verts, tris
}

func planeProfiles() map[string][][2]float64 {
	circle := make([][2]float64, 16)
	for i := range circle {
		a := 2 * math.Pi * float64(i) / float64(len(circle))
		circle[i] = [2]float64{0.15 * math.Cos(a), 0.15 * math.Sin(a)}
	}
	return map[string][][2]float64{
		"thread": {{-0.15, -0.15}, {0.15, -0.05}, {-0.1, 0.17}},
		"square": {{-0.15, -0.15}, {0.15, -0.15}, {0.15, 0.15}, {-0.15, 0.15}},
		"round":  circle,
	}
}

// planeWorkReplay tallies the shared-entity pairs of the generic candidate list
// replayed one pair at a time: edge and vertex certificates by the shipped
// pair function, vertex certificates without certificate D, and the pairs D
// moved from classification to a vertex certificate.
type planeWorkReplay struct {
	edge, vertexOn, vertexOff, fired int
}

// auditLoftPairDataSignedWithoutPlane is auditLoftPairDataSigned as it was
// before certificate D: the fan table and the float vertex certificate, then
// the reference path.
func auditLoftPairDataSignedWithoutPlane(data *LoftAuditData, tris [][3]int, i, j int,
	shortcuts LoftAuditShortcuts) (LoftPairOutcome, error) {
	shared, count := tessellation.SharedVertexIndices(tris[i], tris[j])
	if shortcuts.Certificates && count == 1 && data.fans != nil {
		if v, ok := data.fans.vertexCertificate(i, j, shared[0]); ok && v {
			return LoftPairVertexCertificate, nil
		}
	}
	if shortcuts.Certificates && count == 1 && data.vertexCertificateFloat(tris, i, j, shared[0]) {
		return LoftPairVertexCertificate, nil
	}
	return auditLoftPairDataSignedReference(data, tris, i, j, shortcuts)
}

// checkPlaneAuditMatchesReference runs the generic audit, the certificate-free
// reference audit and a pair-by-pair replay of the shared-entity pairs with
// and without D. It returns the number of pairs D decided and the audit error.
func checkPlaneAuditMatchesReference(t *testing.T, verts []r3.Vec, tris [][3]int, label string) (int, error) {
	t.Helper()
	ctx := t.Context()
	work, err := LoftCrossingAuditWork(proofbound.NewWorkBudget(ctx), verts, tris, loftGenericShortcuts)
	refWork, refErr := LoftCrossingAuditWork(proofbound.NewWorkBudget(ctx), verts, tris, LoftAuditShortcuts{Sweep: true})
	if refErr != nil {
		require.EqualError(t, err, refErr.Error(), label)
		var got, want *LoftContactError
		if errors.As(refErr, &want) {
			require.ErrorAs(t, err, &got, label)
			require.Equal(t, [2]int{want.I, want.J}, [2]int{got.I, got.J}, label)
		}
	} else {
		require.NoError(t, err, label)
		require.Equal(t, refWork.Candidates, work.Candidates, label)
		require.Equal(t, refWork.Scanned, work.Scanned, label)
	}
	for _, tri := range tris {
		if TriangleCollapsed(verts, tri) {
			return 0, err
		}
	}
	cands, cerr := LoftSweepCandidates(proofbound.NewWorkBudget(ctx), verts, tris)
	require.NoError(t, cerr, label)
	data := NewLoftAuditData(verts, tris)
	var replay planeWorkReplay
	for _, c := range cands {
		i, j := c[0], c[1]
		shared, count := tessellation.SharedVertexIndices(tris[i], tris[j])
		if count == 0 {
			continue
		}
		on, onErr := auditLoftPairDataSigned(data, tris, i, j, loftGenericShortcuts, nil, nil)
		off, offErr := auditLoftPairDataSignedWithoutPlane(data, tris, i, j, loftGenericShortcuts)
		if offErr != nil {
			require.EqualError(t, onErr, offErr.Error(), "%s pair %d,%d", label, i, j)
			require.Equal(t, off, on, "%s pair %d,%d", label, i, j)
		} else {
			require.NoError(t, onErr, "%s pair %d,%d", label, i, j)
		}
		switch {
		case on == off:
		case off == LoftPairClassified && on == LoftPairVertexCertificate && count == 1 && offErr == nil &&
			data.separatingPlaneCertificate(tris[i], tris[j], i, j, shared[0]):
			replay.fired++
		default:
			require.Failf(t, "outcome moved", "%s pair %d,%d: %v became %v", label, i, j, off, on)
		}
		switch on {
		case LoftPairEdgeCertificate:
			replay.edge++
		case LoftPairVertexCertificate:
			replay.vertexOn++
		case LoftPairClassified:
		}
		if off == LoftPairVertexCertificate {
			replay.vertexOff++
		}
	}
	if err == nil {
		require.Equal(t, len(cands), work.Candidates, label)
		require.Equal(t, replay.edge, work.EdgeCerts, label)
		require.Equal(t, replay.vertexOn, work.VertexCerts, label)
		require.Equal(t, replay.vertexOff+replay.fired, work.VertexCerts, label)
		require.Equal(t, work.Candidates, work.Skips+work.EdgeCerts+work.VertexCerts+work.Classifications, label)
	}
	return replay.fired, err
}

func TestSeparatingPlaneAuditMatchesReference(t *testing.T) {
	t.Parallel()
	var fired atomic.Int64
	t.Run("profiles", func(t *testing.T) {
		for name, profile := range planeProfiles() {
			t.Run(name, func(t *testing.T) {
				t.Parallel()
				fired.Add(int64(checkPlaneProfile(t, name, profile)))
			})
		}
	})
	require.Positive(t, fired.Load(), "the separating plane never decided a pair")
}

// checkPlaneProfile runs the audit comparisons over helical sweeps of one
// profile and returns how many pairs the separating plane decided.
func checkPlaneProfile(t *testing.T, name string, profile [][2]float64) int {
	t.Helper()
	rng := rand.New(rand.NewPCG(7, uint64(len(profile))))
	fired, refused, admitted := 0, 0, 0
	{
		perTurn, stations := 8, 12
		if len(profile) > 4 {
			perTurn, stations = 4, 6
		}
		height := 0.34
		build := func(squeeze float64) ([]r3.Vec, [][3]int) {
			return planeHelixMesh(profile, 1, 2.5*height*squeeze, perTurn, stations)
		}
		admits := func(squeeze float64) bool {
			verts, tris := build(squeeze)
			return LoftCrossingAudit(proofbound.NewWorkBudget(t.Context()), verts, tris) == nil
		}
		check := func(verts []r3.Vec, tris [][3]int, label string) {
			f, err := checkPlaneAuditMatchesReference(t, verts, tris, label)
			fired += f
			if err != nil {
				refused++
			} else {
				admitted++
			}
		}
		for _, squeeze := range []float64{0.2, 0.4, 0.6, 0.8, 1.0} {
			verts, tris := build(squeeze)
			check(verts, tris, name)
		}
		lo, hi := 0.1, 1.0
		for admits(lo) && lo > 0.002 {
			lo /= 2
		}
		require.False(t, admits(lo), name)
		require.True(t, admits(hi), name)
		for range 16 {
			mid := (lo + hi) / 2
			if admits(mid) {
				hi = mid
				continue
			}
			lo = mid
		}
		edge := lo
		for range 3 {
			edge = math.Nextafter(edge, 0)
		}
		for range 6 {
			verts, tris := build(edge)
			check(verts, tris, name+" threshold")
			edge = math.Nextafter(edge, 2)
		}
		for range 4 {
			verts, tris := build(0.9)
			scale := math.Pow(10, -12+rng.Float64()*12.3)
			k := rng.IntN(len(verts))
			delta := r3.Vec{X: rng.Float64()*2 - 1, Y: rng.Float64()*2 - 1, Z: rng.Float64()*2 - 1}
			verts[k] = verts[k].Add(delta.Scale(scale))
			check(verts, tris, name+" move")
		}
	}
	t.Logf("%d meshes admitted, %d refused, %d pairs decided by the separating plane", admitted, refused, fired)
	require.Positive(t, refused, "no mesh was refused")
	require.Positive(t, admitted, "no mesh was admitted")
	return fired
}

func TestSeparatingPlaneCertificateOffForExactLifts(t *testing.T) {
	t.Parallel()
	verts, tris := planeHelixMesh(planeProfiles()["thread"], 1, 0.9, 8, 12)
	plain := NewLoftAuditData(verts, tris)
	exact := NewLoftAuditDataWithExact(verts, perturbedExact(verts), tris)
	plainFired := 0
	for i := range tris {
		for j := i + 1; j < len(tris); j++ {
			shared, count := tessellation.SharedVertexIndices(tris[i], tris[j])
			if count != 1 {
				continue
			}
			require.False(t, exact.separatingPlaneCertificate(tris[i], tris[j], i, j, shared[0]),
				"pair %d,%d fired on data with exact lifts", i, j)
			if plain.separatingPlaneCertificate(tris[i], tris[j], i, j, shared[0]) {
				plainFired++
			}
		}
	}
	require.Positive(t, plainFired, "the control data never fired, so the exact-lift check proves nothing")
}

// TestSeparatesAtVertexAgreesWithExactSigns feeds separatesAtVertex arbitrary
// planes, including axis-aligned ones whose float sign is zero by the
// zero-column rule, and requires every true answer to be a strict exact
// separation. The plane choice plays no part, so this isolates the proof.
func TestSeparatesAtVertexAgreesWithExactSigns(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(47, 53))
	coord := func(mode int) float64 {
		switch mode {
		case 0:
			return float64(rng.IntN(5) - 2)
		case 1:
			return float64(rng.IntN(9)-4) * math.SmallestNonzeroFloat64
		case 2:
			return float64(rng.IntN(5)-2) * 0x1p-1000
		default:
			return rng.Float64()*2 - 1
		}
	}
	exactSign := func(v, q1, q2, p r3.Vec) int {
		return proof.OrientSignExact(proof.XptOf(v), proof.XptOf(q1), proof.XptOf(q2), proof.XptOf(p))
	}
	fired, zeroSigns := 0, 0
	for range 30000 {
		mode := rng.IntN(4)
		pt := func() r3.Vec { return r3.Vec{X: coord(mode), Y: coord(mode), Z: coord(mode)} }
		v, q1, q2 := pt(), pt(), pt()
		ai, bj := [2]r3.Vec{pt(), pt()}, [2]r3.Vec{pt(), pt()}
		if s, ok := proof.OrientSignFloat(v, q1, q2, ai[0]); ok && s == 0 {
			zeroSigns++
		}
		if !separatesAtVertex(v, q1, q2, ai, bj) {
			continue
		}
		fired++
		sigma := exactSign(v, q1, q2, ai[0])
		require.NotZero(t, sigma, "%v %v %v %v", v, q1, q2, ai)
		require.Equal(t, sigma, exactSign(v, q1, q2, ai[1]), "%v %v %v %v", v, q1, q2, ai)
		require.Equal(t, -sigma, exactSign(v, q1, q2, bj[0]), "%v %v %v %v", v, q1, q2, bj)
		require.Equal(t, -sigma, exactSign(v, q1, q2, bj[1]), "%v %v %v %v", v, q1, q2, bj)
	}
	t.Logf("%d separations, %d certain zero signs among the draws", fired, zeroSigns)
	require.Positive(t, fired)
	require.Positive(t, zeroSigns, "no draw reached the zero-column rule")
}

// TestSeparatesAtVertexDeclinesUnderflowedSign pins the case an unguarded
// orientation filter gets wrong: the z difference of d from a is subnormal, the
// filter used to return the certain sign -1 for d where the exact sign is +1.
// Taking a as the vertex, with d and y both exactly on the + side, an
// unguarded filter pairs d's wrong - with y's correct + and reports a
// separation that does not exist.
func TestSeparatesAtVertexDeclinesUnderflowedSign(t *testing.T) {
	t.Parallel()
	v := r3.Vec{X: 0.25, Y: 0.25, Z: math.SmallestNonzeroFloat64}
	q1 := r3.Vec{X: 1, Y: math.Copysign(0, -1), Z: 0}
	q2 := r3.Vec{X: 0.98046875, Y: -0.1953125, Z: 0}
	d := r3.Vec{X: 0.923828125, Y: -0.3828125, Z: 0}
	y := r3.Vec{X: 0.923828125, Y: -0.3828125, Z: -1}
	exact := func(p r3.Vec) int {
		return proof.OrientSignExact(proof.XptOf(v), proof.XptOf(q1), proof.XptOf(q2), proof.XptOf(p))
	}
	require.Equal(t, 1, exact(d))
	require.Equal(t, 1, exact(y), "the fixture's two corners must lie on the same exact side")
	require.False(t, separatesAtVertex(v, q1, q2, [2]r3.Vec{d, d}, [2]r3.Vec{y, y}))
}
