package loftmesh

import (
	"context"
	"math"
	"math/big"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// vertexFixture is one mesh the float vertex certificate is compared on.
type vertexFixture struct {
	name  string
	verts []r3.Vec
	tris  [][3]int
}

// flatFan is a fan of rim points around one center at height z. Rim angles
// come from angles; both windings alternate when mixed is set.
func flatFan(name string, angles []float64, radii []float64, z float64, mixed bool) vertexFixture {
	f := vertexFixture{name: name, verts: []r3.Vec{{X: 0, Y: 0, Z: z}}}
	for i, a := range angles {
		r := radii[i%len(radii)]
		f.verts = append(f.verts, r3.Vec{X: r * math.Cos(a), Y: r * math.Sin(a), Z: z})
	}
	n := len(angles)
	for k := range n {
		tri := [3]int{0, 1 + k, 1 + (k+1)%n}
		if mixed && k%2 == 1 {
			tri[1], tri[2] = tri[2], tri[1]
		}
		if TriangleCollapsed(f.verts, tri) {
			continue
		}
		f.tris = append(f.tris, tri)
	}
	return f
}

func uniformAngles(n int) []float64 {
	a := make([]float64, n)
	for i := range a {
		a[i] = 2 * math.Pi * float64(i) / float64(n)
	}
	return a
}

// prism is a closed n-gon prism: two fan caps and a wall of quads.
func prism(n int, radius, height float64) vertexFixture {
	f := vertexFixture{name: "prism"}
	f.verts = append(f.verts, r3.Vec{Z: 0}, r3.Vec{Z: height})
	for _, z := range []float64{0, height} {
		for i := range n {
			a := 2 * math.Pi * float64(i) / float64(n)
			f.verts = append(f.verts, r3.Vec{X: radius * math.Cos(a), Y: radius * math.Sin(a), Z: z})
		}
	}
	for i := range n {
		j := (i + 1) % n
		b0, b1, t0, t1 := 2+i, 2+j, 2+n+i, 2+n+j
		f.tris = append(f.tris, [3]int{0, b1, b0}, [3]int{1, t0, t1}, [3]int{b0, b1, t1}, [3]int{b0, t1, t0})
	}
	return f
}

// uvSphere is a closed latitude/longitude sphere mesh.
func uvSphere(rings, sectors int) vertexFixture {
	f := vertexFixture{name: "sphere"}
	f.verts = append(f.verts, r3.Vec{Z: 1}, r3.Vec{Z: -1})
	for r := 1; r < rings; r++ {
		phi := math.Pi * float64(r) / float64(rings)
		for s := range sectors {
			th := 2 * math.Pi * float64(s) / float64(sectors)
			f.verts = append(f.verts, r3.Vec{X: math.Sin(phi) * math.Cos(th), Y: math.Sin(phi) * math.Sin(th),
				Z: math.Cos(phi)})
		}
	}
	at := func(r, s int) int { return 2 + (r-1)*sectors + s%sectors }
	for s := range sectors {
		f.tris = append(f.tris, [3]int{0, at(1, s), at(1, s+1)}, [3]int{1, at(rings-1, s+1), at(rings-1, s)})
	}
	for r := 1; r < rings-1; r++ {
		for s := range sectors {
			f.tris = append(f.tris, [3]int{at(r, s), at(r+1, s), at(r+1, s+1)},
				[3]int{at(r, s), at(r+1, s+1), at(r, s+1)})
		}
	}
	return f
}

// randomSoup draws triangles from a small vertex pool, so many pairs share
// exactly one vertex. scale shrinks the pool to build tiny wedges, and flat
// puts every vertex within a 1e-15 band around one plane.
func randomSoup(rng *rand.Rand, name string, pool, count int, scale, flat float64) vertexFixture {
	f := vertexFixture{name: name}
	for range pool {
		z := rng.Float64()*2 - 1
		if flat > 0 {
			z = flat * float64(rng.IntN(3)-1)
		}
		f.verts = append(f.verts, r3.Vec{X: scale * (rng.Float64()*2 - 1), Y: scale * (rng.Float64()*2 - 1),
			Z: scale * z})
	}
	for len(f.tris) < count {
		tri := [3]int{rng.IntN(pool), rng.IntN(pool), rng.IntN(pool)}
		if tri[0] == tri[1] || tri[1] == tri[2] || tri[0] == tri[2] || TriangleCollapsed(f.verts, tri) {
			continue
		}
		f.tris = append(f.tris, tri)
	}
	return f
}

func vertexFixtures() []vertexFixture {
	rng := rand.New(rand.NewPCG(43, 47))
	var out []vertexFixture
	for _, n := range []int{40, 96} {
		out = append(out, flatFan("fan", uniformAngles(n), []float64{1}, 0, false),
			flatFan("fan-mixed", uniformAngles(n), []float64{1}, 3, true))
	}
	// Rims with exactly antipodal points, collinear points (same direction at
	// two radii) and duplicated directions.
	angles := uniformAngles(40)
	angles = append(angles[:10], append([]float64{angles[10], angles[10] + math.Pi, angles[10]}, angles[10:]...)...)
	out = append(out,
		flatFan("fan-antipodal", angles, []float64{1, 2, 1, 0.5}, 0, false),
		flatFan("fan-antipodal-mixed", angles, []float64{1, 1, 3}, -1, true),
		flatFan("fan-offset", uniformAngles(60), []float64{1}, 1e-15, true),
		prism(24, 2, 3), prism(37, 1, 0.001), uvSphere(6, 10),
		randomSoup(rng, "soup", 16, 70, 1, 0),
		randomSoup(rng, "soup-flat", 12, 50, 1, 1e-15),
		randomSoup(rng, "soup-tiny", 12, 50, 1e-9, 0),
		randomSoup(rng, "soup-tiny-flat", 12, 50, 1e-6, 1e-15))
	return out
}

// perturbedExact lifts the vertices to exact points that differ from the
// floats, as a rational construction's lifts do.
func perturbedExact(verts []r3.Vec) []proof.Xpt {
	out := make([]proof.Xpt, len(verts))
	for i, v := range verts {
		x := proof.XptOf(v)
		k := big.NewInt(1 << 20)
		out[i] = proof.Xpt{
			X: new(big.Int).Add(new(big.Int).Mul(x.X, k), big.NewInt(int64(i%3+1))),
			Y: new(big.Int).Mul(x.Y, k), Z: new(big.Int).Mul(x.Z, k), W: new(big.Int).Mul(x.W, k),
		}
	}
	return out
}

// checkVertexPairs checks every pair of the fixture that shares
// exactly one vertex against the reference and returns how many pairs the
// float certificate decided.
func checkVertexPairs(t *testing.T, f vertexFixture, data *LoftAuditData, label string) (int, int) {
	t.Helper()
	decided, undecided := 0, 0
	for i := range f.tris {
		for j := i + 1; j < len(f.tris); j++ {
			shared, count := tessellation.SharedVertexIndices(f.tris[i], f.tris[j])
			if count != 1 {
				continue
			}
			plane := data.separatingPlaneCertificate(f.tris[i], f.tris[j], i, j, shared[0])
			if data.exactLifts {
				require.False(t, plane, "%s %s pair %d,%d: separating plane on data with exact lifts", f.name, label, i, j)
			}
			for _, shortcuts := range []LoftAuditShortcuts{{}, loftGenericShortcuts} {
				wantOut, wantErr := auditLoftPairDataSignedReference(data, f.tris, i, j, shortcuts)
				gotOut, gotErr := auditLoftPairDataSigned(data, f.tris, i, j, shortcuts, nil, nil)
				if shortcuts.Certificates && plane && wantOut == LoftPairClassified && wantErr == nil {
					// The separating plane admits a pair the reference
					// classifies and admits: the only allowed difference.
					wantOut = LoftPairVertexCertificate
				}
				require.Equal(t, wantOut, gotOut, "%s %s pair %d,%d", f.name, label, i, j)
				if wantErr == nil {
					require.NoError(t, gotErr, "%s %s pair %d,%d", f.name, label, i, j)
				} else {
					require.EqualError(t, gotErr, wantErr.Error(), "%s %s pair %d,%d", f.name, label, i, j)
				}
			}
			if data.vertexCertificateFloat(f.tris, i, j, shared[0]) {
				decided++
				// A float decision is a certificate: the exact path must
				// reach the same one.
				out, err := auditLoftPairDataSignedReference(data, f.tris, i, j, loftGenericShortcuts)
				require.NoError(t, err, "%s %s pair %d,%d", f.name, label, i, j)
				require.Equal(t, LoftPairVertexCertificate, out, "%s %s pair %d,%d", f.name, label, i, j)
				continue
			}
			undecided++
		}
	}
	return decided, undecided
}

func TestVertexCertificateFloatMatchesExactPath(t *testing.T) {
	t.Parallel()
	total, fallbacks := 0, 0
	for _, f := range vertexFixtures() {
		full := NewLoftAuditData(f.verts, f.tris)
		compact := newCompactLoftAuditData(f.verts, f.tris)
		exact := NewLoftAuditDataWithExact(f.verts, perturbedExact(f.verts), f.tris)
		decided, undecided := checkVertexPairs(t, f, full, "full")
		total += decided
		fallbacks += undecided
		t.Logf("%s: %d triangles, %d pairs decided in floats, %d left to the exact path",
			f.name, len(f.tris), decided, undecided)
		checkVertexPairs(t, f, compact, "compact")
		exactDecided, _ := checkVertexPairs(t, f, exact, "exact")
		require.Zero(t, exactDecided, "%s: float decision on data with exact lifts", f.name)
	}
	require.Positive(t, total, "the float certificate decided no pair")
	require.Positive(t, fallbacks, "no pair reached the exact fallback")
}

func TestVertexCertificateFloatKeepsAuditWork(t *testing.T) {
	t.Parallel()
	for _, f := range vertexFixtures() {
		if f.name != "fan" && f.name != "prism" && f.name != "sphere" && f.name != "soup" {
			continue
		}
		data := NewLoftAuditData(f.verts, f.tris)
		vertexCerts, edgeCerts, refused := 0, 0, false
		for i := range f.tris {
			for j := i + 1; j < len(f.tris); j++ {
				_, count := tessellation.SharedVertexIndices(f.tris[i], f.tris[j])
				if count == 0 {
					continue
				}
				out, err := auditLoftPairDataSignedReference(data, f.tris, i, j, loftGenericShortcuts)
				refused = refused || err != nil
				if out == LoftPairClassified && err == nil && count == 1 {
					shared, _ := tessellation.SharedVertexIndices(f.tris[i], f.tris[j])
					if data.separatingPlaneCertificate(f.tris[i], f.tris[j], i, j, shared[0]) {
						out = LoftPairVertexCertificate
					}
				}
				switch out {
				case LoftPairVertexCertificate:
					vertexCerts++
				case LoftPairEdgeCertificate:
					edgeCerts++
				case LoftPairClassified:
				}
			}
		}
		work, err := LoftCrossingAuditWork(proofbound.NewWorkBudget(context.Background()), f.verts, f.tris,
			loftGenericShortcuts)
		require.Equal(t, refused, err != nil, f.name)
		if err != nil {
			continue
		}
		require.Equal(t, vertexCerts, work.VertexCerts, f.name)
		require.Equal(t, edgeCerts, work.EdgeCerts, f.name)
	}
}

func TestVertexCertificateFloatDenseTablesMatchProjections(t *testing.T) {
	t.Parallel()
	for _, f := range vertexFixtures() {
		for _, d := range []*LoftAuditData{NewLoftAuditData(f.verts, f.tris), newCompactLoftAuditData(f.verts, f.tris)} {
			for i, tri := range f.tris {
				for corner, vi := range tri {
					x := d.projection(i, corner)
					p := d.certAxes[i]
					require.Equal(t, math.Float64bits(x.Fu), math.Float64bits(d.projUV[p][2*vi]), f.name)
					require.Equal(t, math.Float64bits(x.Fv), math.Float64bits(d.projUV[p][2*vi+1]), f.name)
					require.Equal(t, x.FloatFinite, d.projFinite[p][vi], f.name)
				}
			}
		}
	}
}

// auditLoftPairDataSignedReference is auditLoftPairDataSigned without the float vertex
// certificate: the exact path every float decision is compared against.
func auditLoftPairDataSignedReference(data *LoftAuditData, tris [][3]int, i, j int, shortcuts LoftAuditShortcuts) (LoftPairOutcome, error) {
	ta := data.corners(i)
	tb := data.corners(j)
	xta := data.xtri(i)
	xtb := data.xtri(j)
	na := data.Norms[i]
	nb := data.Norms[j]
	shared, sharedCount := tessellation.SharedVertexIndices(tris[i], tris[j])
	if sharedCount == 2 {
		edgeA, edgeB := data.Xverts[shared[0]], data.Xverts[shared[1]]
		apex := data.Xverts[tessellation.TriangleApexIndex(tris[j], shared[0], shared[1])]
		if proof.XdotSign(na, proof.Xsub(apex, edgeA)) == 0 {
			if TriTriCoplanarSharedEdge(xta, xtb, na, edgeA, edgeB) {
				return LoftPairClassified, nil
			}
			return LoftPairClassified, ErrLoftContact(i, j, "do not meet exactly along their recorded shared edge")
		}
		if shortcuts.Certificates {
			return LoftPairEdgeCertificate, nil
		}
	}

	signsB := data.planeSigns(i, xtb, tb)
	if shortcuts.Certificates && sharedCount == 1 && meshbool.CountZero(signsB) == 3 &&
		CoplanarIsolatedSharedVertex(data, tris[i], tris[j], i, j, shared[0]) {
		return LoftPairVertexCertificate, nil
	}
	if shortcuts.Certificates && sharedCount == 1 && IsolatedSharedVertex(tris[j], shared[0], signsB) {
		return LoftPairVertexCertificate, nil
	}
	signsA := data.planeSigns(j, xta, ta)
	if shortcuts.Certificates && sharedCount == 1 && IsolatedSharedVertex(tris[i], shared[0], signsA) {
		return LoftPairVertexCertificate, nil
	}
	var projA, projB *[3]meshbool.Xp2
	var compactA, compactB [3]meshbool.Xp2
	if data.compact {
		compactA, compactB = data.projectionTriple(i), data.projectionTriple(j)
		projA, projB = &compactA, &compactB
	} else {
		projA, projB = &data.Projections[i], &data.Projections[j]
	}
	contact, err := meshbool.TriTriClassifyCore(ta, tb, xta, xtb, na, nb,
		projA, projB, &signsA, &signsB, !shortcuts.ExactInput,
		data.floatNorms[i], data.floatNorms[j], true)
	if err != nil {
		return LoftPairClassified, err
	}

	switch sharedCount {
	case 0:
		if contact.Kind == meshbool.ContactNone {
			return LoftPairClassified, nil
		}
		return LoftPairClassified, ErrLoftContact(i, j, "share no recorded vertex, but make contact")
	case 1:
		v := data.Xverts[shared[0]]
		if contact.Kind == meshbool.ContactPoint && contact.P0.Key() == v.Key() {
			return LoftPairClassified, nil
		}
		return LoftPairClassified, ErrLoftContact(i, j, "do not meet exactly at their recorded shared vertex")
	case 2:
		edgeA, edgeB := data.Xverts[shared[0]], data.Xverts[shared[1]]
		if SegMatchesRecordedEdge(contact, edgeA, edgeB) {
			return LoftPairClassified, nil
		}
		if contact.Kind == meshbool.ContactRegion && TriTriCoplanarSharedEdge(xta, xtb, na, edgeA, edgeB) {
			return LoftPairClassified, nil
		}
		return LoftPairClassified, ErrLoftContact(i, j, "do not meet exactly along their recorded shared edge")
	default:
		return LoftPairClassified, ErrLoftContact(i, j, "share an unexpected vertex count")
	}
}
