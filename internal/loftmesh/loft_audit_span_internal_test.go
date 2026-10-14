package loftmesh

import (
	"math/big"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// requireSameAuditOutcome checks one pair: the audit that reads the contact
// span only must return the outcome and the error text of the reference that
// reads the full TriContact.
func requireSameAuditOutcome(t *testing.T, data *LoftAuditData, tris [][3]int, i, j int,
	shortcuts LoftAuditShortcuts, label string) {
	t.Helper()
	wantOut, wantErr := auditLoftPairDataSignedReference(data, tris, i, j, shortcuts)
	gotOut, gotErr := auditLoftPairDataSigned(data, tris, i, j, shortcuts, nil, nil)
	if shared, count := tessellation.SharedVertexIndices(tris[i], tris[j]); shortcuts.Certificates && count == 1 &&
		wantOut == LoftPairClassified && wantErr == nil &&
		data.separatingPlaneCertificate(tris[i], tris[j], i, j, shared[0]) {
		// The separating plane admits a pair the reference classifies and
		// admits: the only allowed difference.
		wantOut = LoftPairVertexCertificate
	}
	require.Equal(t, wantOut, gotOut, "%s pair %d,%d", label, i, j)
	if wantErr == nil {
		require.NoError(t, gotErr, "%s pair %d,%d", label, i, j)
		return
	}
	require.EqualError(t, gotErr, wantErr.Error(), "%s pair %d,%d", label, i, j)
}

// TestAuditLoftPairSpanMatchesReference compares auditLoftPairDataSigned,
// which reads the contact span only, with the reference that reads the full
// TriContact, on every pair of four fixtures, with no shortcut (every pair
// reaches the exact classification) and with the generic shortcuts. Without
// it a swapped end point leaves the rest of the package green.
func TestAuditLoftPairSpanMatchesReference(t *testing.T) {
	t.Parallel()
	counts := [3]int{}
	for _, f := range vertexFixtures() {
		switch f.name {
		case "prism", "soup", "soup-flat", "fan-antipodal-mixed":
		default:
			continue
		}
		for _, data := range []*LoftAuditData{NewLoftAuditData(f.verts, f.tris)} {
			for i := range f.tris {
				for j := i + 1; j < len(f.tris); j++ {
					_, shared := tessellation.SharedVertexIndices(f.tris[i], f.tris[j])
					if shared > 2 {
						continue
					}
					counts[shared]++
					for _, shortcuts := range []LoftAuditShortcuts{{}, loftGenericShortcuts} {
						requireSameAuditOutcome(t, data, f.tris, i, j, shortcuts, f.name)
					}
				}
			}
		}
	}
	t.Logf("pairs by shared vertex count: %v", counts)
	for k, n := range counts {
		require.Positive(t, n, "no pair shares %d vertices", k)
	}
}

// xptBitsEqual reports whether two exact points hold the same four integers,
// not merely the same rational point.
func xptBitsEqual(a, b proof.Xpt) bool {
	same := func(x, y *big.Int) bool {
		if x == nil || y == nil {
			return x == nil && y == nil
		}
		return x.Cmp(y) == 0
	}
	return same(a.X, b.X) && same(a.Y, b.Y) && same(a.Z, b.Z) && same(a.W, b.W)
}

// requireSameSpan calls TriTriClassifyCore and TriTriContactSpanCore with
// the arguments the audit builds for the pair and requires the same error,
// the same kind and bit-identical end points.
func requireSameSpan(t *testing.T, data *LoftAuditData, i, j int, exactInput bool, label string) {
	t.Helper()
	ta, tb := data.corners(i), data.corners(j)
	xta, xtb := data.xtri(i), data.xtri(j)
	na, nb := data.Norms[i], data.Norms[j]
	signsB := data.planeSigns(i, xtb, tb)
	signsA := data.planeSigns(j, xta, ta)
	var projA, projB *[3]meshbool.Xp2
	var compactA, compactB [3]meshbool.Xp2
	if data.compact {
		compactA, compactB = data.projectionTriple(i), data.projectionTriple(j)
		projA, projB = &compactA, &compactB
	} else {
		projA, projB = &data.Projections[i], &data.Projections[j]
	}
	full, ferr := meshbool.TriTriClassifyCore(ta, tb, xta, xtb, na, nb, projA, projB, &signsA, &signsB,
		!exactInput, data.floatNorms[i], data.floatNorms[j], true)
	span, serr := meshbool.TriTriContactSpanCore(ta, tb, xta, xtb, na, nb, projA, projB, &signsA, &signsB,
		!exactInput, data.floatNorms[i], data.floatNorms[j], true)
	if ferr != nil {
		require.EqualError(t, serr, ferr.Error(), label)
		return
	}
	require.NoError(t, serr, label)
	require.Equal(t, full.Kind, span.Kind, label)
	require.True(t, xptBitsEqual(full.P0, span.P0), "%s: P0", label)
	require.True(t, xptBitsEqual(full.P1, span.P1), "%s: P1", label)
}

// spanCase is a two-triangle mesh: the triangles' vertex indices say how many
// vertices they share.
type spanCase struct {
	name  string
	verts []r3.Vec
	tris  [][3]int
}

// spanAdversarialCases builds two-triangle meshes of every contact the audit
// distinguishes, each at several scales, in both winding orders and in both
// argument orders.
func spanAdversarialCases(rng *rand.Rand) []spanCase {
	grid := func() r3.Vec {
		return r3.NewVec(float64(rng.IntN(5)-2), float64(rng.IntN(5)-2), float64(rng.IntN(5)-2))
	}
	flat := func() r3.Vec { return r3.NewVec(float64(rng.IntN(5)-2), float64(rng.IntN(5)-2), 0) }
	type shape struct {
		name string
		tris [][3]int
		pick func() r3.Vec
	}
	// Index layouts: 0 shared (6 vertices), 1 shared (5), 2 shared (4).
	disjoint := [][3]int{{0, 1, 2}, {3, 4, 5}}
	vertex := [][3]int{{0, 1, 2}, {0, 3, 4}}
	edge := [][3]int{{0, 1, 2}, {1, 0, 3}}
	shapes := []shape{
		{"disjoint", disjoint, grid}, {"disjoint-flat", disjoint, flat},
		{"vertex", vertex, grid}, {"vertex-flat", vertex, flat},
		{"edge", edge, grid}, {"edge-flat", edge, flat},
	}
	var out []spanCase
	for _, scale := range []float64{1, 1e-9, 1e12, 1e150} {
		for _, sh := range shapes {
			for range 20 {
				n := 0
				for _, tri := range sh.tris {
					for _, v := range tri {
						n = max(n, v+1)
					}
				}
				verts := make([]r3.Vec, n)
				for k := range verts {
					p := sh.pick()
					verts[k] = r3.NewVec(p.X*scale, p.Y*scale, p.Z*scale)
				}
				if TriangleCollapsed(verts, sh.tris[0]) || TriangleCollapsed(verts, sh.tris[1]) {
					continue
				}
				for _, reverse := range []bool{false, true} {
					for _, swap := range []bool{false, true} {
						tris := [][3]int{sh.tris[0], sh.tris[1]}
						if reverse {
							tris[0][1], tris[0][2] = tris[0][2], tris[0][1]
						}
						if swap {
							tris[0], tris[1] = tris[1], tris[0]
						}
						out = append(out, spanCase{name: sh.name, verts: verts, tris: tris})
					}
				}
			}
		}
	}
	return out
}

// TestAuditLoftPairSpanMatchesReferenceAdversarial runs random small pairs
// whose contacts are a shared vertex, a shared edge, a coplanar overlap, a
// touch, a crossing and a gap, at scales from 1e-9 to 1e150, in both
// windings and both argument orders. For each it requires the audit to match
// the reference, and the span to equal the full classification's kind and end
// points bit for bit.
func TestAuditLoftPairSpanMatchesReferenceAdversarial(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(71, 73))
	kinds := map[meshbool.ContactKind]int{}
	for _, c := range spanAdversarialCases(rng) {
		for _, data := range []*LoftAuditData{NewLoftAuditData(c.verts, c.tris), newCompactLoftAuditData(c.verts, c.tris)} {
			for _, shortcuts := range []LoftAuditShortcuts{{}, loftGenericShortcuts} {
				requireSameAuditOutcome(t, data, c.tris, 0, 1, shortcuts, c.name)
			}
			requireSameSpan(t, data, 0, 1, false, c.name)
			requireSameSpan(t, data, 0, 1, true, c.name)
			ta, tb := data.xtri(0), data.xtri(1)
			sp, err := meshbool.TriTriContactSpanCore(data.corners(0), data.corners(1), ta, tb, data.Norms[0], data.Norms[1],
				nil, nil, nil, nil, true, data.floatNorms[0], data.floatNorms[1], true)
			if err == nil {
				kinds[sp.Kind]++
			}
		}
	}
	t.Logf("contacts by kind: %v", kinds)
	for _, k := range []meshbool.ContactKind{meshbool.ContactNone, meshbool.ContactPoint,
		meshbool.ContactSegment, meshbool.ContactRegion} {
		require.Positive(t, kinds[k], "no pair produced contact kind %v", k)
	}
}
