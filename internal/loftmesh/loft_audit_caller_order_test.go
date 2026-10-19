package loftmesh

import (
	"errors"
	"math/rand/v2"
	"slices"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// callerOrderShortcuts are the shortcut sets every caller-order test runs: the
// exact-only path, the certificates, and the certificates with the sweep.
var callerOrderShortcuts = []LoftAuditShortcuts{
	{Certificates: true},
	{Certificates: true, Sweep: true},
	{},
}

// TestAuditLoftPairDataRefusesForeignCornerOrder builds the table from
// triangles {0,1,2} and {0,3,4}, then passes the caller's {0,1,2} and {3,0,4}.
// Read by position, the caller's second triangle makes vertex 3 the shared
// point, and the separating-plane certificate would admit a pair the exact
// path refuses. The two triangles cross each other away from vertex 0.
func TestAuditLoftPairDataRefusesForeignCornerOrder(t *testing.T) {
	t.Parallel()
	verts := []r3.Vec{{X: -3, Y: 4, Z: -3}, {X: 3, Y: -2, Z: -3}, {X: 4, Y: 3}, {X: -5, Y: 3, Z: -2}, {X: 4, Y: 2, Z: -4}}
	table := [][3]int{{0, 1, 2}, {0, 3, 4}}
	caller := [][3]int{{0, 1, 2}, {3, 0, 4}}
	data := NewLoftAuditData(verts, table)

	// The triangles genuinely cross: their contact is not the shared vertex.
	contact, err := meshbool.TriTriClassify(data.corners(0), data.corners(1), data.xtri(0), data.xtri(1),
		data.Norms[0], data.Norms[1])
	require.NoError(t, err)
	require.False(t, contact.Kind == meshbool.ContactPoint && contact.P0.Key() == data.Xverts[0].Key(),
		"the fixture must cross away from the shared vertex")
	_, refErr := auditLoftPairDataSignedReference(data, table, 0, 1, LoftAuditShortcuts{})
	require.ErrorIs(t, refErr, decaderr.ErrDegenerate, "the exact reference refuses the matching order")

	for _, sc := range callerOrderShortcuts {
		out, err := AuditLoftPairData(data, caller, 0, 1, sc)
		require.ErrorIs(t, err, ErrLoftAuditOrder, "shortcuts %+v returned outcome %v, err %v for a pair that crosses", sc, out, err)
		_, err = AuditLoftPairData(data, table, 0, 1, sc)
		require.ErrorIs(t, err, decaderr.ErrDegenerate, "shortcuts %+v", sc)
	}
}

// TestAuditLoftPairDataCallerOrderRandom audits 9,000 random shared-vertex
// pairs (3,000 point sets, three random caller corner orders each). A caller
// order that equals the table's keeps the exact-only reference's outcome and
// error bit for bit; any other order is refused with ErrLoftAuditOrder.
func TestAuditLoftPairDataCallerOrderRandom(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(1795, 7))
	table := [][3]int{{0, 1, 2}, {0, 3, 4}}
	audits, refused, matched, wrong, differ := 0, 0, 0, 0, 0
	for audits < 9000 {
		var verts []r3.Vec
		for range 5 {
			verts = append(verts, r3.Vec{X: float64(rng.IntN(11) - 5), Y: float64(rng.IntN(11) - 5), Z: float64(rng.IntN(11) - 5)})
		}
		if TriangleCollapsed(verts, table[0]) || TriangleCollapsed(verts, table[1]) {
			continue
		}
		data := NewLoftAuditData(verts, table)
		for range 3 {
			caller := permuteTriples(rng, table)
			sc := callerOrderShortcuts[rng.IntN(len(callerOrderShortcuts))]
			audits++
			got, err := AuditLoftPairData(data, caller, 0, 1, sc)
			if caller[0] != table[0] || caller[1] != table[1] {
				refused++
				if errorsIsOrder(err) {
					continue
				}
				wrong++
				// Count the audits whose verdict also differs from the exact
				// reference run on the table's own order.
				want, wantErr := auditLoftPairDataSignedReference(data, table, 0, 1, LoftAuditShortcuts{})
				if (wantErr == nil) != (err == nil) || (err == nil && !sc.Certificates && want != got) {
					differ++
				}
				continue
			}
			matched++
			want, wantErr := auditLoftPairDataSignedReference(data, table, 0, 1, sc)
			if sc.Certificates {
				// The certificates may decide sooner but never differ in verdict.
				require.Equal(t, wantErr == nil, err == nil, "verts %v", verts)
				continue
			}
			require.Equal(t, want, got, "verts %v", verts)
			if wantErr != nil {
				require.EqualError(t, err, wantErr.Error(), "verts %v", verts)
				continue
			}
			require.NoError(t, err, "verts %v", verts)
		}
	}
	require.Zero(t, wrong, "of %d mismatched-order audits, %d were not refused with ErrLoftAuditOrder "+
		"and %d of those differ from the exact reference", refused, wrong, differ)
	require.Positive(t, refused)
	require.Positive(t, matched)
}

func errorsIsOrder(err error) bool { return errors.Is(err, ErrLoftAuditOrder) }

// callerOrderLayouts builds the audit table of tris in each layout. Every
// builder is handed the caller's slice, which the test then mutates.
var callerOrderLayouts = map[string]func(verts []r3.Vec, tris [][3]int) *LoftAuditData{
	"full":    NewLoftAuditData,
	"compact": newCompactLoftAuditData,
	"exact": func(verts []r3.Vec, tris [][3]int) *LoftAuditData {
		exact := make([]proof.Xpt, len(verts))
		for i, v := range verts {
			exact[i] = proof.XptOf(v)
		}
		return NewLoftAuditDataWithExact(verts, exact, tris)
	},
	"selected": func(verts []r3.Vec, tris [][3]int) *LoftAuditData {
		return newLoftAuditDataForCandidates(verts, tris, loftCandidates{starts: []int{0, 1, 1}, js: []int32{1}})
	},
}

// TestLoftAuditTableOwnsItsCornerOrder mutates the caller's triangle list in
// place after the table is built. The table keeps its own copy, so a list in
// the new order is refused, a list in the old order keeps its outcome, and
// writing one corner is the same as replacing the triple.
func TestLoftAuditTableOwnsItsCornerOrder(t *testing.T) {
	t.Parallel()
	verts := []r3.Vec{{X: -3, Y: 4, Z: -3}, {X: 3, Y: -2, Z: -3}, {X: 4, Y: 3}, {X: -5, Y: 3, Z: -2}, {X: 4, Y: 2, Z: -4}}
	original := [][3]int{{0, 1, 2}, {0, 3, 4}}
	mutations := map[string]func(tris [][3]int){
		"replace triple": func(tris [][3]int) { tris[1] = [3]int{3, 0, 4} },
		"write corners":  func(tris [][3]int) { tris[1][0], tris[1][1] = 3, 0 },
	}
	for layout, build := range callerOrderLayouts {
		for mutation, mutate := range mutations {
			t.Run(layout+"/"+mutation, func(t *testing.T) {
				t.Parallel()
				tris := slices.Clone(original)
				data := build(verts, tris)
				for _, sc := range callerOrderShortcuts {
					before, beforeErr := AuditLoftPairData(data, original, 0, 1, sc)
					mutate(tris)
					out, err := AuditLoftPairData(data, tris, 0, 1, sc)
					require.ErrorIs(t, err, ErrLoftAuditOrder, "shortcuts %+v: outcome %v, err %v", sc, out, err)
					after, afterErr := AuditLoftPairData(data, slices.Clone(original), 0, 1, sc)
					require.Equal(t, before, after)
					require.Equal(t, beforeErr, afterErr)
					tris = slices.Clone(original)
				}
			})
		}
	}
}

// TestLoftAuditFansOwnTheirCornerOrder rotates every triple of the caller's
// list in place after the table (and its fan table) is built. A list in the
// original order must keep every fan decision, so the fan table cannot read
// the caller's list.
func TestLoftAuditFansOwnTheirCornerOrder(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(61, 67))
	verts, tris := fanFixture(rng, []r3.Vec{{Z: 2}}, 24, 40, 3)
	for layout, build := range map[string]func([]r3.Vec, [][3]int) *LoftAuditData{
		"full": NewLoftAuditData, "compact": newCompactLoftAuditData,
	} {
		original := slices.Clone(tris)
		caller := slices.Clone(tris)
		data := build(verts, caller)
		require.NotNil(t, data.fans, layout)
		type verdict struct {
			out        LoftPairOutcome
			err        string
			fan, fanOK bool
		}
		decide := func(list [][3]int) map[[2]int]verdict {
			out := map[[2]int]verdict{}
			for i := range list {
				for j := i + 1; j < len(list); j++ {
					if _, n := tessellation.SharedVertexIndices(list[i], list[j]); n != 1 {
						continue
					}
					o, err := AuditLoftPairData(data, list, i, j, LoftAuditShortcuts{Certificates: true})
					shared, _ := tessellation.SharedVertexIndices(list[i], list[j])
					v := verdict{out: o}
					v.fan, v.fanOK = data.fans.vertexCertificate(i, j, shared[0])
					if err != nil {
						v.err = err.Error()
					}
					out[[2]int{i, j}] = v
				}
			}
			return out
		}
		before := decide(original)
		require.NotEmpty(t, before)
		for n, tri := range caller {
			caller[n] = [3]int{tri[1], tri[2], tri[0]}
		}
		require.Equal(t, before, decide(slices.Clone(original)), layout)
	}
}

// ownedVertexFixture is two triangles that overlap over a region: they share
// the edge (0,0)-(2,0) and both lie on the plane z = 0.
func ownedVertexFixture() ([]r3.Vec, [][3]int) {
	return []r3.Vec{{}, {X: 2}, {Y: 2}, {X: 1, Y: 1}}, [][3]int{{0, 1, 2}, {0, 1, 3}}
}

// ownedLayouts builds each layout from the caller's verts, an exact slice of
// proof.XptOf(verts) that the caller may later mutate, and tris.
var ownedLayouts = map[string]func(verts []r3.Vec, exact []proof.Xpt, tris [][3]int) *LoftAuditData{
	"full":    func(v []r3.Vec, _ []proof.Xpt, t [][3]int) *LoftAuditData { return NewLoftAuditData(v, t) },
	"compact": func(v []r3.Vec, _ []proof.Xpt, t [][3]int) *LoftAuditData { return newCompactLoftAuditData(v, t) },
	"exact":   NewLoftAuditDataWithExact,
	"selected": func(v []r3.Vec, _ []proof.Xpt, t [][3]int) *LoftAuditData {
		return newLoftAuditDataForCandidates(v, t, loftCandidates{starts: []int{0, 1, 1}, js: []int32{1}})
	},
}

// ownedAudit is the audit the owned-vertex tests repeat after a mutation.
func ownedAudit(data *LoftAuditData, tris [][3]int) (LoftPairOutcome, error) {
	return AuditLoftPairData(data, tris, 0, 1, LoftAuditShortcuts{Certificates: true, ExactInput: true})
}

// TestLoftAuditTableOwnsItsVertices changes the caller's vertex after the
// table is built. The table's exact points, triangles and projections were
// built from the old vertex, so the audit must keep refusing the overlap.
func TestLoftAuditTableOwnsItsVertices(t *testing.T) {
	t.Parallel()
	for layout, build := range ownedLayouts {
		t.Run(layout, func(t *testing.T) {
			t.Parallel()
			verts, tris := ownedVertexFixture()
			exact := make([]proof.Xpt, len(verts))
			for i, v := range verts {
				exact[i] = proof.XptOf(v)
			}
			data := build(verts, exact, tris)
			_, err := ownedAudit(data, tris)
			require.ErrorIs(t, err, decaderr.ErrDegenerate)
			verts[3].Z = 1
			out, err := ownedAudit(data, tris)
			require.ErrorIs(t, err, decaderr.ErrDegenerate, "outcome %v, err %v after verts[3].Z = 1", out, err)
		})
	}
}

// TestLoftAuditExactTableOwnsItsPoints replaces a caller exact point, and
// writes into another point's integer, after NewLoftAuditDataWithExact. Every
// cached exact triple, normal and homogeneous projection keeps its value.
func TestLoftAuditExactTableOwnsItsPoints(t *testing.T) {
	t.Parallel()
	mutations := map[string]func(exact []proof.Xpt){
		"replace point": func(exact []proof.Xpt) { exact[3] = proof.XptOf(r3.Vec{X: 1, Y: 1, Z: 1}) },
		"write integer": func(exact []proof.Xpt) { exact[3].Z.SetInt64(1) },
	}
	for name, mutate := range mutations {
		t.Run(name, func(t *testing.T) {
			t.Parallel()
			verts, tris := ownedVertexFixture()
			exact := make([]proof.Xpt, len(verts))
			for i, v := range verts {
				exact[i] = proof.XptOf(v)
			}
			data := NewLoftAuditDataWithExact(verts, exact, tris)
			snapshot := func() []string {
				var out []string
				for _, p := range data.Xverts {
					out = append(out, p.Key())
				}
				for i := range tris {
					for k := range 3 {
						out = append(out, data.Xtris[i][k].Key(), data.Projections[i][k].Hu.String(),
							data.Projections[i][k].Hv.String(), data.Projections[i][k].Hw.String())
					}
				}
				return out
			}
			before := snapshot()
			_, err := ownedAudit(data, tris)
			require.ErrorIs(t, err, decaderr.ErrDegenerate)
			mutate(exact)
			require.Equal(t, before, snapshot())
			out, err := ownedAudit(data, tris)
			require.ErrorIs(t, err, decaderr.ErrDegenerate, "outcome %v, err %v after the mutation", out, err)
		})
	}
}
